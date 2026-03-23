#!/usr/bin/env python3
"""Interfejs sprzętowy robota Unitree G1 EDU.

MIEJSCE TEGO MODUŁU W ARCHITEKTURZE:
=====================================
Ten plik to „tłumacz" między abstrakcyjnym językiem RoboMVP
(„przesuń się do pozy {'x': 0.5, 'y': 1.0, 'yaw': 0}") a konkretnym
protokołem DDS Unitree SDK 2 (SetVelocity z prędkością i czasem).

Cały reszta kodu (StateMachine, motion_sequences, main_node) nie wie
nic o SDK. Gdybyśmy chcieli użyć innego robota, wystarczyłoby zastąpić
ten jeden plik. To jest wzorzec projektowy „Adapter".

JAK DZIAŁA STEROWANIE PRĘDKOŚCIĄ (LOCOCLIENT)?
================================================
G1 EDU używa trybu Sport Mode (wysokiego poziomu): zamiast sterować
każdym ze stawów nogi osobno (12 stawów × 4 nogi = 48 DOF!), wysyłamy
trzy liczby: vx (do przodu), vy (boczny) i omega/vyaw (obrót).
Firmware robota sam oblicza sekwencję kroków – jak dać prędkość
samochodowi zamiast sterować każdym tłokiem silnika.

KOMENDA: LocoClient.SetVelocity(vx, vy, omega, duration)
    vx      – prędkość do przodu [m/s]       (+ = przód)
    vy      – prędkość boczna [m/s]           (+ = lewo)
    omega   – prędkość kątowa [rad/s]         (+ = obrót w lewo)
    duration – czas wykonania po stronie SDK  [s]

Po wysłaniu SetVelocity MUSIMY jeszcze czekać duration sekund po stronie
naszego kodu – SDK wykona ruch asynchronicznie na robocie, ale nasz
proces musi dać mu czas. Stąd time.sleep(duration_s).

KONWERSJA POZY NA PRĘDKOŚĆ:
============================
Sekwencja poz (lista słowników) opisuje GDZIE robot ma dotrzeć.
UnitreeRobotAPI przelicza różnicę między bieżącą a docelową pozą
na prędkości i czas trwania ruchu:

    odległość = sqrt(dx² + dy²)
    czas = odległość / LINEAR_SPEED
    vx = (dy/odległość) * LINEAR_SPEED  # komponent do przodu
    vy = (dx/odległość) * LINEAR_SPEED  # komponent boczny

Wymagania (tylko tryb robot):
    pip install unitree_sdk2py
"""

import math
import time


class UnitreeRobotAPI:
    """Interfejs sprzętowy dla robota Unitree G1 EDU.

    Zarządza połączeniem z robotem przez Unitree SDK 2 i udostępnia
    metody do sterowania lokomocją i ramionami.

    Konwencja osi waypoints (spójna z układem TF robota):
        x  – ruch boczny [m]      (+ = lewo)
        y  – ruch do przodu [m]   (+ = przód)
        z  – wysokość ramienia [m] (używana przez G1ArmActionClient)
        yaw – orientacja [rad]    (+ = obrót w lewo, 0 = kierunek startowy)
    """

    # ---------------------------------------------------------------
    # Parametry sterowania – wszystkie w jednym miejscu, łatwe do tuningu
    # ---------------------------------------------------------------
    # Prędkości dobrane konserwatywnie dla bezpieczeństwa demonstracji.
    # Na otwartej przestrzeni można zwiększyć LINEAR_SPEED do ~0.5 m/s.
    _LINEAR_SPEED: float = 0.3   # m/s – prędkość translacyjna
    _YAW_SPEED: float = 0.5      # rad/s – prędkość obrotowa

    # Tolerancje: poniżej tych wartości ruch jest uznany za zakończony
    # i krok jest pomijany (oszczędza to czas na mikrokorekty).
    _POSITION_TOLERANCE: float = 0.02  # m
    _YAW_TOLERANCE: float = 0.05       # rad

    # Podział budżetu czasu timeout_s na fazy ruchu.
    # 40% na obrót, max 90% pozostałości na translację, reszta to rezerwa
    # na StopMove i opóźnienia sieci (DDS przez Ethernet).
    _YAW_TIMEOUT_FRACTION: float = 0.4
    _TRANSLATION_TIMEOUT_FRACTION: float = 0.9

    def __init__(self, network_interface: str = 'eth0') -> None:
        """Inicjalizuje interfejs robota.

        Args:
            network_interface: Interfejs Ethernet do robota (np. 'eth0').
                Sprawdź aktualną nazwę przez: ip link show
        """
        self._network_interface = network_interface
        self._loco_client = None
        self._arm_action_client = None
        self._sdk_available = False

        # Śledzona poza robota – aktualizowana po każdym kroku ruchu.
        # Unitree SDK nie dostarcza odometrii w Sport Mode, więc estymujemy
        # pozycję przez dead reckoning (całkowanie komend prędkości).
        # Jest to uproszczenie – w praktyce robot ześlizgnie się nieco
        # względem zakodowanej trajektorii. Na potrzeby MVP jest wystarczające.
        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_yaw: float = 0.0

    # ------------------------------------------------------------------
    # Połączenie i rozłączenie
    # ------------------------------------------------------------------

    def connect(self, logger=None) -> None:
        """Nawiązuje połączenie z robotem przez Unitree SDK.

        Inicjalizuje transport DDS, LocoClient i G1ArmActionClient.
        Po inicjalizacji przełącza robota w tryb chodzenia (Start = FSM ID 200).

        WAŻNE: ChannelFactoryInitialize() należy wywołać DOKŁADNIE RAZ
        na cały proces. Wielokrotne wywołanie powoduje błąd DDS.
        Dlatego jest tu, nie w konstruktorze – chcemy mieć kontrolę
        nad momentem inicjalizacji i możliwość przechwycenia błędu.

        Args:
            logger: Logger ROS2 (opcjonalny).

        Raises:
            RuntimeError: Gdy unitree_sdk2py nie jest zainstalowane
                lub gdy inicjalizacja SDK się nie powiedzie.
        """
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        except ImportError as exc:
            msg = (
                f'Brak modułu Unitree SDK: {exc}. '
                'Zainstaluj pakiet: pip install unitree_sdk2py. '
                'Bez SDK robot nie może działać w trybie sprzętowym.'
            )
            self._log(logger, 'error', msg)
            raise RuntimeError(msg) from exc

        try:
            # Inicjalizacja transportu DDS – RAZ na cały proces.
            # Argument 0 = użyj pierwszej dostępnej karty sieciowej
            # (nadpisane przez drugi argument – nazwę interfejsu).
            ChannelFactoryInitialize(0, self._network_interface)

            self._loco_client = LocoClient()
            self._loco_client.SetTimeout(10.0)
            self._loco_client.Init()

            self._arm_action_client = G1ArmActionClient()
            self._arm_action_client.SetTimeout(10.0)
            self._arm_action_client.Init()

            # Przełączenie w tryb chodzenia (FSM ID = 200 = Sport Mode).
            # Bez wywołania Start() robot ignoruje komendy SetVelocity.
            self._loco_client.Start()
            self._sdk_available = True
            self._log(
                logger,
                'info',
                f'Połączono z robotem przez interfejs: {self._network_interface}. '
                'Robot w trybie chodzenia (Sport Mode aktywny).',
            )
        except Exception as exc:
            msg = (
                f'Błąd inicjalizacji SDK Unitree na interfejsie {self._network_interface}: '
                f'{exc}. Sprawdź połączenie sieciowe i stan robota.'
            )
            self._log(logger, 'error', msg)
            raise RuntimeError(msg) from exc

    def disconnect(self) -> None:
        """Rozłącza połączenie i bezpiecznie zatrzymuje robota.

        Kolejność operacji jest ważna:
        1. StopMove() – wyzerowanie prędkości (robot natychmiast staje)
        2. Damp()     – tryb tłumiony (serwomechanizmy rozluźnione, bezpieczne)
        3. release arm – ramiona w pozycji spoczynkowej

        Damp() bez wcześniejszego StopMove() może spowodować że robot
        zatrzyma się z impulsem inercji (nogi lekko się ślizgają).
        Dlatego zawsze najpierw StopMove.

        Metoda jest bezpieczna do wywołania gdy robot nie jest połączony
        (wszystkie try/except) – używamy tego w destroy_node() bez obawy
        o wyjątki podczas czyszczenia zasobów.
        """
        if self._loco_client is not None:
            try:
                self._loco_client.StopMove()
            except Exception:
                pass
            try:
                self._loco_client.Damp()
            except Exception:
                pass
            self._loco_client = None

        if self._arm_action_client is not None:
            try:
                from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map
                release_id = action_map.get('release arm')
                if release_id is not None:
                    self._arm_action_client.ExecuteAction(release_id)
            except Exception:
                pass
            self._arm_action_client = None

        self._sdk_available = False

    # ------------------------------------------------------------------
    # Sterowanie lokomocją
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: dict, timeout_s: float = 5.0) -> None:
        """Przemieszcza robota do podanej bezwzględnej pozy.

        ALGORYTM (dwufazowy):
        Faza 1 – Obrót: robot obraca się w miejscu do żądanego yaw.
            Obliczamy deltę kąta, normalizujemy do [-π, π] żeby wybrać
            krótszą ścieżkę (np. -90° zamiast +270°), i wysyłamy
            prędkość kątową z odpowiednim znakiem.

        Faza 2 – Translacja: robot idzie do żądanej pozycji (x, y).
            Obliczamy wektor przesunięcia i dekompozujemy go na vx (przód)
            i vy (bok). Dzielimy przez odległość żeby uzyskać jednostkowy
            wektor kierunku, mnożymy przez LINEAR_SPEED.

        Komponent 'z' (wysokość ramienia) NIE jest obsługiwany przez
        lokomocję – do sterowania ramionami użyj execute_arm_action().

        Args:
            pose: Słownik z kluczami 'x', 'y', 'z', 'yaw'.
            timeout_s: Łączny limit czasu na cały krok [s].

        Raises:
            RuntimeError: Gdy connect() nie było wywołane wcześniej.
        """
        if not self._sdk_available or self._loco_client is None:
            raise RuntimeError(
                'SDK Unitree nie jest zainicjalizowane. '
                'Wywołaj connect() przed move_to_pose().'
            )

        target_x = float(pose.get('x', 0.0))
        target_y = float(pose.get('y', 0.0))
        target_yaw = float(pose.get('yaw', self._current_yaw))

        start = time.monotonic()

        # Faza 1: Obrót do żądanej orientacji
        dyaw = target_yaw - self._current_yaw
        # Normalizacja do [-π, π]:
        #   (dyaw + π) % (2π) daje wartość w [0, 2π]
        #   Odjęcie π daje wartość w [-π, π]
        # Przykład: dyaw = 3π → znormalizowane = π (obrót o 180° w lewo)
        #           dyaw = -3π → znormalizowane = -π (obrót o 180° w prawo)
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

        if abs(dyaw) > self._YAW_TOLERANCE:
            rotation_time = abs(dyaw) / self._YAW_SPEED
            allocated_rot = min(rotation_time, timeout_s * self._YAW_TIMEOUT_FRACTION)
            self._send_velocity(
                vx=0.0,
                vy=0.0,
                vyaw=math.copysign(self._YAW_SPEED, dyaw),  # znak dyaw = kierunek obrotu
                duration_s=allocated_rot,
            )
            self._current_yaw = target_yaw

        # Faza 2: Translacja do żądanej pozycji
        dx = target_x - self._current_x
        dy = target_y - self._current_y
        distance = math.hypot(dx, dy)  # sqrt(dx² + dy²) – bezpieczniejsze niż ręczne obliczenie

        if distance > self._POSITION_TOLERANCE:
            elapsed = time.monotonic() - start
            remaining = max(0.0, timeout_s - elapsed)
            move_time = distance / self._LINEAR_SPEED

            # Mapowanie osi waypoints → osi LocoClient:
            # W układzie pozy: y = przód, x = bok
            # W LocoClient:   vx = przód, vy = bok
            # Dlatego dy/distance → vx (komponent do przodu)
            #         dx/distance → vy (komponent boczny)
            vx = (dy / distance) * self._LINEAR_SPEED
            vy = (dx / distance) * self._LINEAR_SPEED
            self._send_velocity(
                vx=vx,
                vy=vy,
                vyaw=0.0,
                duration_s=min(move_time, remaining * self._TRANSLATION_TIMEOUT_FRACTION),
            )

        # Aktualizacja śledzonej pozy po zakończeniu ruchu (dead reckoning)
        self._current_x = target_x
        self._current_y = target_y

    # ------------------------------------------------------------------
    # Sterowanie ramionami
    # ------------------------------------------------------------------

    def execute_arm_action(self, action_name: str) -> int:
        """Wykonuje predefiniowaną akcję ramion robota.

        Używa G1ArmActionClient, który wysyła do robota ID akcji
        z wbudowanego słownika gestów. Robot wykonuje animację ramion
        zdefiniowaną fabrycznie w oprogramowaniu.

        Dostępne akcje (z action_map w SDK):
            'release arm' – zwolnienie ramion (pozycja spoczynkowa)
            'shake hand'  – podanie ręki
            'high five'   – przybicie piątki
            'hug'         – objęcie (dobra pozycja do trzymania pudełka!)
            'clap'        – klaskanie
            'heart'       – serce dłońmi
            'hands up'    – ręce do góry
            i inne...

        Args:
            action_name: Klucz z słownika action_map SDK.

        Returns:
            Kod błędu z SDK (0 = sukces, inne = błąd).

        Raises:
            RuntimeError: Gdy connect() nie było wywołane.
            ValueError: Gdy action_name nie ma w słowniku SDK.
        """
        if not self._sdk_available or self._arm_action_client is None:
            raise RuntimeError(
                'SDK Unitree nie jest zainicjalizowane. '
                'Wywołaj connect() przed execute_arm_action().'
            )

        try:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map
        except ImportError as exc:
            raise RuntimeError(f'Brak modułu arm_action_client: {exc}') from exc

        action_id = action_map.get(action_name)
        if action_id is None:
            raise ValueError(
                f'Nieznana akcja ramion: {action_name!r}. '
                f'Dostępne akcje: {list(action_map.keys())}'
            )

        return self._arm_action_client.ExecuteAction(action_id)

    @property
    def is_connected(self) -> bool:
        """Zwraca True gdy SDK jest zainicjalizowane i połączone z robotem."""
        return self._sdk_available and self._loco_client is not None

    # ------------------------------------------------------------------
    # Metody wewnętrzne
    # ------------------------------------------------------------------

    @staticmethod
    def _log(logger, level: str, message: str) -> None:
        """Loguje wiadomość przez logger ROS2 lub stdout jako fallback.

        Statyczna metoda (nie potrzebuje self) – może być wywołana nawet
        przed inicjalizacją instancji.
        """
        if logger is not None:
            getattr(logger, level)(message)
        else:
            print(f'[UnitreeRobotAPI/{level.upper()}] {message}')

    def _send_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        duration_s: float,
    ) -> None:
        """Wysyła komendę prędkości i czeka na jej wykonanie.

        SDK obsługuje duration po stronie serwera – robot sam zatrzyma się
        po upływie czasu. My też czekamy (time.sleep) żeby nie wysyłać
        następnej komendy zanim robot skończy obecny ruch.

        POPRAWKA (błąd #10): Używamy try/finally żeby StopMove() było
        zawsze wywołane, nawet jeśli wątek zostanie przerwany (Ctrl+C,
        KeyboardInterrupt) podczas time.sleep(). Bez tego robot mógłby
        kontynuować ruch po zakończeniu programu.

        Jak działa try/finally:
            try:      – wykonaj kod w bloku
            finally:  – ZAWSZE wykonaj ten kod, czy był wyjątek czy nie
        To gwarantuje bezpieczne zatrzymanie nawet przy crashu.

        Args:
            vx: Prędkość do przodu/tyłu [m/s].
            vy: Prędkość boczna [m/s].
            vyaw: Prędkość kątowa [rad/s].
            duration_s: Czas trwania komendy [s].
        """
        if duration_s <= 0.0:
            return

        self._loco_client.SetVelocity(vx, vy, vyaw, duration_s)
        try:
            time.sleep(duration_s)
        finally:
            # POPRAWKA: StopMove() jest teraz w finally – gwarantuje
            # zatrzymanie robota nawet przy przerwaniu wątku.
            # Poprzednia wersja miała StopMove() po time.sleep() bez
            # zabezpieczenia, więc Ctrl+C podczas sleep pomijał zatrzymanie.
            try:
                self._loco_client.StopMove()
            except Exception:
                # StopMove może rzucić wyjątek jeśli połączenie zostało
                # już zerwane (np. podczas shutdown). Ignorujemy go –
                # liczy się że próbowaliśmy zatrzymać robota.
                pass
