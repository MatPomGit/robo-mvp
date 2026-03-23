#!/usr/bin/env python3
"""Predefiniowane sekwencje ruchów robota Unitree G1 EDU.

KONCEPCJA ARCHITEKTONICZNA: Dlaczego sekwencje, nie planowanie ruchu?
=====================================================================
W klasycznej robotyce mobilnej robot „planuje" trasę w czasie rzeczywistym,
analizując otoczenie i dynamicznie wyznaczając kolejne kroki. To podejście
wymaga map, algorytmów planowania (np. A*, RRT) i złożonej percepcji.

W projekcie RoboMVP celowo rezygnujemy z tej złożoności na rzecz
„zakodowanych na stałe" trajektorii. Działa to jak choreografia:
z góry wiemy, że stół jest w odległości ~0.8 m, więc po prostu
programujemy te dokładne przesunięcia. Takie podejście:
- Eliminuje źródła błędów związane z planowaniem
- Jest deterministyczne – ta sama sekwencja daje ten sam wynik
- Jest gotowe do działania w ciągu godzin, nie tygodni
- Dla demonstracji w kontrolowanym środowisku jest w zupełności wystarczające

STRUKTURA POZY:
    Każda poza to słownik {'x', 'y', 'z', 'yaw'} reprezentujący ABSOLUTNĄ
    pozycję w przestrzeni robota (nie przyrostowe przesunięcie!).
    UnitreeRobotAPI.move_to_pose() przelicza różnicę między bieżącą
    a docelową pozą na komendy prędkości wysyłane do LocoClient.

    x   – przesunięcie boczne [m]  (+ = lewo, od środka startowego)
    y   – przesunięcie do przodu [m] (+ = przód)
    z   – wysokość ramienia [m]  (obsługiwane przez G1ArmActionClient)
    yaw – orientacja [rad]  (+ = obrót w lewo, 0 = kierunek startowy)
"""

# math na poziomie modułu, nie wewnątrz funkcji.
# Importowanie wewnątrz funkcji działa, ale jest nieefektywne i niespójne –
# Python ładuje moduł przy pierwszym imporcie i cachuje go, więc koszt
# wielokrotnych importów jest minimalny, ale konwencja PEP-8 nakazuje
# umieszczanie importów na górze pliku, żeby czytelnik od razu widział
# wszystkie zależności modułu.
import math
import time


# ---------------------------------------------------------------------------
# Sekwencje ruchów – pojedyncze etapy scenariusza
# ---------------------------------------------------------------------------
# DLACZEGO LISTY SŁOWNIKÓW?
# Słownik {'x':..., 'y':..., 'z':..., 'yaw':...} jest bardziej czytelny
# niż krotka (x, y, z, yaw), bo od razu widać co oznacza każda wartość.
# Python nie wymaga klasy do przechowania prostych danych – YAGNI (You Ain't
# Gonna Need It). Gdyby sekwencje stały się bardziej złożone (np. z akcjami
# ramion), warto rozważyć dataclassę.

def get_approach_table() -> list:
    """Zwraca sekwencję podejścia do pierwszego stołu.

    Robot porusza się do przodu w 3 krokach, zatrzymując się w pozycji
    roboczej (~0.8 m przed stołem), skąd kamera ciała dobrze widzi marker.

    Pozycja startowa (y=0) to punkt, gdzie robot stoi po wywołaniu
    sekwencji. Każdy krok dodaje ~0.3 m, co przy prędkości 0.3 m/s
    daje czas wykonania ~1 s/krok (w unitree_robot_api._LINEAR_SPEED).
    """
    return [
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},   # pozycja startowa
        {'x': 0.0, 'y': 0.3, 'z': 0.0, 'yaw': 0.0},   # krok naprzód
        {'x': 0.0, 'y': 0.6, 'z': 0.0, 'yaw': 0.0},   # bliżej stołu
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # pozycja robocza
    ]


def get_pick_box() -> list:
    """Zwraca sekwencję podniesienia pudełka ze stołu.

    Sekwencja symuluje naturalny ruch „zejścia" ramienia:
    1. Pozycja wyjściowa – ramię przy ciele (z=0)
    2. Ramię w połowie drogi w dół (z=-0.3)
    3. Pozycja chwytu – ramię przy powierzchni stołu (z=-0.5)
    4. Uniesienie pudełka z powrotem do góry (z=0)

    Komponent 'z' jest przekazywany do G1ArmActionClient przez
    wyższe warstwy (main_node). Lokomocja (x, y, yaw) pozostaje
    niezmieniona – robot stoi w miejscu podczas manipulacji.
    """
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # pozycja wyjściowa
        {'x': 0.0, 'y': 0.8, 'z': -0.3, 'yaw': 0.0},  # ramię w dół (50%)
        {'x': 0.0, 'y': 0.8, 'z': -0.5, 'yaw': 0.0},  # pozycja chwytu
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # uniesienie pudełka
    ]


def get_rotate_180() -> list:
    """Zwraca sekwencję obrotu o 180 stopni.

    Obrót jest podzielony na dwa kroki po 90° (π/2 rad każdy).
    Dlaczego nie jeden krok 180°? Przy dużych obrótach robot może
    stracić równowagę lub przekroczyć timeout. Podział na mniejsze
    kroki sprawia, że każdy krok jest bezpieczniejszy i łatwiej
    wykryć ewentualne problemy (np. przeszkodę po lewej stronie).

    Po obrocie yaw = π (180°) robot jest skierowany w kierunku
    drugiego stołu. Pozycja (x=0, y=0.8) nie zmienia się –
    obrót w miejscu nie przesuwa robota.
    """
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},           # orientacja startowa (0°)
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi / 2},   # pół drogi (90°)
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi},       # cel (180°)
    ]


def get_walk_to_second_table() -> list:
    """Zwraca sekwencję chodzenia do drugiego stołu.

    Robot, już obrócony o 180° (yaw=π), idzie „do przodu" względem
    nowej orientacji. Ponieważ poza jest absolutna, y rośnie dalej
    (od 0.8 do 3.2 m) – czyli robot przebywa łącznie ~2.4 m.

    Odległość 2.4 m przy prędkości 0.3 m/s to ~8 sekund marszu.
    Kroki co ~0.7–0.9 m pozwalają na regularne aktualizacje stanu
    automatu stanowego, który może zareagować jeśli marker drugiego
    stołu pojawi się wcześniej niż oczekiwano.
    """
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi},   # pozycja startowa (po obrocie)
        {'x': 0.0, 'y': 1.5, 'z': 0.0, 'yaw': math.pi},   # pierwszy krok do przodu
        {'x': 0.0, 'y': 2.5, 'z': 0.0, 'yaw': math.pi},   # połowa drogi
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},   # przed drugim stołem
    ]


def get_place_box() -> list:
    """Zwraca sekwencję odłożenia pudełka na drugi stół.

    Lustrzane odbicie get_pick_box() – ramię schodzi do powierzchni
    stołu, zwalnia chwyt, a następnie wraca do pozycji wyjściowej.
    Robot stoi w miejscu (x=0, y=3.2, yaw=π) przez całą sekwencję.
    """
    return [
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},    # pozycja wyjściowa
        {'x': 0.0, 'y': 3.2, 'z': -0.3, 'yaw': math.pi},   # ramię w dół (50%)
        {'x': 0.0, 'y': 3.2, 'z': -0.5, 'yaw': math.pi},   # odłożenie pudełka
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},    # ramię w górę (koniec)
    ]


# ---------------------------------------------------------------------------
# Narzędzia korekcji i wykonania
# ---------------------------------------------------------------------------

def apply_offset_to_sequence(sequence: list, dx: float, dy: float, dz: float) -> list:
    """Stosuje offset korekcji do całej sekwencji ruchów.

    DLACZEGO KOREKCJA NA POZIOMIE SEKWENCJI, NIE KROKU?
    Marker pudełka jest wykrywany jednorazowo przed startem sekwencji.
    Gdybyśmy koryggowali każdy krok osobno w czasie wykonywania, robot
    musiałby widzieć marker przez całą sekwencję manipulacji – co jest
    trudne, bo podczas podnoszenia pudełka zasłania ono kamerę.
    Dlatego: jeden pomiar offsetu → korekcja całej sekwencji → wykonanie.

    Funkcja nie modyfikuje oryginalnej listy (dict(pose) tworzy płytką kopię),
    co jest dobrą praktyką – unikamy modyfikacji danych wejściowych.

    Args:
        sequence: Lista poz do wykonania (nie jest modyfikowana).
        dx: Korekcja boczna (metry). Pozytywna = przesuń w lewo.
        dy: Korekcja przód/tył (metry). Pozytywna = przesuń do przodu.
        dz: Korekcja pionowa (metry). Pozytywna = unieś ramię wyżej.

    Returns:
        Nowa, skorygowana sekwencja poz.
    """
    corrected = []
    for pose in sequence:
        # dict(pose) tworzy kopię słownika – nie modyfikujemy oryginału.
        # To jest tzw. „defensive copy" – dobra praktyka gdy nie chcemy
        # efektów ubocznych w wywołującym kodzie.
        corrected_pose = dict(pose)
        corrected_pose['x'] = pose.get('x', 0.0) + dx
        corrected_pose['y'] = pose.get('y', 0.0) + dy
        corrected_pose['z'] = pose.get('z', 0.0) + dz
        corrected.append(corrected_pose)
    return corrected


def execute_sequence(
    sequence: list,
    robot_api=None,
    logger=None,
    total_timeout_s: float = 30.0,
    step_timeout_s: float = 5.0,
) -> bool:
    """Wykonuje sekwencję ruchów przez Unitree SDK lub loguje w trybie demo.

    WZORZEC NULL OBJECT PATTERN:
    Parametr robot_api może być None (brak połączenia z robotem) lub
    instancją UnitreeRobotAPI. Zamiast sprawdzać wszędzie 'if robot_api:',
    używamy tego jednego miejsca jako punktu decyzji. To sprawia, że
    wywołujący kod (main_node) jest prosty i nie musi wiedzieć o trybach.

    DLACZEGO DWA TIMEOUTY (total i step)?
    - total_timeout_s: zabezpieczenie przed „zawieszeniem się" całej sekwencji
      (np. gdy robot nie może dojść do celu i kręci się w kółko)
    - step_timeout_s: zabezpieczenie przed pojedynczym krokiem, który trwa
      zbyt długo (np. gdy SDK nie odpowiada na konkretną komendę)
    Oba są potrzebne – bez total_timeout robot mógłby utknąć nawet gdy
    każdy krok mieści się w step_timeout.

    WAŻNA UWAGA ARCHITEKTONICZNA:
    Ta funkcja jest BLOKUJĄCA – zawiera time.sleep() wewnątrz move_to_pose().
    Wywoływanie jej z callbacku timera ROS2 (jak w main_node._step) blokuje
    cały executor ROS2 na czas wykonania sekwencji. W trybie produkcyjnym
    należałoby uruchomić tę funkcję w osobnym wątku lub użyć ROS2 Action Server.
    W tym MVP celowo akceptujemy to ograniczenie dla prostoty kodu.

    Args:
        sequence: Lista słowników poz do wykonania.
        robot_api: Interfejs API robota (None = tryb bez robota).
        logger: Logger ROS2 do wypisywania komunikatów.
        total_timeout_s: Maksymalny czas całej sekwencji [s].
        step_timeout_s: Maksymalny czas pojedynczego kroku [s].

    Returns:
        True jeśli sekwencja wykonana pomyślnie, False w przypadku błędu.
    """
    # time.monotonic() zamiast time.time() – nie cofa się przy zmianach
    # zegara systemowego (np. synchronizacja NTP), co mogłoby spowodować
    # fałszywy timeout lub brak timeoutu.
    start = time.monotonic()
    total = len(sequence)  # POPRAWKA: zmienna 'total' była używana w f-stringu niżej,
                           # ale nigdy nie była zdefiniowana → NameError w trybie demo.

    for i, pose in enumerate(sequence):
        # Sprawdzenie globalnego timeoutu PRZED każdym krokiem.
        # Nie czekamy do błędu w kroku – oszczędzamy czas.
        elapsed_total = time.monotonic() - start
        if elapsed_total > total_timeout_s:
            if logger:
                logger.error(
                    f'Timeout całej sekwencji po {elapsed_total:.2f}s '
                    f'(limit: {total_timeout_s:.2f}s). '
                    f'Wykonano {i}/{total} kroków.'
                )
            return False

        step_start = time.monotonic()

        if robot_api is not None:
            # Tryb sprzętowy: wysyłamy komendę do fizycznego robota.
            # Wyjątek jest przechwytywany tu (nie w robot_api), bo to
            # execute_sequence decyduje jak zareagować na błąd kroku
            # (przerwać sekwencję, zalogować, zwrócić False).
            try:
                robot_api.move_to_pose(pose, timeout_s=step_timeout_s)
            except Exception as e:
                if logger:
                    logger.error(
                        f'Błąd wykonania kroku {i + 1}/{total}: {e}. '
                        'Sprawdź połączenie z robotem i stan interfejsu SDK. '
                        'Wykonanie sekwencji zakończone błędem.'
                    )
                return False
        else:
            # Tryb bez robota (demo/testy): logujemy komendę zamiast ją wysyłać.
            # Dzięki temu można testować logikę automatu stanowego bez sprzętu.
            if logger:
                logger.info(
                    f'[no_robot] Krok {i + 1}/{total}: '  # POPRAWKA: było {total} – NameError
                    f'x={pose.get("x", 0):.2f}, '
                    f'y={pose.get("y", 0):.2f}, '
                    f'z={pose.get("z", 0):.2f}, '
                    f'yaw={pose.get("yaw", 0):.2f}'
                )

        # Sprawdzenie timeoutu kroku PO jego wykonaniu.
        # move_to_pose() ma własny timeout, ale tutaj mamy dodatkowe
        # zabezpieczenie na wypadek gdyby move_to_pose() zignorował timeout.
        elapsed_step = time.monotonic() - step_start
        if elapsed_step > step_timeout_s:
            if logger:
                logger.error(
                    f'Timeout kroku {i + 1}/{total} po {elapsed_step:.2f}s '
                    f'(limit: {step_timeout_s:.2f}s). '
                    'Sprawdź czy robot odpowiada na komendy SDK.'
                )
            return False

    return True
