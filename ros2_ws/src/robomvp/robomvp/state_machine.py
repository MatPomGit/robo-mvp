#!/usr/bin/env python3
"""Deterministyczny automat stanowy dla systemu RoboMVP.

CO TO JEST AUTOMAT STANOWY I DLACZEGO GO UŻYWAMY?
==================================================
Automat stanowy (ang. Finite State Machine, FSM) to model obliczeniowy,
w którym system może znajdować się dokładnie w jednym z skończonej liczby
„stanów". Przejście do nowego stanu następuje gdy spełniony jest określony
warunek (tzw. guard condition).

W robotyce FSM jest naturalnym wyborem do sterowania sekwencyjnym zachowaniem.
Zamiast jednej dużej funkcji z mnóstwem if/elif, mamy osobne handlery dla
każdego stanu – łatwiejsze w debugowaniu, testowaniu i rozszerzaniu.

SEKWENCJA STANÓW W ROBOMVP:
    SEARCH_TABLE  → wykryto marker stołu startowego (ID=21)
    DETECT_MARKER → wykryto marker pudełka (ID=10)
    ALIGN_WITH_BOX → offset korekcji poniżej progu wyrównania
    PICK_BOX      → oczekiwanie na zakończenie sekwencji podniesienia
    ROTATE_180    → oczekiwanie na zakończenie sekwencji obrotu
    NAVIGATE_TO_TARGET_MARKER → marker docelowego stołu (ID=22/30) w zasięgu
    PLACE_BOX     → oczekiwanie na zakończenie sekwencji odkładania
    FINISHED

WAŻNA ZASADA PROJEKTOWA – STANY „CZEKAJĄCE":
=============================================
Stany PICK_BOX, ROTATE_180 i PLACE_BOX to stany „czekające" – robot
wchodzi w nie, wykonuje akcję (sekwencję ruchów) w main_node, a stan
zmienia się dopiero gdy akcja zostanie zgłoszona jako zakończona.

Jak to działa w praktyce?
1. StateMachine.step() widzi że jesteśmy w PICK_BOX.
2. Nie robi NICZEGO – czeka na sygnał z zewnątrz.
3. main_node wywołuje state_machine.notify_sequence_done() po zakończeniu.
4. Dopiero wtedy step() dokonuje przejścia do ROTATE_180.

To jest wzorzec „separation of concerns": StateMachine zarządza KIEDY
zmieniać stan, a main_node zarządza CO robić podczas stanu.

PORÓWNANIE Z POPRZEDNIĄ (BŁĘDNĄ) IMPLEMENTACJĄ:
===============================================
Poprzednia wersja _handle_pick_box() wyglądała tak:
    def _handle_pick_box(self):
        self._log('Sekwencja podniesienia zakończona...')
        self._transition_to(State.ROTATE_180)  # natychmiastowe przejście!

Problem: main_node._step() działa tak:
    previous_state = state_machine.current_state   # = ALIGN_WITH_BOX
    new_state = state_machine.step()               # krok: ALIGN→PICK→ROTATE = ROTATE!
    if new_state != previous_state:                # ALIGN != ROTATE → True
        _execute_state_action(new_state)           # akcja dla ROTATE, nie PICK!

Automat skakał przez dwa stany w jednym kroku timera, więc akcja
dla PICK_BOX nigdy nie była uruchamiana. Robot nigdy nie podnosił pudełka.
"""

import time
from enum import IntEnum


class State(IntEnum):
    """Stany automatu stanowego scenariusza manipulacji.

    DLACZEGO IntEnum ZAMIAST str?
    IntEnum pozwala na porównania numeryczne (state >= State.PICK_BOX),
    serializację jako int (przydatne w wiadomości ROS2 State.msg),
    oraz czytelne nazwy w logach (State.PICK_BOX zamiast 3).
    Zwykły Enum bez dziedziczenia po int wymagałby .value wszędzie.
    """
    SEARCH_TABLE = 0
    DETECT_MARKER = 1
    ALIGN_WITH_BOX = 2
    PICK_BOX = 3
    ROTATE_180 = 4
    NAVIGATE_TO_TARGET_MARKER = 5
    PLACE_BOX = 6
    FINISHED = 7


# Mapowanie stanu na czytelną nazwę do publikacji w tematach ROS2.
# Słownik jest szybszy niż f-string z nazwą enuma gdy wywołujemy go
# setki razy na sekundę (co prawda tutaj nie, ale to dobra nawyczka).
STATE_NAMES = {
    State.SEARCH_TABLE: 'SEARCH_TABLE',
    State.DETECT_MARKER: 'DETECT_MARKER',
    State.ALIGN_WITH_BOX: 'ALIGN_WITH_BOX',
    State.PICK_BOX: 'PICK_BOX',
    State.ROTATE_180: 'ROTATE_180',
    State.NAVIGATE_TO_TARGET_MARKER: 'NAVIGATE_TO_TARGET_MARKER',
    State.PLACE_BOX: 'PLACE_BOX',
    State.FINISHED: 'FINISHED',
}

# Zbiór stanów, które wymagają zewnętrznego potwierdzenia zakończenia
# przed przejściem dalej. main_node musi wywołać notify_sequence_done()
# gdy sekwencja ruchu zostanie ukończona.
_SEQUENCE_STATES = frozenset({
    State.PICK_BOX,
    State.ROTATE_180,
    State.PLACE_BOX,
})


class StateMachine:
    """Deterministyczny automat stanowy scenariusza manipulacji.

    Zarządza przejściami między stanami na podstawie:
    - wykrytych markerów (update_marker)
    - wartości offsetu korekcji (update_offset)
    - potwierdzeń zakończenia sekwencji ruchów (notify_sequence_done)
    - timeoutów bezpieczeństwa (jeśli warunek przejścia nie jest spełniony)
    """

    def __init__(self, config: dict, logger=None):
        """Inicjalizuje automat stanowy z konfiguracją sceny.

        Args:
            config: Słownik z parametrami konfiguracji scene.yaml.
            logger: Logger ROS2 (opcjonalny; jeśli None, używa print).
        """
        self._config = config
        self._logger = logger
        self._state = State.SEARCH_TABLE
        self._last_marker_id = None
        self._last_pose = None
        self._last_offset = None

        # FLAGA POTWIERDZENIA SEKWENCJI
        # Gdy wchodzimy w stan wymagający sekwencji (PICK_BOX, ROTATE_180,
        # PLACE_BOX), ustawiamy _sequence_done = False. Automat czeka
        # w tym stanie dopóki main_node nie wywoła notify_sequence_done().
        self._sequence_done: bool = False

        # Identyfikatory markerów z konfiguracji (nie zakodowane na stałe)
        self._box_marker_id = config.get('box_marker_id', 10)
        self._pickup_table_marker = config.get('table_markers', {}).get(
            'pickup_table', 21
        )
        self._place_table_marker = config.get('table_markers', {}).get(
            'place_table', 22
        )
        self._target_marker_id = config.get('target_marker', self._place_table_marker)

        # Progi odległości
        self._stop_distance = config.get('stop_distance_threshold', 0.3)
        self._align_threshold = config.get('alignment_threshold', 0.05)

        # Timeouty stanów: domyślne wartości są nadpisywane przez scene.yaml
        default_timeouts = {
            'search_table': 20.0,
            'detect_marker': 20.0,
            'align_with_box': 10.0,
            'navigate_to_target_marker': 25.0,
        }
        configured_timeouts = config.get('state_timeouts', {})
        self._state_timeouts = {}
        for key, default_value in default_timeouts.items():
            self._state_timeouts[key] = self._sanitize_timeout(
                configured_timeouts.get(key, default_value),
                default_value,
                key,
            )

        # Moment wejścia w bieżący stan – używany do sprawdzania timeoutów
        self._state_enter_time = time.monotonic()

    # ------------------------------------------------------------------
    # Sanityzacja danych wejściowych
    # ------------------------------------------------------------------

    def _sanitize_timeout(self, value, default_value: float, key: str) -> float:
        """Normalizuje timeout: konwertuje na float i sprawdza czy > 0.

        DLACZEGO SANITYZACJA?
        Plik YAML może zawierać błędy (np. '20s' zamiast 20, lub 0).
        Zamiast rzucać wyjątkiem przy starcie węzła, logujemy ostrzeżenie
        i używamy bezpiecznej wartości domyślnej. Robot może pracować
        z suboptymalnymi timeoutami – nie może pracować gdy węzeł się crashuje.
        """
        try:
            timeout = float(value)
        except (TypeError, ValueError):
            self._log(
                f'Nieprawidłowy timeout "{key}={value}", używam domyślnego {default_value:.2f}s'
            )
            return default_value

        if timeout <= 0.0:
            self._log(
                f'Timeout "{key}" musi być dodatni, używam domyślnego {default_value:.2f}s'
            )
            return default_value

        return timeout

    # ------------------------------------------------------------------
    # Właściwości publiczne
    # ------------------------------------------------------------------

    @property
    def current_state(self) -> State:
        """Zwraca aktualny stan automatu."""
        return self._state

    @property
    def current_state_name(self) -> str:
        """Zwraca czytelną nazwę aktualnego stanu."""
        return STATE_NAMES[self._state]

    @property
    def is_in_sequence_state(self) -> bool:
        """Zwraca True gdy automat czeka na zakończenie sekwencji ruchu."""
        return self._state in _SEQUENCE_STATES

    # ------------------------------------------------------------------
    # Interfejs aktualizacji danych czujnikowych
    # ------------------------------------------------------------------

    def update_marker(self, marker_id: int, x: float, y: float, z: float):
        """Aktualizuje ostatnio wykryty marker i jego pozycję 3D.

        Wywoływana przez main_node po odebraniu wiadomości marker_pose.
        StateMachine nie subskrybuje tematów ROS2 bezpośrednio – to
        celowe odsprzężenie (decoupling): StateMachine jest testowalny
        bez ROS2, wystarczy wywołać tę metodę z dowolnymi wartościami.
        """
        self._last_marker_id = marker_id
        self._last_pose = (x, y, z)
        self._log(
            f'Wykryto marker ID={marker_id} na pozycji '
            f'({x:.3f}, {y:.3f}, {z:.3f}) m względem kamery.'
        )

    def update_offset(self, dx: float, dy: float, dz: float):
        """Aktualizuje ostatni obliczony offset korekcji pozycji.

        Wywoływana przez main_node po odebraniu wiadomości offset.
        Offset jest potrzebny tylko w stanie ALIGN_WITH_BOX.
        """
        self._last_offset = (dx, dy, dz)
        self._log(
            f'Zaktualizowano offset korekcji: '
            f'dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f} m.'
        )

    def notify_sequence_done(self):
        """Sygnalizuje, że bieżąca sekwencja ruchów została zakończona.

        KLUCZOWA METODA DLA STANÓW PICK_BOX, ROTATE_180 I PLACE_BOX.
        main_node wywołuje tę metodę po pomyślnym zakończeniu execute_sequence().
        Bez tego wywołania automat zostanie w aktualnym stanie na zawsze
        (lub do upłynięcia timeoutu awaryjnego, jeśli zostanie dodany).

        Przykład przepływu:
            1. step() → PICK_BOX, _sequence_done = False
            2. main_node uruchamia execute_sequence(get_pick_box(), ...)
            3. execute_sequence() zwraca True
            4. main_node wywołuje notify_sequence_done()
            5. _sequence_done = True
            6. Kolejne step() → ROTATE_180
        """
        if self._state in _SEQUENCE_STATES:
            self._sequence_done = True
            self._log(
                f'Sekwencja zakończona w stanie {self.current_state_name}. '
                'Automat jest gotowy do przejścia do następnego stanu.'
            )
        else:
            self._log(
                f'notify_sequence_done() wywołane w stanie {self.current_state_name}, '
                f'który nie jest stanem sekwencji. Ignoruję.'
            )

    # ------------------------------------------------------------------
    # Główna metoda kroku
    # ------------------------------------------------------------------

    def step(self) -> State:
        """Wykonuje jeden krok automatu stanowego.

        Sprawdza warunki przejścia dla bieżącego stanu.
        Gwarantuje CO NAJWYŻEJ JEDNO przejście na wywołanie step().
        Dzięki temu main_node zawsze widzi każdy stan i może uruchomić
        odpowiednią sekwencję ruchów.

        Returns:
            Nowy (lub aktualny) stan po wykonaniu kroku.
        """
        if self._state == State.SEARCH_TABLE:
            self._handle_search_table()
        elif self._state == State.DETECT_MARKER:
            self._handle_detect_marker()
        elif self._state == State.ALIGN_WITH_BOX:
            self._handle_align_with_box()
        elif self._state == State.PICK_BOX:
            self._handle_pick_box()
        elif self._state == State.ROTATE_180:
            self._handle_rotate_180()
        elif self._state == State.NAVIGATE_TO_TARGET_MARKER:
            self._handle_navigate_to_target()
        elif self._state == State.PLACE_BOX:
            self._handle_place_box()
        # State.FINISHED: celowo nie ma handlera – stan terminalny

        return self._state

    # ------------------------------------------------------------------
    # Handlery stanów (prywatne)
    # ------------------------------------------------------------------

    def _transition_to(self, new_state: State):
        """Przechodzi do nowego stanu, resetuje timer i flagę sekwencji.

        DLACZEGO RESET _sequence_done PRZY KAŻDYM PRZEJŚCIU?
        Gdy wchodzimy w nowy stan sekwencyjny, zawsze musimy zacząć
        od False. Gdybyśmy nie resetowali, stary sygnał done z poprzedniej
        sekwencji mógłby natychmiast zakończyć nową sekwencję.
        """
        old_name = STATE_NAMES[self._state]
        new_name = STATE_NAMES[new_state]
        self._state = new_state
        self._state_enter_time = time.monotonic()
        self._sequence_done = False  # reset flagi dla nowego stanu
        self._log(f'Przejście stanu: {old_name} → {new_name}')

    def _is_timeout(self, key: str) -> bool:
        """Sprawdza czy upłynął timeout dla bieżącego stanu.

        Timeouty są „awaryjnym wyjściem" gdy warunek przejścia
        nie jest spełniany zbyt długo – np. marker nie jest wykrywany.
        Wolne wyjście awaryjne jest lepsze niż zablokowanie w stanie na zawsze.
        """
        timeout_s = self._state_timeouts.get(key)
        if timeout_s is None:
            return False
        elapsed = time.monotonic() - self._state_enter_time
        return elapsed >= timeout_s

    def _handle_search_table(self):
        """Stan: szukanie stołu startowego przez skanowanie przestrzeni."""
        if (
            self._pickup_table_marker is not None
            and self._last_marker_id == self._pickup_table_marker
        ):
            self._log(
                f'Wykryto marker stołu startowego (ID={self._pickup_table_marker}). '
                'Przechodzę do detekcji markera na pudełku.'
            )
            self._transition_to(State.DETECT_MARKER)
            return

        if self._is_timeout('search_table'):
            self._log(
                'Timeout SEARCH_TABLE – nie wykryto markera stołu w czasie. '
                'Przejście awaryjne do DETECT_MARKER.'
            )
            self._transition_to(State.DETECT_MARKER)

    def _handle_detect_marker(self):
        """Stan: wykrywanie markera na pudełku."""
        if (
            self._box_marker_id is not None
            and self._last_marker_id == self._box_marker_id
            and self._last_pose is not None
        ):
            self._log(
                f'Wykryto marker pudełka (ID={self._box_marker_id}). '
                'Przechodzę do wyrównywania pozycji.'
            )
            self._transition_to(State.ALIGN_WITH_BOX)
            return

        if self._is_timeout('detect_marker'):
            self._log(
                'Timeout DETECT_MARKER – nie wykryto markera pudełka w czasie. '
                'Przejście awaryjne do ALIGN_WITH_BOX.'
            )
            self._transition_to(State.ALIGN_WITH_BOX)

    def _handle_align_with_box(self):
        """Stan: wyrównanie pozycji z pudełkiem na podstawie offsetu.

        Automat czeka aż obydwa komponenty offsetu (dx i dz) zejdą
        poniżej progu alignment_threshold. Wartość dy (głębokość) jest
        pominięta – podejście do stołu odbyło się w etapie approach_table.
        """
        if self._last_offset is not None:
            dx, _, dz = self._last_offset
            if abs(dx) < self._align_threshold and abs(dz) < self._align_threshold:
                self._log(
                    f'Wyrównanie zakończone (|dx|={abs(dx):.4f} < {self._align_threshold}, '
                    f'|dz|={abs(dz):.4f} < {self._align_threshold}). '
                    'Przechodzę do podniesienia pudełka.'
                )
                self._transition_to(State.PICK_BOX)
                return

        if self._is_timeout('align_with_box'):
            self._log(
                'Timeout ALIGN_WITH_BOX – wyrównanie nie zakończone w czasie. '
                'Przejście awaryjne do PICK_BOX.'
            )
            self._transition_to(State.PICK_BOX)

    def _handle_pick_box(self):
        """Stan: oczekiwanie na zakończenie sekwencji podniesienia pudełka.

        POPRAWKA (błąd #2): Poprzednia implementacja natychmiast
        przechodziła do ROTATE_180, pomijając całkowicie sekwencję podniesienia.
        Teraz automat czeka dopóki main_node nie wywoła notify_sequence_done().

        Diagram przepływu:
            main_node wykrywa ALIGN_WITH_BOX → PICK_BOX
            main_node uruchamia execute_sequence(get_pick_box())
            execute_sequence kończy się → main_node wywołuje notify_sequence_done()
            Kolejne step() → _sequence_done=True → przejście do ROTATE_180
        """
        if self._sequence_done:
            self._log(
                'Sekwencja podniesienia pudełka zakończona. '
                'Przechodzę do obrotu o 180°.'
            )
            self._transition_to(State.ROTATE_180)

    def _handle_rotate_180(self):
        """Stan: oczekiwanie na zakończenie sekwencji obrotu.

        POPRAWKA (błąd #2): Analogicznie do _handle_pick_box –
        automat czeka na notify_sequence_done() zamiast przechodzić natychmiast.
        """
        if self._sequence_done:
            self._log(
                'Obrót o 180° zakończony. '
                'Przechodzę do nawigacji do drugiego stołu.'
            )
            self._transition_to(State.NAVIGATE_TO_TARGET_MARKER)

    def _handle_navigate_to_target(self):
        """Stan: nawigacja do drugiego stołu przez detekcję markera docelowego.

        Warunek zatrzymania: marker stołu docelowego (ID=22 lub 30, zależnie
        od konfiguracji) jest w odległości mniejszej niż stop_distance_threshold.
        Odległość szacowana jest przez z (głębokość w układzie kamery).

        Sprawdzamy zarówno target_marker_id jak i place_table_marker,
        bo w konfiguracji możemy mieć różne markery dla nawigacji i odkładania.
        """
        if (
            self._last_marker_id in {
                self._target_marker_id,
                self._place_table_marker,
            }
            and self._last_pose is not None
        ):
            _, _, z = self._last_pose
            if z < self._stop_distance:
                self._log(
                    f'Dotarłem do docelowego stołu '
                    f'(z={z:.3f} m < próg={self._stop_distance:.3f} m). '
                    'Przechodzę do odkładania pudełka.'
                )
                self._transition_to(State.PLACE_BOX)
                return

        if self._is_timeout('navigate_to_target_marker'):
            self._log(
                'Timeout NAVIGATE_TO_TARGET_MARKER – nie dotarto do celu w czasie. '
                'Kończę scenariusz bez odkładania pudełka (stan awaryjny).'
            )
            self._transition_to(State.FINISHED)

    def _handle_place_box(self):
        """Stan: oczekiwanie na zakończenie sekwencji odkładania pudełka.

        POPRAWKA (błąd #2): Analogicznie do _handle_pick_box i _handle_rotate_180.
        Automat czeka na notify_sequence_done() zamiast przechodzić natychmiast.
        """
        if self._sequence_done:
            self._log(
                'Sekwencja odkładania pudełka zakończona. '
                'Scenariusz zakończony pomyślnie.'
            )
            self._transition_to(State.FINISHED)

    # ------------------------------------------------------------------
    # Logowanie
    # ------------------------------------------------------------------

    def _log(self, message: str):
        """Loguje wiadomość przez logger ROS2 lub stdout (fallback).

        Prefiks [StateMachine] ułatwia filtrowanie logów gdy wiele
        węzłów pisze do tego samego strumienia.
        """
        if self._logger:
            self._logger.info(f'[StateMachine] {message}')
        else:
            print(f'[StateMachine] {message}')
