"""Testy jednostkowe automatu stanowego RoboMVP.

FILOZOFIA TESTOWANIA AUTOMATU STANOWEGO:
=========================================
StateMachine jest celowo niezależna od ROS2 – można ją testować bez
uruchamiania jakiegokolwiek węzła, roscore czy demona DDS.
Wystarczy zainicjalizować klasę z konfiguracją i wywoływać metody.

Dobre testy automatu stanowego sprawdzają:
1. Czy każde przejście stanu wymaga spełnienia swojego warunku (nie mniej)
2. Czy automat NIE przechodzi do następnego stanu bez spełnienia warunku
3. Czy timeouty działają poprawnie przy nieprawidłowych wartościach
4. Czy stany sekwencyjne (PICK_BOX, ROTATE_180, PLACE_BOX) CZEKAJĄ
   na notify_sequence_done() zamiast przechodzić natychmiast

Uruchamianie testów:
    cd ros2_ws/src/robomvp
    python -m pytest ../../../../tests/ -v
"""

import sys
from pathlib import Path

# Dodajemy katalog pakietu do sys.path żeby importy działały bez instalacji ROS2.
# W środowisku CI/CD bez ROS2 pytest nie zna ścieżek ament_index,
# więc musimy sami wskazać gdzie szukać modułów.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'robomvp'))

from robomvp.state_machine import State, StateMachine


def _base_config():
    """Bazowa konfiguracja sceny do testów – odwzorowanie scene.yaml."""
    return {
        'table_markers': {'pickup_table': 21, 'place_table': 22},
        'box_marker_id': 10,
        'target_marker': 30,
        'stop_distance_threshold': 0.3,
        'alignment_threshold': 0.05,
    }


# ---------------------------------------------------------------------------
# Testy ścieżki nominalnej (happy path)
# ---------------------------------------------------------------------------

def test_full_scenario_with_sequence_notifications():
    """Weryfikuje pełny scenariusz z poprawną notyfikacją sekwencji.

    KLUCZOWY TEST POPRAWKI BŁĘDU #2.
    Poprzednia implementacja przechodziła przez PICK_BOX i ROTATE_180
    natychmiast, bez czekania na zakończenie sekwencji.
    Teraz każdy stan sekwencyjny wymaga wywołania notify_sequence_done().

    Diagram testu:
        SEARCH_TABLE → (marker stołu 21) → DETECT_MARKER
        DETECT_MARKER → (marker pudełka 10) → ALIGN_WITH_BOX
        ALIGN_WITH_BOX → (offset = 0) → PICK_BOX
        PICK_BOX (czeka...) → (notify_done) → ROTATE_180
        ROTATE_180 (czeka...) → (notify_done) → NAVIGATE_TO_TARGET_MARKER
        NAVIGATE_TO_TARGET_MARKER → (marker 30, z < 0.3) → PLACE_BOX
        PLACE_BOX (czeka...) → (notify_done) → FINISHED
    """
    sm = StateMachine(_base_config())

    # SEARCH_TABLE → DETECT_MARKER po wykryciu markera stołu startowego
    sm.update_marker(21, 0.0, 0.0, 1.0)
    assert sm.step() == State.DETECT_MARKER

    # DETECT_MARKER → ALIGN_WITH_BOX po wykryciu markera pudełka
    sm.update_marker(10, 0.0, 0.0, 1.0)
    assert sm.step() == State.ALIGN_WITH_BOX

    # ALIGN_WITH_BOX → PICK_BOX gdy offset < alignment_threshold
    sm.update_offset(0.0, 0.0, 0.0)
    assert sm.step() == State.PICK_BOX

    # PICK_BOX powinien CZEKAĆ bez notify_sequence_done()
    assert sm.step() == State.PICK_BOX, (
        'PICK_BOX nie powinno przechodzić dalej bez notify_sequence_done()'
    )
    assert sm.step() == State.PICK_BOX, (
        'PICK_BOX nie powinno przechodzić dalej po wielu step() bez notyfikacji'
    )

    # Teraz notyfikujemy zakończenie sekwencji
    sm.notify_sequence_done()
    assert sm.step() == State.ROTATE_180

    # ROTATE_180 też powinno czekać
    assert sm.step() == State.ROTATE_180, (
        'ROTATE_180 nie powinno przechodzić dalej bez notify_sequence_done()'
    )

    sm.notify_sequence_done()
    assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER

    # NAVIGATE_TO_TARGET_MARKER → PLACE_BOX gdy marker docelowy wystarczająco blisko
    sm.update_marker(30, 0.0, 0.0, 0.2)  # z=0.2 < stop_distance=0.3
    assert sm.step() == State.PLACE_BOX

    # PLACE_BOX też powinno czekać
    assert sm.step() == State.PLACE_BOX, (
        'PLACE_BOX nie powinno przechodzić dalej bez notify_sequence_done()'
    )

    sm.notify_sequence_done()
    assert sm.step() == State.FINISHED


def test_navigate_to_target_accepts_place_table_marker():
    """NAVIGATE_TO_TARGET_MARKER akceptuje marker stołu docelowego (ID=22).

    W konfiguracji target_marker=30, ale marker place_table=22 też
    powinien triggerować zatrzymanie. To ważne gdy robot widzi stół
    przez swój marker ID=22 zamiast osobnego markera nawigacyjnego ID=30.
    """
    sm = StateMachine(_base_config())

    # Skrót do stanu nawigacji
    sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()  # → DETECT_MARKER
    sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()  # → ALIGN_WITH_BOX
    sm.update_offset(0.0, 0.0, 0.0); sm.step()       # → PICK_BOX
    sm.notify_sequence_done(); sm.step()               # → ROTATE_180
    sm.notify_sequence_done(); sm.step()               # → NAVIGATE_TO_TARGET_MARKER

    # Marker 22 (place_table) powinien też zatrzymać nawigację
    sm.update_marker(22, 0.0, 0.0, 0.15)
    assert sm.step() == State.PLACE_BOX


def test_navigate_does_not_stop_when_marker_too_far():
    """NAVIGATE_TO_TARGET_MARKER nie zatrzymuje się gdy marker za daleko."""
    sm = StateMachine(_base_config())

    sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
    sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
    sm.update_offset(0.0, 0.0, 0.0); sm.step()
    sm.notify_sequence_done(); sm.step()
    sm.notify_sequence_done(); sm.step()  # → NAVIGATE_TO_TARGET_MARKER

    # z=0.5 > stop_distance=0.3 – za daleko, nie powinno przejść
    sm.update_marker(30, 0.0, 0.0, 0.5)
    assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER


def test_align_with_box_waits_when_offset_too_large():
    """ALIGN_WITH_BOX nie przechodzi do PICK_BOX gdy offset powyżej progu."""
    sm = StateMachine(_base_config())

    sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
    sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()

    # Offset 0.1 > alignment_threshold=0.05 – robot nie jest wystarczająco wyrównany
    sm.update_offset(0.1, 0.0, 0.0)
    assert sm.step() == State.ALIGN_WITH_BOX, (
        'ALIGN_WITH_BOX nie powinno przechodzić gdy |dx|=0.1 > threshold=0.05'
    )

    # Po zmniejszeniu offsetu poniżej progu powinno przejść
    sm.update_offset(0.02, 0.0, 0.01)
    assert sm.step() == State.PICK_BOX


def test_notify_sequence_done_only_affects_sequence_states():
    """notify_sequence_done() ignorowana w stanach niesekwencyjnych."""
    sm = StateMachine(_base_config())
    # Jesteśmy w SEARCH_TABLE (stan niesekwencyjny)
    assert sm.current_state == State.SEARCH_TABLE
    sm.notify_sequence_done()  # powinno być zignorowane
    assert sm.step() == State.SEARCH_TABLE  # stan bez zmian (brak markera)


# ---------------------------------------------------------------------------
# Testy sanityzacji danych wejściowych
# ---------------------------------------------------------------------------

def test_invalid_state_timeout_falls_back_to_default():
    """Nieprawidłowe wartości timeoutów używają bezpiecznych domyślnych.

    Plik YAML może zawierać błędy (zero, ujemne, tekst).
    Automat powinien działać dalej z domyślnymi wartościami, nie crashować.
    """
    sm = StateMachine(
        {
            **_base_config(),
            'state_timeouts': {
                'search_table': 0,           # zero → nieprawidłowe
                'detect_marker': 'bad-value', # tekst → nieprawidłowe
                'align_with_box': -5.0,       # ujemne → nieprawidłowe
            },
        }
    )

    # Wszystkie trzy powinny wróćić do domyślnych wartości
    assert sm._state_timeouts['search_table'] == 20.0
    assert sm._state_timeouts['detect_marker'] == 20.0
    assert sm._state_timeouts['align_with_box'] == 10.0


def test_missing_marker_ids_use_defaults():
    """Brak konfiguracji ID markerów używa domyślnych wartości."""
    sm = StateMachine({})  # pusta konfiguracja

    # Sprawdzamy wartości domyślne zdefiniowane w StateMachine.__init__
    assert sm._box_marker_id == 10
    assert sm._pickup_table_marker == 21
    assert sm._place_table_marker == 22


# ---------------------------------------------------------------------------
# Test właściwości is_in_sequence_state
# ---------------------------------------------------------------------------

def test_is_in_sequence_state():
    """Weryfikuje flagę is_in_sequence_state dla różnych stanów."""
    sm = StateMachine(_base_config())

    # Stany niebędące sekwencyjnymi
    assert not sm.is_in_sequence_state  # SEARCH_TABLE

    sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()  # → DETECT_MARKER
    assert not sm.is_in_sequence_state

    sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()  # → ALIGN_WITH_BOX
    assert not sm.is_in_sequence_state

    sm.update_offset(0.0, 0.0, 0.0); sm.step()  # → PICK_BOX
    assert sm.is_in_sequence_state  # PICK_BOX jest stanem sekwencyjnym

    sm.notify_sequence_done(); sm.step()  # → ROTATE_180
    assert sm.is_in_sequence_state  # ROTATE_180 też sekwencyjny

    sm.notify_sequence_done(); sm.step()  # → NAVIGATE_TO_TARGET_MARKER
    assert not sm.is_in_sequence_state  # nawigacja nie jest sekwencją
