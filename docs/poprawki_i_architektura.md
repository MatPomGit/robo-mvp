# Dokumentacja poprawek i decyzji architektonicznych RoboMVP

## Spis treści

1. [Przegląd znalezionych błędów](#1-przegląd-znalezionych-błędów)
2. [Błąd krytyczny: automat stanowy pomijał sekwencje](#2-błąd-krytyczny-automat-stanowy-pomijał-sekwencje)
3. [Błąd krytyczny: NameError w trybie demo](#3-błąd-krytyczny-nameerror-w-trybie-demo)
4. [Błąd bezpieczeństwa: robot nie zatrzymuje się przy Ctrl+C](#4-błąd-bezpieczeństwa-robot-nie-zatrzymuje-się-przy-ctrlc)
5. [Błąd architektoniczny: blokowanie executora ROS2](#5-błąd-architektoniczny-blokowanie-executora-ros2)
6. [Błąd wykrywania markerów: pomylone typy detektorów](#6-błąd-wykrywania-markerów-pomylone-typy-detektorów)
7. [Błąd instalacji: nieprawidłowa ścieżka plików konfiguracyjnych](#7-błąd-instalacji-nieprawidłowa-ścieżka-plików-konfiguracyjnych)
8. [Import cv2 wewnątrz metody](#8-import-cv2-wewnątrz-metody)
9. [Import math wewnątrz funkcji](#9-import-math-wewnątrz-funkcji)
10. [Nowa metoda: notify_sequence_done()](#10-nowa-metoda-notify_sequence_done)
11. [Zaktualizowane testy jednostkowe](#11-zaktualizowane-testy-jednostkowe)

---

## 1. Przegląd znalezionych błędów

| # | Plik | Typ błędu | Wpływ |
|---|------|-----------|-------|
| 1 | `motion_sequences.py` | `NameError` – zmienna `total` niezdefiniowana | Crash w trybie demo (bez robota) |
| 2 | `state_machine.py` | Natychmiastowe przejścia w stanach sekwencyjnych | Robot nigdy nie podnosi pudełka ani się nie obraca |
| 3 | `unitree_robot_api.py` | `StopMove()` nie wywoływane przy Ctrl+C | Robot kontynuuje ruch po zakończeniu programu |
| 4 | `main_node.py` | Blokujące `execute_sequence()` w callbacku timera | Brak odbioru wiadomości ROS2 podczas ruchu |
| 5 | `marker_detection.py` | `hasattr` nie rozróżnia typów detektorów | Błędne wywołanie metody QR zamiast AprilTag |
| 6 | `setup.py` | Ścieżka względna `../../../config/` | Pliki YAML nie są kopiowane przy `colcon build` |
| 7 | `marker_pose_estimator.py` | Import `cv2` wewnątrz metody | Późne wykrycie błędu instalacji |
| 8 | `motion_sequences.py` | Import `math` wewnątrz funkcji | Naruszenie PEP-8, słaba czytelność |

---

## 2. Błąd krytyczny: automat stanowy pomijał sekwencje

### Na czym polegał problem

W oryginalnej implementacji handlery stanów `_handle_pick_box()` i `_handle_rotate_180()` wyglądały tak:

```python
def _handle_pick_box(self):
    self._log('Sekwencja podniesienia pudełka wykonana. Przechodzę do obrotu.')
    self._transition_to(State.ROTATE_180)  # ← natychmiastowe przejście!
```

Metoda `step()` była wywoływana przez timer w `main_node._step()`. Logika w `main_node` wyglądała tak:

```python
def _step(self):
    previous_state = self._state_machine.current_state   # np. ALIGN_WITH_BOX
    new_state = self._state_machine.step()               # wywołuje step()
    if new_state != previous_state:
        self._execute_state_action(new_state)            # uruchom akcję dla nowego stanu
```

Problem: gdy robot był w stanie `ALIGN_WITH_BOX` i warunki wyrównania były spełnione, metoda `step()` robiła:

1. Wchodzi do `_handle_align_with_box()` → offset OK → `_transition_to(PICK_BOX)`
2. Wraca do `step()` i...

Zaraz, nie — `step()` wykonuje tylko **jeden handler** per wywołanie. Spójrzmy dokładniej:

```
Wywołanie 1 step(): ALIGN → _handle_align_with_box → OK → przejście do PICK_BOX → return PICK_BOX
main_node widzi: previous=ALIGN, new=PICK_BOX → wywołuje _execute_state_action(PICK_BOX)
Wywołuje execute_sequence(get_pick_box())  ← to działa!

Wywołanie 2 step(): PICK_BOX → _handle_pick_box → NATYCHMIAST → przejście do ROTATE_180 → return ROTATE_180
main_node widzi: previous=PICK_BOX, new=ROTATE_180 → wywołuje _execute_state_action(ROTATE_180)
```

Problem jest subtelniejszy: **akcja dla PICK_BOX jest wprawdzie wywoływana, ale równolegle z natychmiastowym przejściem do ROTATE_180**. Sekwencja `get_pick_box()` jest uruchamiana (w nowym wątku po naszej poprawce), ale automat stanowy już jest w ROTATE_180 i w następnym timerze uruchamia sekwencję obrotu – **zanim zakończyło się podniesienie pudełka**.

W oryginalnym kodzie (bez wątków, z blokującym `execute_sequence` w timerze) było jeszcze gorzej: `execute_sequence` blokowała timer na czas ruchu. Podczas blokady nie mogły zajść żadne zmiany stanu. Kiedy timer odblokował się, `step()` było wywoływane ponownie i `_handle_pick_box()` **natychmiast** przechodziło do ROTATE_180, uruchamiając obrót bez czekania na rzeczywiste zakończenie sekwencji podniesienia.

### Rozwiązanie

Wprowadzono nową metodę `notify_sequence_done()` i flagę `_sequence_done`:

```python
# W StateMachine:
def _handle_pick_box(self):
    if self._sequence_done:   # czekamy na sygnał z zewnątrz
        self._log('Sekwencja podniesienia zakończona. Przechodzę do ROTATE_180.')
        self._transition_to(State.ROTATE_180)
    # jeśli _sequence_done = False → nic nie robimy, zostajemy w PICK_BOX

def notify_sequence_done(self):
    if self._state in _SEQUENCE_STATES:
        self._sequence_done = True
```

W `main_node` po zakończeniu sekwencji wątek wywołuje:

```python
# W main_node._run_sequence_in_thread():
if notify_done:
    with self._state_lock:
        self._state_machine.notify_sequence_done()
```

Dzięki temu przepływ jest deterministyczny:

```
timer: step() → PICK_BOX (sequence_done=False) → brak przejścia → nic
wątek: execute_sequence(get_pick_box()) → zakończone → notify_sequence_done()
timer: step() → PICK_BOX (sequence_done=True) → przejście do ROTATE_180
timer: step() → ROTATE_180 (sequence_done=False) → brak przejścia
wątek: execute_sequence(get_rotate_180()) → zakończone → notify_sequence_done()
timer: step() → ROTATE_180 (sequence_done=True) → przejście do NAVIGATE_TO_TARGET_MARKER
```

---

## 3. Błąd krytyczny: NameError w trybie demo

### Lokalizacja

Plik `motion_sequences.py`, funkcja `execute_sequence()`, linia z f-stringiem:

```python
logger.info(
    f'[no_robot] Krok {i + 1}/{total}: '  # ← 'total' nie istnieje w tym scope!
```

### Dlaczego pozostawało niezauważone

Błąd ujawniał się **tylko** gdy `robot_api is None` (tryb demo, brak połączenia z robotem). W trybie sprzętowym ta gałąź kodu nigdy nie była wykonywana. Ponieważ testy zwykle uruchamia się bez robota, błąd powinien być widoczny od razu — ale brak automatycznych testów dla `execute_sequence()` sprawiał, że go nie wykryto.

### Poprawka

```python
total = len(sequence)   # dodano przed pętlą for
for i, pose in enumerate(sequence):
    ...
    logger.info(f'[no_robot] Krok {i + 1}/{total}: ...')   # ← teraz total jest zdefiniowane
```

---

## 4. Błąd bezpieczeństwa: robot nie zatrzymuje się przy Ctrl+C

### Lokalizacja

Plik `unitree_robot_api.py`, metoda `_send_velocity()`:

```python
def _send_velocity(self, vx, vy, vyaw, duration_s):
    self._loco_client.SetVelocity(vx, vy, vyaw, duration_s)
    time.sleep(duration_s)          # ← jeśli tu przyjdzie KeyboardInterrupt...
    self._loco_client.StopMove()    # ← ...ta linia NIGDY nie jest wykonywana
```

### Dlaczego to jest problem bezpieczeństwa

Robot Unitree G1 EDU waży ~35 kg i może się przemieszczać z prędkością do 3 m/s. Jeśli operator naciśnie Ctrl+C w połowie ruchu, a `StopMove()` nie zostanie wywołane, robot kontynuuje ruch przez `duration_s` sekund z pełną prędkością — bez nadzoru oprogramowania.

### Poprawka

```python
def _send_velocity(self, vx, vy, vyaw, duration_s):
    self._loco_client.SetVelocity(vx, vy, vyaw, duration_s)
    try:
        time.sleep(duration_s)
    finally:
        # 'finally' gwarantuje wykonanie ZAWSZE – czy byl wyjątek czy nie.
        # Zatrzymuje robota nawet gdy wątek jest przerywany przez Ctrl+C.
        try:
            self._loco_client.StopMove()
        except Exception:
            pass   # połączenie mogło już być zerwane podczas shutdown
```

Blok `finally` w Pythonie wykonuje się niezależnie od tego czy w `try` wystąpił wyjątek. Jest to jedyny pewny sposób na zagwarantowanie cleanup operations w Pythonie.

---

## 5. Błąd architektoniczny: blokowanie executora ROS2

### Na czym polega problem

ROS2 używa modelu **single-threaded executor**: wszystkie callbacki (timery, subskrypcje, serwisy) są obsługiwane sekwencyjnie w jednym wątku. Gdy jeden callback blokuje (czeka), reszta czeka w kolejce.

Oryginalny kod wywoływał `execute_sequence()` bezpośrednio z callbacku timera:

```python
def _step(self):                               # ← callback timera ROS2
    ...
    self._run_sequence(sequence, 'pick_box')   # ← wywołuje execute_sequence()
                                               # ← execute_sequence() woła time.sleep(5s)
                                               # ← CAŁY EXECUTOR JEST ZABLOKOWANY NA 5 SEKUND
```

W czasie blokady:
- `_on_marker_pose()` **nie jest wywoływane** → automat nie dostaje nowych danych o markerze
- `_on_offset()` **nie jest wywoływane** → offset korekcji nie jest aktualizowany
- Timer `_step()` **nie może odpalić** → automat stanowy zatrzymuje się

### Rozwiązanie: wątki dla sekwencji blokujących

Sekwencje ruchu są uruchamiane w osobnych wątkach przez `threading.Thread`:

```python
def _launch_sequence_thread(self, sequence, sequence_name, notify_done):
    thread = threading.Thread(
        target=self._run_sequence_in_thread,
        args=(sequence, sequence_name, notify_done),
        daemon=True,   # ← wątek kończy się z procesem głównym
    )
    thread.start()
```

Wątek demona (`daemon=True`) jest ważny: bez niego program mógłby „wisieć" po Ctrl+C, czekając na zakończenie wątku sekwencji.

Dostęp do automatu stanowego jest chroniony mutexem:

```python
self._state_lock = threading.Lock()

# W timerze:
with self._state_lock:
    new_state = self._state_machine.step()

# W wątku sekwencji:
with self._state_lock:
    self._state_machine.notify_sequence_done()
```

Mutex zapobiega **race condition**: sytuacji, w której timer i wątek sekwencji jednocześnie modyfikują automat stanowy, prowadząc do niespójnego stanu.

---

## 6. Błąd wykrywania markerów: pomylone typy detektorów

### Na czym polegał problem

Oryginalny `_detect_markers()` sprawdzał typ detektora przez `hasattr`:

```python
if self._marker_type == 'apriltag' and hasattr(self._detector, 'detect'):
    results = self._detect_apriltags(gray)
elif hasattr(self._detector, 'detectAndDecode'):
    results = self._detect_qr(color)
```

Problem: `cv2.QRCodeDetector` **też ma metodę `detect()`** (używaną do lokalizacji kodu QR bez dekodowania). Gdy biblioteka `apriltag` nie była dostępna i użyto `QRCodeDetector` jako fallback, `marker_type` był zmieniony na `'qr'`, więc pierwszy warunek był False. To działało. Jednak kolejność warunków mogła prowadzić do pomyłek gdy ktoś zmieniał logikę inicjalizacji.

### Rozwiązanie

Osobna flaga logiczna zamiast hasattr:

```python
self._use_apriltag: bool = False   # ustawiana w _init_detector()

def _detect_markers(self, gray, color):
    if self._use_apriltag:
        return self._detect_apriltags(gray)
    else:
        return self._detect_qr(color)
```

Flaga `_use_apriltag` jest ustawiana na `True` tylko gdy import `apriltag` się powiedzie. Jest jednoznaczna i nie zależy od atrybutów obiektu detektora.

---

## 7. Błąd instalacji: nieprawidłowa ścieżka plików konfiguracyjnych

### Na czym polegał problem

W `setup.py`:

```python
(join('share', package_name, 'config'), glob('../../../config/*.yaml'))
```

Wzorzec `../../../config/*.yaml` jest ścieżką **względną od katalogu roboczego procesu `colcon build`**, nie od lokalizacji `setup.py`. Zwykle `colcon build` uruchamia się z `ros2_ws/`, więc `../../../` prowadziłoby do... katalogu nad repozytorium, nie do `config/` projektu.

W efekcie `colcon build` nie kopiował plików `scene.yaml` i `camera.yaml` do `install/share/robomvp/config/`, przez co węzeł nie mógł znaleźć konfiguracji i używał wartości domyślnych (z ostrzeżeniem w logach).

### Poprawka

```python
import os
(
    join('share', package_name, 'config'),
    glob(join(os.path.dirname(__file__), '..', '..', '..', 'config', '*.yaml')),
)
```

`os.path.dirname(__file__)` to katalog `ros2_ws/src/robomvp/`, więc `../../../` prowadzi do katalogu głównego repozytorium gdzie leży `config/`. Ta ścieżka jest stabilna niezależnie od katalogu roboczego procesu budowania.

---

## 8. Import cv2 wewnątrz metody

### Lokalizacja

`marker_pose_estimator.py`, metoda `_estimate_pose()`:

```python
def _estimate_pose(self, msg):
    import cv2   # ← import wewnątrz metody
    ...
```

### Problem

Import wewnątrz metody jest dozwolony w Pythonie, ale powoduje że błąd `ModuleNotFoundError: No module named 'cv2'` pojawia się dopiero gdy metoda zostanie wywołana — czyli po kilku sekundach od startu węzła, gdy przyjdzie pierwsza wiadomość z kamery. Utrudnia to diagnozowanie problemów z instalacją.

### Poprawka

Przeniesienie importu na poziom modułu (góra pliku):

```python
import cv2   # ← teraz błąd pojawia się przy starcie węzła
```

---

## 9. Import math wewnątrz funkcji

### Lokalizacja

`motion_sequences.py`, funkcje `get_rotate_180()` i `get_walk_to_second_table()`:

```python
def get_rotate_180():
    import math   # ← naruszenie PEP-8
    return [...]
```

### Problem

Poza naruszeniem PEP-8, import wewnątrz funkcji ukrywa zależności modułu. Czytając nagłówek pliku nie widać że moduł używa `math`. Konwencja PEP-8 nakazuje wszystkie importy umieszczać na początku pliku.

### Poprawka

Import `math` na poziomie modułu, jako drugi import po `import time`.

---

## 10. Nowa metoda: notify_sequence_done()

Aby naprawić błąd #2, dodano do `StateMachine` nową publiczną metodę oraz zestaw pomocniczych mechanizmów:

```python
# Stałe: zbiór stanów wymagających notyfikacji
_SEQUENCE_STATES = frozenset({
    State.PICK_BOX,
    State.ROTATE_180,
    State.PLACE_BOX,
})

# Flaga w instancji
self._sequence_done: bool = False

# Metoda publiczna
def notify_sequence_done(self):
    if self._state in _SEQUENCE_STATES:
        self._sequence_done = True

# Właściwość pomocnicza
@property
def is_in_sequence_state(self) -> bool:
    return self._state in _SEQUENCE_STATES
```

Flaga `_sequence_done` jest resetowana do `False` przy każdym przejściu stanu (w `_transition_to()`), żeby stary sygnał z poprzedniej sekwencji nie wpłynął na nową.

---

## 11. Zaktualizowane testy jednostkowe

Testy w `tests/test_state_machine.py` zostały przepisane aby odzwierciedlały poprawioną architekturę:

Poprzednia wersja testu `test_navigate_to_target_accepts_target_marker` zakładała natychmiastowe przejścia:

```python
sm.update_offset(0.0, 0.0, 0.0)
assert sm.step() == State.PICK_BOX
assert sm.step() == State.ROTATE_180          # ← błędne założenie
assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER  # ← błędne założenie
```

Nowe testy sprawdzają zarówno **oczekiwanie** (bez notify → brak przejścia) jak i **przejście po notyfikacji**:

```python
sm.update_offset(0.0, 0.0, 0.0)
assert sm.step() == State.PICK_BOX

# Robot czeka w PICK_BOX
assert sm.step() == State.PICK_BOX, 'PICK_BOX nie powinno przejść bez notify_sequence_done()'

# Dopiero po notyfikacji
sm.notify_sequence_done()
assert sm.step() == State.ROTATE_180
```

Dodano również nowe testy: `test_align_with_box_waits_when_offset_too_large`, `test_navigate_does_not_stop_when_marker_too_far`, `test_notify_sequence_done_only_affects_sequence_states` i `test_is_in_sequence_state`.
