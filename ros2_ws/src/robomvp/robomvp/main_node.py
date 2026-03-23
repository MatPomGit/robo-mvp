#!/usr/bin/env python3
"""Główny węzeł RoboMVP – orkiestrator całego pipeline demonstracyjnego.

ROLA TEGO WĘZŁA W ARCHITEKTURZE:
=================================
Ten węzeł to „dyrygent" całego systemu. Nie wykonuje żadnej percepcji
ani obliczeń – deleguje te zadania do innych węzłów (marker_detection,
marker_pose_estimator) i reaguje na ich wyniki, uruchamiając odpowiednie
akcje przez StateMachine i UnitreeRobotAPI.

Przepływ danych:
    kamera → marker_detection → marker_pose_estimator
           → /robomvp/marker_pose + /robomvp/offset
           → robomvp_main (ten węzeł)
           → StateMachine.step() → UnitreeRobotAPI.move_to_pose()

WAŻNA UWAGA O BLOKOWANIU EXECUTORA ROS2:
=========================================
execute_sequence() wywołuje time.sleep() w każdym kroku ruchu
(przez UnitreeRobotAPI._send_velocity). Wywołanie jej bezpośrednio
z callbacku timera ROS2 ZABLOKOWAŁOBY cały executor na czas wykonania
sekwencji – żadne wiadomości (marker_pose, offset) nie byłyby przetwarzane.

ROZWIĄZANIE: execute_sequence() jest uruchamiana w osobnym wątku (threading.Thread).
Automat stanowy jest chroniony mutexem (_state_lock) przed jednoczesnym
dostępem z wątku timera i wątku sekwencji.

W pełnym systemie produkcyjnym lepszym rozwiązaniem byłby ROS2 Action Server
(rclpy.action), który obsługuje długotrwałe zadania natywnie. W tym MVP
wątek jest wystarczający i prostszy w implementacji.

Subskrybowane tematy:
    /robomvp/marker_pose  – poza 3D markera (robomvp/MarkerPose)
    /robomvp/offset       – offset korekcji pozycji (robomvp/Offset)

Publikowane tematy:
    /robomvp/state          – aktualny stan automatu (robomvp/State)
    /robomvp/motion_command – nazwa wykonywanej sekwencji (std_msgs/String)

Parametry ROS2:
    scene_config_path     – ścieżka do pliku scene.yaml
    step_period           – okres kroku automatu [s], domyślnie 1.0
    network_interface     – interfejs Ethernet do robota, domyślnie 'eth0'
    require_robot_connection – True = błąd gdy brak robota, domyślnie False
"""

import threading
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from robomvp.motion_sequences import (
    apply_offset_to_sequence,
    execute_sequence,
    get_approach_table,
    get_pick_box,
    get_place_box,
    get_rotate_180,
    get_walk_to_second_table,
)
from robomvp.msg import MarkerPose, Offset
from robomvp.msg import State as StateMsg
from robomvp.state_machine import State, StateMachine
from robomvp.unitree_robot_api import UnitreeRobotAPI


class RoboMVPMain(Node):
    """Węzeł główny systemu RoboMVP."""

    def __init__(self):
        super().__init__('robomvp_main')

        # Deklaracja parametrów ROS2.
        # Każdy parametr musi być zadeklarowany przed pierwszym użyciem.
        # Wartość domyślna jest używana gdy parametr nie jest przekazany
        # z linii poleceń ani pliku launch.
        self.declare_parameter('scene_config_path', '')
        self.declare_parameter('step_period', 1.0)
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('require_robot_connection', False)

        scene_config_path = self.get_parameter('scene_config_path').value
        step_period = float(self.get_parameter('step_period').value)
        network_interface = str(self.get_parameter('network_interface').value)
        self._require_robot_connection = bool(
            self.get_parameter('require_robot_connection').value
        )

        self._config = self._load_config(scene_config_path)

        # Inicjalizacja API robota.
        # Jeśli połączenie się nie powiedzie i require_robot_connection=False,
        # self._robot_api = None, a sekwencje będą tylko logowane.
        self._robot_api = UnitreeRobotAPI(network_interface=network_interface)
        self._robot_connected = False
        try:
            self._robot_api.connect(logger=self.get_logger())
            self._robot_connected = True
            self.get_logger().info(
                'Połączenie z robotem aktywne – ruch będzie wykonywany na sprzęcie.'
            )
        except RuntimeError as exc:
            if self._require_robot_connection:
                self.get_logger().error(
                    f'Nie można połączyć z robotem: {exc}. '
                    'Parametr require_robot_connection=True wymaga aktywnego połączenia.'
                )
                raise
            self.get_logger().warning(
                f'Nie można połączyć z robotem: {exc}. '
                'Kontynuuję bez połączenia sprzętowego '
                '(sekwencje ruchów będą logowane, nie wykonywane).'
            )
            self._robot_api = None

        # Współczynniki skalowania offsetu (z konfiguracji sceny)
        offset_scale = self._config.get('offset_scale', {})
        self._scale_dx = float(offset_scale.get('dx', 1.0))
        self._scale_dy = float(offset_scale.get('dy', 1.0))
        self._scale_dz = float(offset_scale.get('dz', 1.0))

        self._state_machine = StateMachine(self._config, logger=self.get_logger())

        motion_timeouts = self._config.get('motion_timeouts', {})
        self._motion_total_timeout_s = self._sanitize_timeout(
            motion_timeouts.get('total', 30.0),
            default_value=30.0,
            key='motion_timeouts.total',
        )
        self._motion_step_timeout_s = self._sanitize_timeout(
            motion_timeouts.get('step', 5.0),
            default_value=5.0,
            key='motion_timeouts.step',
        )

        # Subskrypcje tematów percepcji
        self._sub_pose = self.create_subscription(
            MarkerPose, '/robomvp/marker_pose', self._on_marker_pose, 10
        )
        self._sub_offset = self.create_subscription(
            Offset, '/robomvp/offset', self._on_offset, 10
        )

        # Wydawcy
        self._pub_state = self.create_publisher(StateMsg, '/robomvp/state', 10)
        self._pub_motion = self.create_publisher(String, '/robomvp/motion_command', 10)

        # Bieżący offset korekcji (aktualizowany przez _on_offset)
        self._current_offset = (0.0, 0.0, 0.0)

        # MUTEX dla bezpieczeństwa wątkowego.
        # Wątek timera (_step) i wątek sekwencji (_run_sequence_thread)
        # oba modyfikują/odczytują automat stanowy. Mutex zapobiega
        # sytuacji wyścigu (race condition), gdzie oba wątki jednocześnie
        # zmieniają stan automatu.
        self._state_lock = threading.Lock()

        # Flaga czy sekwencja jest aktualnie wykonywana.
        # Zapobiega uruchomieniu drugiej sekwencji gdy pierwsza jeszcze trwa.
        self._sequence_running = False

        # Timer kroków automatu – wywołuje _step() co step_period sekund
        self._timer = self.create_timer(step_period, self._step)

        self.get_logger().info(
            'Węzeł główny RoboMVP uruchomiony. '
            f'Połączenie z robotem: {"aktywne" if self._robot_connected else "brak"}. '
            'Oczekiwanie na dane z czujników.'
        )

    # ------------------------------------------------------------------
    # Wczytywanie konfiguracji
    # ------------------------------------------------------------------

    def _load_config(self, config_path: str) -> dict:
        """Wczytuje konfigurację sceny z pliku YAML.

        Jeśli ścieżka nie jest podana lub plik nie istnieje, przeszukuje
        typowe lokalizacje względem pliku modułu i zwraca domyślną konfigurację.
        """
        if not config_path:
            current = Path(__file__).resolve()
            # Szukamy scene.yaml w katalogu config/ w różnych poziomach
            # hierarchii katalogów (rozbudowane repozytoria mogą mieć
            # różne struktury katalogów zależnie od sposobu instalacji)
            possible_paths = [
                *[parent / 'config' / 'scene.yaml' for parent in current.parents],
                Path('/opt/ros2_ws/config/scene.yaml'),
            ]
            for path in possible_paths:
                if path.is_file():
                    config_path = str(path)
                    break

        if config_path and Path(config_path).is_file():
            try:
                with open(config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                self.get_logger().info(f'Załadowano konfigurację sceny: {config_path}')
                return cfg or {}
            except Exception as e:
                self.get_logger().warning(
                    f'Błąd wczytywania konfiguracji: {e}. '
                    'Sprawdź poprawność pliku YAML i uprawnienia do odczytu. '
                    'Używam domyślnych wartości konfiguracji.'
                )

        self.get_logger().warning(
            'Nie znaleziono pliku konfiguracyjnego sceny. Używam domyślnej konfiguracji. '
            'Podaj ścieżkę przez parametr ROS2: --ros-args -p scene_config_path:=<ścieżka>'
        )
        return {
            'stop_distance_threshold': 0.3,
            'alignment_threshold': 0.05,
            'offset_scale': {'dx': 1.0, 'dy': 1.0, 'dz': 1.0},
            'state_timeouts': {
                'search_table': 20.0,
                'detect_marker': 20.0,
                'align_with_box': 10.0,
                'navigate_to_target_marker': 25.0,
            },
            'motion_timeouts': {'total': 30.0, 'step': 5.0},
        }

    def _sanitize_timeout(self, value, default_value: float, key: str) -> float:
        """Zamienia timeout na dodatni float lub używa wartości domyślnej."""
        try:
            timeout = float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Nieprawidłowy timeout "{key}={value}", używam domyślnego {default_value:.2f}s'
            )
            return default_value

        if timeout <= 0.0:
            self.get_logger().warn(
                f'Timeout "{key}" musi być dodatni, używam domyślnego {default_value:.2f}s'
            )
            return default_value

        return timeout

    # ------------------------------------------------------------------
    # Callbacki subskrypcji ROS2
    # ------------------------------------------------------------------

    def _on_marker_pose(self, msg: MarkerPose):
        """Odbiera pozę markera i aktualizuje automat stanowy.

        Ten callback jest wywoływany z wątku executora ROS2.
        Używamy locka bo automat stanowy może być jednocześnie
        czytany przez wątek sekwencji.
        """
        self.get_logger().debug(
            f'Odebrano marker_pose: id={msg.marker_id}, '
            f'x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}'
        )
        with self._state_lock:
            self._state_machine.update_marker(msg.marker_id, msg.x, msg.y, msg.z)

    def _on_offset(self, msg: Offset):
        """Odbiera offset korekcji i aktualizuje automat stanowy."""
        dx = msg.dx * self._scale_dx
        dy = msg.dy * self._scale_dy
        dz = msg.dz * self._scale_dz
        self._current_offset = (dx, dy, dz)
        self.get_logger().debug(
            f'Odebrano offset: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}'
        )
        with self._state_lock:
            self._state_machine.update_offset(dx, dy, dz)

    # ------------------------------------------------------------------
    # Krok automatu stanowego
    # ------------------------------------------------------------------

    def _step(self):
        """Wykonuje jeden krok automatu stanowego i reaguje na zmiany stanu.

        Ta metoda jest wywoływana przez timer ROS2 co step_period sekund.
        Kluczowa zasada: nie blokujemy tutaj – sekwencje ruchu są
        uruchamiane w osobnym wątku przez _launch_sequence_thread().
        """
        with self._state_lock:
            previous_state = self._state_machine.current_state
            new_state = self._state_machine.step()

        # Publikujemy aktualny stan – nawet gdy się nie zmienił,
        # żeby inne węzły i narzędzia monitorujące (rqt, rviz) miały
        # ciągły strumień informacji o stanie systemu.
        state_msg = StateMsg()
        state_msg.state_name = self._state_machine.current_state_name
        state_msg.state_id = int(new_state)
        self._pub_state.publish(state_msg)

        if new_state != previous_state:
            self.get_logger().info(
                f'Zmiana stanu: {previous_state.name} → {new_state.name}. '
                'Uruchamiam akcję dla nowego stanu.'
            )
            self._execute_state_action(new_state)

        if new_state == State.FINISHED:
            self.get_logger().info(
                'Scenariusz zakończony pomyślnie. Zatrzymuję timer automatu.'
            )
            self._timer.cancel()

    def _execute_state_action(self, state: State):
        """Wybiera i uruchamia akcję odpowiednią dla nowego stanu.

        Akcje sekwencyjne (PICK_BOX, ROTATE_180, PLACE_BOX) są uruchamiane
        w wątkach, żeby nie blokować executora ROS2. Pozostałe akcje
        (publikacja komendy, logowanie) są wykonywane natychmiast.
        """
        dx, dy, dz = self._current_offset

        if state == State.DETECT_MARKER:
            self.get_logger().info('Akcja: approach_table (podejście do stołu).')
            self._publish_motion('approach_table')
            sequence = apply_offset_to_sequence(get_approach_table(), dx, dy, dz)
            # approach_table jest sekwencją nawigacyjną, uruchamiamy w wątku
            self._launch_sequence_thread(sequence, 'approach_table', notify_done=False)

        elif state == State.ALIGN_WITH_BOX:
            # ALIGN_WITH_BOX nie ma sekwencji ruchu – automat czeka na
            # zmniejszenie offsetu przez aktualizacje z marker_pose_estimator.
            self.get_logger().info(
                'Akcja: align_with_box – oczekiwanie na wyrównanie offsetu.'
            )
            self._publish_motion('align_with_box')

        elif state == State.PICK_BOX:
            self.get_logger().info('Akcja: pick_box (podniesienie pudełka).')
            self._publish_motion('pick_box')
            sequence = apply_offset_to_sequence(get_pick_box(), dx, dy, dz)
            # notify_done=True: po zakończeniu sekwencji automat dostaje
            # sygnał notify_sequence_done() i może przejść do ROTATE_180
            self._launch_sequence_thread(sequence, 'pick_box', notify_done=True)

        elif state == State.ROTATE_180:
            self.get_logger().info('Akcja: rotate_180 (obrót robota).')
            self._publish_motion('rotate_180')
            self._launch_sequence_thread(get_rotate_180(), 'rotate_180', notify_done=True)

        elif state == State.NAVIGATE_TO_TARGET_MARKER:
            self.get_logger().info(
                'Akcja: walk_to_second_table (nawigacja do celu).'
            )
            self._publish_motion('walk_to_second_table')
            self._launch_sequence_thread(
                get_walk_to_second_table(), 'walk_to_second_table', notify_done=False
            )

        elif state == State.PLACE_BOX:
            self.get_logger().info('Akcja: place_box (odłożenie pudełka).')
            self._publish_motion('place_box')
            sequence = apply_offset_to_sequence(get_place_box(), dx, dy, dz)
            self._launch_sequence_thread(sequence, 'place_box', notify_done=True)

        elif state == State.FINISHED:
            self.get_logger().info('Akcja: finished – scenariusz zakończony.')
            self._publish_motion('finished')

    # ------------------------------------------------------------------
    # Uruchamianie sekwencji w wątkach
    # ------------------------------------------------------------------

    def _launch_sequence_thread(
        self,
        sequence: list,
        sequence_name: str,
        notify_done: bool,
    ):
        """Uruchamia sekwencję ruchu w nowym wątku.

        DLACZEGO WĄTEK?
        execute_sequence() wywołuje time.sleep() (przez SDK Unitree).
        Wywołanie tego bezpośrednio w timerze ROS2 zablokowałoby cały
        executor – żadne subskrypcje (marker_pose, offset) nie byłyby
        przetwarzane przez cały czas ruchu.

        Parametr notify_done decyduje czy po zakończeniu sekwencji
        automatycznie wywołać state_machine.notify_sequence_done().
        Jest to True dla stanów sekwencyjnych (PICK_BOX, ROTATE_180,
        PLACE_BOX) i False dla sekwencji nawigacyjnych (approach_table,
        walk_to_second_table), gdzie automat przechodzi dalej na podstawie
        wykrycia markera, nie zakończenia sekwencji.
        """
        if self._sequence_running:
            self.get_logger().warning(
                f'Próba uruchomienia sekwencji {sequence_name!r} '
                'gdy poprzednia sekwencja jeszcze trwa. Ignoruję.'
            )
            return

        self._sequence_running = True
        thread = threading.Thread(
            target=self._run_sequence_in_thread,
            args=(sequence, sequence_name, notify_done),
            daemon=True,  # wątek demona kończy się gdy główny proces się kończy
            name=f'seq_{sequence_name}',
        )
        thread.start()

    def _run_sequence_in_thread(
        self,
        sequence: list,
        sequence_name: str,
        notify_done: bool,
    ):
        """Cel wątku sekwencji – wykonuje sekwencję i opcjonalnie notyfikuje automat.

        Ta metoda działa w osobnym wątku. Wszelki dostęp do _state_machine
        jest chroniony _state_lock.
        """
        try:
            self.get_logger().info(
                f'Start sekwencji: {sequence_name} ({len(sequence)} kroków). '
                f'Tryb: {"robot" if self._robot_api is not None else "demo"}.'
            )
            ok = execute_sequence(
                sequence,
                robot_api=self._robot_api,
                logger=self.get_logger(),
                total_timeout_s=self._motion_total_timeout_s,
                step_timeout_s=self._motion_step_timeout_s,
            )

            if not ok:
                self.get_logger().error(
                    f'Sekwencja {sequence_name!r} zakończyła się błędem lub timeoutem.'
                )
                # Nie notyfikujemy automatu o zakończeniu – automat zostanie
                # w bieżącym stanie. Timer automatu dalej pracuje i może
                # zareagować na timeout stanu lub nowe dane z czujników.
                return

            self.get_logger().info(f'Sekwencja {sequence_name!r} zakończona sukcesem.')

            if notify_done:
                with self._state_lock:
                    self._state_machine.notify_sequence_done()
                self.get_logger().info(
                    f'Automat stanowy powiadomiony o zakończeniu {sequence_name!r}.'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Nieoczekiwany błąd w wątku sekwencji {sequence_name!r}: {exc}'
            )
        finally:
            # Zawsze zwalniamy flagę – nawet przy wyjątku.
            # 'finally' gwarantuje wykonanie niezależnie od ścieżki kodu.
            self._sequence_running = False

    # ------------------------------------------------------------------
    # Publikowanie
    # ------------------------------------------------------------------

    def _publish_motion(self, command: str):
        """Publikuje nazwę wykonywanej sekwencji na temat /robomvp/motion_command."""
        msg = String()
        msg.data = command
        self._pub_motion.publish(msg)
        self.get_logger().info(f'Wysłano komendę ruchu: {command}')

    # ------------------------------------------------------------------
    # Czyszczenie zasobów
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Rozłącza robota i niszczy węzeł ROS2.

        destroy_node() jest wywoływane przez rclpy przy kończeniu procesu
        (Ctrl+C, kill). Zawsze rozłączamy robota bezpiecznie – robot
        powinien zatrzymać się nawet gdy oprogramowanie się crashuje.
        """
        if self._robot_api is not None:
            self._robot_api.disconnect()
            self._robot_api = None
        super().destroy_node()


def main(args=None):
    """Punkt wejścia węzła robomvp_main."""
    rclpy.init(args=args)
    node = RoboMVPMain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
