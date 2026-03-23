#!/usr/bin/env python3
"""Moduł estymacji pozy markerów dla systemu RoboMVP.

JAK OBLICZAMY POZYCJĘ 3D MARKERA Z OBRAZU 2D?
===============================================
To fundamentalne pytanie wizji komputerowej: jak z płaskiego obrazu
(2D) odtworzyć informację o głębokości (3D)?

METODA 1 – Model aparatu otworkowego (pinhole camera):
Jeśli znamy rzeczywisty rozmiar markera (np. 10 cm) i zmierzony rozmiar
w pikselach, możemy obliczyć odległość przez proporcję:

    z = (f * rozmiar_rzeczywisty) / rozmiar_piksele

gdzie f to ogniskowa kamery w pikselach. Im dalej jest marker, tym
mniejszy się wydaje – ta prosta relacja jest sercem całej metody.

Pozycje boczne (x, y) obliczamy z odchylenia środka markera
od centrum obrazu, znając ogniskową:

    x_3d = (x_px - cx) * z / fx
    y_3d = (y_px - cy) * z / fy

METODA 2 – solvePnP (dla pełnej orientacji):
Jeśli mamy narożniki markera (4 punkty 2D → 4 punkty 3D w układzie
markera), OpenCV może rozwiązać układ równań i obliczyć dokładny
wektor obrotu i translacji. Wymaga rozszerzenia MarkerDetection.msg
o 4 narożniki – na razie używamy uproszczonej Metody 1.

Subskrybowane tematy:
    /robomvp/marker_detections  – wykryte markery (robomvp/MarkerDetection)

Publikowane tematy:
    /robomvp/marker_pose  – poza 3D markera (robomvp/MarkerPose)
    /robomvp/offset       – offset korekcji pozycji (robomvp/Offset)

Parametry ROS2:
    camera_config_path  – ścieżka do pliku camera.yaml
    marker_size         – rzeczywisty rozmiar boku markera [m], domyślnie 0.1
"""

import math

import cv2  # POPRAWKA: import na poziomie modułu (był wewnątrz _estimate_pose).
            # Importowanie cv2 wewnątrz metody oznaczało że błąd
            # 'ModuleNotFoundError: cv2' pojawiał się dopiero gdy
            # przyszła pierwsza wiadomość z kamery (po kilku sekundach).
            # Teraz błąd pojawia się natychmiast przy starcie węzła –
            # dużo łatwiej wykryć podczas konfiguracji systemu.

import numpy as np
import rclpy
import yaml
from rclpy.node import Node

from robomvp.msg import MarkerDetection, MarkerPose, Offset


class MarkerPoseEstimatorNode(Node):
    """Węzeł ROS2 obliczający pozycję 3D markerów na podstawie wykryć."""

    def __init__(self):
        super().__init__('marker_pose_estimator')

        self.declare_parameter('camera_config_path', '')
        self.declare_parameter('marker_size', 0.1)

        config_path = (
            self.get_parameter('camera_config_path')
            .get_parameter_value()
            .string_value
        )
        self._marker_size = (
            self.get_parameter('marker_size').get_parameter_value().double_value
        )

        self._camera_matrix, self._dist_coeffs = self._load_camera_params(config_path)

        self._sub = self.create_subscription(
            MarkerDetection,
            '/robomvp/marker_detections',
            self._on_detection,
            10,
        )

        self._pub_pose = self.create_publisher(MarkerPose, '/robomvp/marker_pose', 10)
        self._pub_offset = self.create_publisher(Offset, '/robomvp/offset', 10)

        self.get_logger().info(
            'Węzeł estymacji pozy markerów uruchomiony. '
            f'Rozmiar markera: {self._marker_size:.3f} m. '
            'Oczekiwanie na wykrycia markerów.'
        )

    def _load_camera_params(self, config_path: str):
        """Wczytuje macierz kamery i współczynniki dystorsji z pliku YAML.

        CZYM JEST MACIERZ KAMERY?
        Parametry wewnętrzne (intrinsics) opisują geometrię kamery:
            fx, fy – ogniskowe w pikselach (jak bardzo zoom)
            cx, cy – punkt główny (centrum obrazu, zwykle ≈ szerokość/2)

        Macierz K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        Przetwarza punkty 3D na pikselowe: p = K * P

        WSPÓŁCZYNNIKI DYSTORSJI:
        Prawdziwe soczewki nie są idealne – zniekształcają obraz.
        k1, k2, k3 – dystorsja radialna (zakrzywienie)
        p1, p2     – dystorsja styczna (niesymetria soczewki)
        Korekcja dystorsji jest ważna dla dokładności estymacji pozy.
        """
        default_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        default_dist = np.zeros((5, 1), dtype=np.float64)

        if not config_path:
            self.get_logger().warning(
                'Brak ścieżki do konfiguracji kamery – używam domyślnych parametrów. '
                'Podaj ścieżkę: --ros-args -p camera_config_path:=<ścieżka>'
            )
            return default_matrix, default_dist

        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            cam = cfg.get('body_camera', {})
            cm = cam.get('camera_matrix', {})
            dc = cam.get('distortion_coeffs', {})
            matrix = np.array([
                [cm.get('fx', 600.0), 0.0, cm.get('cx', 320.0)],
                [0.0, cm.get('fy', 600.0), cm.get('cy', 240.0)],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            dist = np.array([
                [dc.get('k1', 0.0)],
                [dc.get('k2', 0.0)],
                [dc.get('p1', 0.0)],
                [dc.get('p2', 0.0)],
                [dc.get('k3', 0.0)]
            ], dtype=np.float64)
            if cfg.get('marker_size'):
                self._marker_size = float(cfg['marker_size'])
            return matrix, dist
        except Exception as e:
            self.get_logger().warning(
                f'Błąd wczytywania konfiguracji kamery: {e}. '
                'Używam domyślnych parametrów kalibracji.'
            )
            return default_matrix, default_dist

    def _on_detection(self, msg: MarkerDetection):
        """Oblicza pozę 3D markera i publikuje wyniki."""
        try:
            pose = self._estimate_pose(msg)
            self._pub_pose.publish(pose)

            offset = self._compute_offset(pose)
            self._pub_offset.publish(offset)

            self.get_logger().debug(
                f'Marker {msg.marker_id}: '
                f'pos=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) m, '
                f'offset=({offset.dx:.3f}, {offset.dy:.3f}, {offset.dz:.3f}) m'
            )
        except Exception as e:
            self.get_logger().error(
                f'Błąd estymacji pozy markera {msg.marker_id}: {e}. '
                'Sprawdź parametry kalibracji kamery.'
            )

    def _estimate_pose(self, msg: MarkerDetection) -> MarkerPose:
        """Oblicza przybliżoną pozycję 3D markera metodą pinhole.

        Wzór: z = (f_avg * marker_size_m) / marker_size_px
        Następnie x_3d, y_3d z proporcji pikseli do odległości.
        Punkt środkowy jest korygowany o dystorsję soczewki przed obliczeniem.
        """
        pose = MarkerPose()
        pose.marker_id = msg.marker_id

        fx = float(self._camera_matrix[0, 0])
        fy = float(self._camera_matrix[1, 1])
        cx = float(self._camera_matrix[0, 2])
        cy = float(self._camera_matrix[1, 2])
        f_avg = (fx + fy) / 2.0

        # Głębokość z rozmiaru markera w obrazie.
        # Większy marker w pikselach = bliżej kamery.
        if msg.size > 0:
            z = (f_avg * self._marker_size) / msg.size
        else:
            z = 1.0  # bezpieczna wartość domyślna gdy rozmiar nieznany

        # Pozycja X, Y na podstawie odchylenia od centrum obrazu.
        # Obliczamy wstępnie, potem nadpiszemy po korekcji dystorsji.
        x = (msg.image_x - cx) * z / fx
        y = (msg.image_y - cy) * z / fy

        pose.x = float(x)
        pose.y = float(y)
        pose.z = float(z)

        # Kwaternion jednostkowy = brak obrotu (marker równoległy do kamery).
        # Uproszczenie: bez solvePnP nie znamy orientacji markera.
        pose.qx = 0.0
        pose.qy = 0.0
        pose.qz = 0.0
        pose.qw = 1.0

        # Korekcja dystorsji soczewki dla środka markera.
        # undistortPoints() przetwarza punkt pikselowy z zakrzywionego obrazu
        # na punkt w idealnym, niezdystorsowanym układzie.
        # Parametr P=camera_matrix zapewnia że wynik jest w pikselach
        # (bez P wynik byłby w znormalizowanych współrzędnych kamery).
        point = np.array([[[msg.image_x, msg.image_y]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(
            point, self._camera_matrix, self._dist_coeffs, P=self._camera_matrix
        )
        u = float(undistorted[0, 0, 0])
        v = float(undistorted[0, 0, 1])
        pose.x = float((u - cx) * z / fx)
        pose.y = float((v - cy) * z / fy)

        return pose

    def _compute_offset(self, pose: MarkerPose) -> Offset:
        """Oblicza offset korekcji między bieżącą a oczekiwaną pozycją markera.

        INTERPRETACJA OFFSETU:
        Oczekiwana pozycja markera to (x=0, y=0, z=cokolwiek) – marker
        powinien być dokładnie na wprost robota, wycentrowany.
        Jeśli marker jest przesunięty w lewo (pose.x < 0), robot musi
        przesunąć się w lewo (dx > 0). Stąd dx = -pose.x.

        Komponent dy jest zerowy – głębokość (odległość do stołu) jest
        kontrolowana przez sekwencję approach_table, nie przez offset.
        Komponent dz koryguje pionowe odchylenie (pitch kamery).
        """
        offset = Offset()
        offset.dx = -pose.x   # korekcja boczna: przesuń w kierunku markera
        offset.dy = 0.0        # korekcja głębokości: zarządzana przez sekwencję
        offset.dz = -pose.y   # korekcja pionowa: odchylenie w osi Y kamery
        return offset

    def _rvec_to_quaternion(self, rvec: np.ndarray):
        """Konwertuje wektor obrotu Rodriguesa na kwaternion.

        WEKTOR RODRIGUESA (rvec):
        OpenCV reprezentuje orientacje jako wektor, gdzie:
        - kierunek wektora = oś obrotu
        - długość wektora = kąt obrotu w radianach

        Konwersja na kwaternion (qx, qy, qz, qw):
            q = (sin(θ/2) * axis, cos(θ/2))

        TODO: Użyć tej metody gdy _estimate_pose zostanie rozszerzone
        o pełną estymację przez cv2.solvePnP z narożnikami markera.
        """
        angle = float(np.linalg.norm(rvec))
        if angle < 1e-10:
            return 0.0, 0.0, 0.0, 1.0  # kwaternion jednostkowy = brak obrotu
        axis = rvec.flatten() / angle
        s = math.sin(angle / 2.0)
        qx = float(axis[0] * s)
        qy = float(axis[1] * s)
        qz = float(axis[2] * s)
        qw = float(math.cos(angle / 2.0))
        return qx, qy, qz, qw


def main(args=None):
    """Punkt wejścia węzła marker_pose_estimator."""
    rclpy.init(args=args)
    node = MarkerPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
