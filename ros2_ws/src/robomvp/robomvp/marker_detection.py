#!/usr/bin/env python3
"""Moduł detekcji markerów wizualnych dla systemu RoboMVP.

DLACZEGO MARKERY WIZUALNE, NIE LIDAR/GPS?
==========================================
Dostępne technologie lokalizacji:
- GPS: dokładność ~3 m, nie działa w pomieszczeniach
- LIDAR: bardzo dokładny, ale wymaga mapy środowiska (SLAM)
- Markery wizualne: dokładność ~1 cm, tanie, deterministyczne

Markery wizualne AprilTag to czarno-białe wzory (podobne do QR),
które dają natychmiastową, wiarygodną informację o pozycji 3D
bez potrzeby mapowania środowiska. Idealne dla MVP.

CO TO JEST APRILTAG?
=====================
AprilTag to rodzina znaczników wizualnych zaprojektowana przez
University of Michigan. Każdy marker ma unikalny wzór bitowy
w siatce (np. 6×6 bitów dla rodziny tag36h11). Detektor:
1. Szuka kwadratowych konturów w obrazie
2. Dekoduje wzór bitowy wewnątrz kwadratu
3. Oblicza ID markera i pozycję jego narożników

Rodzina tag36h11: 587 unikalnych markerów, odporność na błędy
bitowe (hamming distance = 11), dobra do aplikacji przemysłowych.

Subskrybowane tematy:
    /camera/body/image_raw  – obraz kamery ciała (bliskie markery)
    /camera/head/image_raw  – obraz kamery głowy (dalekie markery)

Publikowane tematy:
    /robomvp/marker_detections  – wykryte markery (robomvp/MarkerDetection)

Parametry ROS2:
    marker_type – typ markera: 'apriltag' (domyślnie) lub 'qr'
"""

import cv2  # POPRAWKA: import na poziomie modułu, nie wewnątrz metody.
            # Dzięki temu błąd „brak cv2" pojawi się przy starcie węzła,
            # a nie dopiero gdy przyjdzie pierwszy obraz z kamery.
            # Wczesne wykrywanie błędów = łatwiejsze debugowanie.

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from robomvp.msg import MarkerDetection


class MarkerDetectionNode(Node):
    """Węzeł ROS2 wykrywający markery wizualne w obrazach kamer."""

    def __init__(self):
        super().__init__('marker_detection')

        self.declare_parameter('marker_type', 'apriltag')
        self._marker_type = (
            self.get_parameter('marker_type').get_parameter_value().string_value
        )

        self._bridge = CvBridge()

        # _use_apriltag: osobna flaga zamiast polegania na hasattr().
        # POPRAWKA (błąd #9): hasattr(detector, 'detect') nie rozróżniał
        # detektora AprilTag od QRCodeDetector, bo oba mają metodę detect()
        # (choć o innej sygnaturze). Osobna flaga jest jednoznaczna.
        self._use_apriltag: bool = False
        self._detector = self._init_detector()

        # Subskrypcje obu kamer.
        # queue_size=10: ROS2 buforuje max 10 wiadomości gdy callback
        # jest zajęty. Przy 10 Hz kamery i czasie przetwarzania < 100 ms
        # nie powinniśmy tracić klatek.
        self._sub_body = self.create_subscription(
            Image, '/camera/body/image_raw', self._on_body_image, 10
        )
        self._sub_head = self.create_subscription(
            Image, '/camera/head/image_raw', self._on_head_image, 10
        )

        self._pub_detections = self.create_publisher(
            MarkerDetection, '/robomvp/marker_detections', 10
        )

        self.get_logger().info(
            f'Węzeł detekcji markerów uruchomiony '
            f'(typ: {"apriltag" if self._use_apriltag else "qr"}). '
            'Oczekiwanie na obrazy z kamer.'
        )

    def _init_detector(self):
        """Inicjalizuje detektor markerów AprilTag lub QR (fallback).

        Hierarchia prób:
        1. Jeśli marker_type == 'apriltag': próbuj AprilTag
        2. Jeśli import apriltag nie powiedzie się: fallback do QR z ostrzeżeniem
        3. Jeśli marker_type == 'qr': od razu QR

        Returns:
            Obiekt detektora: apriltag.Detector lub cv2.QRCodeDetector.
        """
        if self._marker_type == 'apriltag':
            try:
                import apriltag
                options = apriltag.DetectorOptions(families='tag36h11')
                detector = apriltag.Detector(options)
                self._use_apriltag = True  # ustawienie flagi przy sukcesie
                return detector
            except ImportError:
                self.get_logger().warning(
                    'Biblioteka apriltag niedostępna – używam QR jako zamiennika. '
                    'Aby zainstalować AprilTag: pip install apriltag'
                )
                # self._use_apriltag pozostaje False
        # Fallback: QR detektor z OpenCV (zawsze dostępny z cv2)
        return cv2.QRCodeDetector()

    def _on_body_image(self, msg: Image):
        """Przetwarza obraz z kamery ciała (bliskie markery, manipulacja)."""
        self._process_image(msg, source='body')

    def _on_head_image(self, msg: Image):
        """Przetwarza obraz z kamery głowy (dalekie markery, nawigacja)."""
        self._process_image(msg, source='head')

    def _process_image(self, msg: Image, source: str):
        """Konwertuje obraz ROS2 → OpenCV, wykrywa markery, publikuje wyniki.

        DLACZEGO OBRAZ SZARY (GRAYSCALE)?
        Detekcja markerów działa na kanale luminancji – kolorowe kanały RGB
        tylko dodają szum i spowalniają obliczenia. Konwersja BGR→GRAY
        to operacja O(n) ale eliminuje 2/3 danych do przetworzenia.
        """
        try:
            # imgmsg_to_cv2: konwersja wiadomości ROS2 Image → tablica NumPy
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            detections = self._detect_markers(gray, cv_image)
            for detection in detections:
                self._pub_detections.publish(detection)
                self.get_logger().debug(
                    f'[{source}] Marker ID={detection.marker_id} '
                    f'@ ({detection.image_x:.1f}, {detection.image_y:.1f}) px'
                )
        except Exception as e:
            self.get_logger().error(
                f'Błąd przetwarzania obrazu [{source}]: {e}. '
                'Sprawdź poprawność wiadomości obrazu i instalację cv_bridge.'
            )

    def _detect_markers(self, gray: np.ndarray, color: np.ndarray) -> list:
        """Wykrywa markery w obrazie i zwraca listę MarkerDetection.

        POPRAWKA (błąd #9): Używamy self._use_apriltag zamiast hasattr().
        Poprzednia wersja sprawdzała hasattr(detector, 'detect'), ale
        cv2.QRCodeDetector też ma metodę detect() – co powodowało wywołanie
        błędnej metody gdy AprilTag był niedostępny i użyto QR jako fallback.
        """
        if self._use_apriltag:
            return self._detect_apriltags(gray)
        else:
            return self._detect_qr(color)

    def _detect_apriltags(self, gray: np.ndarray) -> list:
        """Wykrywa markery AprilTag i zwraca listę MarkerDetection.

        Rozmiar markera w pikselach obliczamy jako średnią z dwóch przekątnych
        kwadratu. To jest bardziej stabilne niż długość boku, bo uwzględnia
        perspektywę (marker widziany pod kątem ma różne długości boków,
        ale przekątne są bardziej symetryczne).
        """
        results = []
        try:
            detections = self._detector.detect(gray)
            for d in detections:
                msg = MarkerDetection()
                msg.marker_id = int(d.tag_id)
                msg.image_x = float(d.center[0])
                msg.image_y = float(d.center[1])
                # Rozmiar jako średnia długość przekątnych kwadratu markera
                corners = d.corners
                size = float(np.mean([
                    np.linalg.norm(corners[0] - corners[2]),
                    np.linalg.norm(corners[1] - corners[3])
                ]))
                msg.size = size
                results.append(msg)
        except Exception as e:
            self.get_logger().warning(
                f'Błąd detekcji AprilTag: {e}. '
                'Sprawdź czy biblioteka apriltag jest poprawnie zainstalowana.'
            )
        return results

    def _detect_qr(self, color: np.ndarray) -> list:
        """Wykrywa kody QR i zwraca listę MarkerDetection.

        Kody QR powinny zawierać dane w formacie 'ID:N' lub samą cyfrę,
        np. 'ID:10' → marker_id=10, lub '21' → marker_id=21.
        """
        results = []
        try:
            data, points, _ = self._detector.detectAndDecode(color)
            if points is not None and data:
                points = points[0]
                center_x = float(np.mean(points[:, 0]))
                center_y = float(np.mean(points[:, 1]))
                size = float(np.mean([
                    np.linalg.norm(points[0] - points[2]),
                    np.linalg.norm(points[1] - points[3])
                ]))
                marker_id = self._parse_qr_id(data)
                msg = MarkerDetection()
                msg.marker_id = marker_id
                msg.image_x = center_x
                msg.image_y = center_y
                msg.size = size
                results.append(msg)
        except Exception as e:
            self.get_logger().warning(
                f'Błąd detekcji QR: {e}. '
                'Sprawdź czy marker QR jest wyraźnie widoczny w obrazie.'
            )
        return results

    def _parse_qr_id(self, data: str) -> int:
        """Parsuje identyfikator z danych kodu QR.

        Obsługiwane formaty:
            'ID:10'  → 10  (preferowany format – jawna etykieta)
            '21'     → 21  (sama liczba)
            inne     → 0   (nierozpoznany format → ID=0, ignorowany przez automat)
        """
        try:
            if data.startswith('ID:'):
                return int(data.split(':')[1])
            return int(data)
        except (ValueError, IndexError):
            return 0


def main(args=None):
    """Punkt wejścia węzła marker_detection."""
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
