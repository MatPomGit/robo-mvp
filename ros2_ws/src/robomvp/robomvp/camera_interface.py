#!/usr/bin/env python3
"""Interfejs kamer dla systemu RoboMVP.

ROLA TEGO WĘZŁA:
================
Kamera to „zmysł wzroku" robota. Ten węzeł jest mostem między
sprzętowym strumieniem wideo (urządzenie /dev/videoX) a siecią
tematów ROS2, gdzie inne węzły mogą odbierać obrazy.

DLACZEGO OSOBNY WĘZEŁ DLA KAMER?
Separacja odpowiedzialności (separation of concerns): węzeł kamery
zajmuje się TYLKO pozyskiwaniem obrazów i ich publikacją. Węzły
detekcji markerów nie wiedzą skąd pochodzi obraz – po prostu
subskrybują temat. Gdybyśmy chcieli zamienić kamerę USB na kamerę
głębokości lub symulator, wystarczyłoby zastąpić ten jeden węzeł.

Publikowane tematy:
    /camera/body/image_raw  – obraz kamery ciała (manipulacja)
    /camera/head/image_raw  – obraz kamery głowy (nawigacja)

Parametry ROS2:
    publish_rate       – częstotliwość publikacji [Hz], domyślnie 10.0
    body_camera_device – indeks urządzenia V4L2 kamery ciała, domyślnie 0
    head_camera_device – indeks V4L2 kamery głowy; -1 = użyj obrazu ciała
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraInterface(Node):
    """Węzeł ROS2 publikujący obrazy z kamer robota Unitree G1 EDU."""

    def __init__(self):
        super().__init__('camera_interface')

        # Deklaracja parametrów ROS2.
        # ROS2 wymaga jawnej deklaracji parametrów przed ich odczytem.
        # Domyślna wartość 10.0 Hz to kompromis: wystarczająco szybko
        # dla detekcji markerów, nie przeciążając magistrali DDS.
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('body_camera_device', 0)
        self.declare_parameter('head_camera_device', -1)

        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._body_camera_device = self._read_device_parameter('body_camera_device')
        self._head_camera_device = self._read_device_parameter('head_camera_device')

        # CvBridge konwertuje obrazy między formatem OpenCV (tablica NumPy)
        # a formatem ROS2 (sensor_msgs/Image). Inicjalizujemy raz, nie w każdej klatce.
        self._bridge = CvBridge()

        # Wydawcy tematów kamer.
        # queue_size=10: bufor 10 wiadomości – gdy subskrybent przetwarza
        # wolniej niż 10 Hz, starsze klatki są odrzucane zamiast czekać.
        self._pub_body = self.create_publisher(Image, '/camera/body/image_raw', 10)
        self._pub_head = self.create_publisher(Image, '/camera/head/image_raw', 10)

        # Kamera ciała jest wymagana – bez niej cały pipeline jest bezużyteczny
        self._body_cap = self._open_capture(self._body_camera_device, 'ciała')
        if self._body_cap is None:
            raise RuntimeError(
                'Nie udało się otworzyć wymaganej kamery ciała. '
                'Sprawdź parametr body_camera_device i /dev/video*.'
            )

        # Kamera głowy jest opcjonalna – gdy brak, mapujemy obraz z kamery ciała.
        # To umożliwia uruchomienie z jedną kamerą (parametr head_camera_device:=-1).
        self._head_cap = None
        self._use_body_as_head = False
        if self._head_camera_device < 0:
            self._use_body_as_head = True
            self.get_logger().warning(
                'Kamera głowy wyłączona (head_camera_device < 0). '
                'Publikuję obraz z kamery ciała na /camera/head/image_raw.'
            )
        else:
            self._head_cap = self._open_capture(self._head_camera_device, 'głowy')
            if self._head_cap is None:
                self._use_body_as_head = True
                self.get_logger().warning(
                    'Nie można otworzyć kamery głowy – przełączam na obraz z kamery ciała.'
                )

        # Timer publikowania: create_timer(okres, callback)
        # Okres = 1/częstotliwość. Przy 10 Hz timer odpala co 100 ms.
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_images)

        head_description = (
            'body_fallback' if self._use_body_as_head
            else f'/dev/video{self._head_camera_device}'
        )
        self.get_logger().info(
            f'Kamery zainicjalizowane. '
            f'body=/dev/video{self._body_camera_device}, '
            f'head={head_description}.'
        )

    def _read_device_parameter(self, name: str) -> int:
        """Odczytuje indeks urządzenia kamery z parametru ROS2.

        ROS2 może przekazać parametr jako int lub string (z linii poleceń).
        Konwersja int() obsługuje oba przypadki.
        """
        param = self.get_parameter(name).value
        try:
            return int(param)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'Parametr {name} musi być liczbą całkowitą, otrzymano: {param!r}'
            ) from exc

    def _open_capture(self, device_index: int, camera_name: str):
        """Otwiera urządzenie kamerowe przez OpenCV VideoCapture.

        VideoCapture(index) otwiera kamerę /dev/video{index} przez V4L2.
        Musimy sprawdzić isOpened() – VideoCapture() nie rzuca wyjątku
        gdy urządzenie nie istnieje, tylko tworzy nieotwartą instancję.
        """
        cap = cv2.VideoCapture(device_index)
        if cap.isOpened():
            self.get_logger().info(
                f'Kamera {camera_name} otwarta (/dev/video{device_index}).'
            )
            return cap
        cap.release()
        self.get_logger().error(
            f'Nie można otworzyć kamery {camera_name} (/dev/video{device_index}). '
            'Sprawdź czy urządzenie istnieje i ma odpowiednie uprawnienia '
            '(dodaj użytkownika do grupy video: sudo usermod -aG video $USER).'
        )
        return None

    def _publish_images(self):
        """Callback timera: odczytuje klatki i publikuje je na tematy ROS2."""
        # Znacznik czasu dla obu kamer – taki sam żeby zachować synchronizację.
        # Węzły downstream (marker_detection) mogą zakładać że klatki z obu
        # kamer z tym samym stampem są z tej samej chwili.
        stamp = self.get_clock().now().to_msg()

        body_img = self._read_capture(self._body_cap, 'ciała')
        head_img = body_img if self._use_body_as_head else self._read_capture(
            self._head_cap, 'głowy'
        )

        # cv2_to_imgmsg: konwersja numpy.ndarray (BGR) → sensor_msgs/Image
        # encoding='bgr8': OpenCV domyślnie używa BGR (nie RGB!)
        body_msg = self._bridge.cv2_to_imgmsg(body_img, encoding='bgr8')
        body_msg.header.stamp = stamp
        body_msg.header.frame_id = 'body_camera_frame'
        self._pub_body.publish(body_msg)

        head_msg = self._bridge.cv2_to_imgmsg(head_img, encoding='bgr8')
        head_msg.header.stamp = stamp
        head_msg.header.frame_id = 'head_camera_frame'
        self._pub_head.publish(head_msg)

    def _read_capture(self, cap, camera_name: str):
        """Odczytuje jedną klatkę z VideoCapture.

        cap.read() zwraca (True, frame) przy sukcesie lub (False, None) przy błędzie.
        Rzucamy RuntimeError zamiast zwracać None – pozwala to wywołującemu
        kodowi reagować na błąd przez wyjątek zamiast sprawdzania None.
        """
        if cap is not None and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                return frame
        raise RuntimeError(
            f'Błąd odczytu klatki z kamery {camera_name}. '
            'Sprawdź połączenie kamery.'
        )

    def destroy_node(self):
        """Zwalnia zasoby kamer przed zniszczeniem węzła.

        VideoCapture.release() zamyka urządzenie V4L2. Bez tego
        kamera mogłaby pozostać zablokowana po zakończeniu procesu
        i nie dać się otworzyć przy kolejnym uruchomieniu.
        """
        if self._body_cap is not None:
            self._body_cap.release()
            self._body_cap = None
        if self._head_cap is not None:
            self._head_cap.release()
            self._head_cap = None
        super().destroy_node()


def main(args=None):
    """Punkt wejścia węzła camera_interface."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Błąd krytyczny camera_interface: {exc}')
        else:
            print(f'Błąd krytyczny camera_interface: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
