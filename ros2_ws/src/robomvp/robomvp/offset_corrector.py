#!/usr/bin/env python3
"""Moduł korekcji offsetu ruchu dla systemu RoboMVP.

DLACZEGO OSOBNY MODUŁ DLA KOREKCJI?
Korekcja offsetu to obliczenie czysto matematyczne, niezależne od ROS2,
SDK ani automatu stanowego. Osobna klasa OffsetCorrector jest:
- Łatwiejsza do testowania jednostkowego (nie wymaga węzłów ROS2)
- Wielokrotnego użytku (może być importowana z różnych węzłów)
- Konfigurowalnie skalowalna (współczynniki scale_dx, scale_dy, scale_dz)

W bieżącej architekturze RoboMVP korekcja odbywa się głównie przez
apply_offset_to_sequence() w motion_sequences.py i bezpośrednio w
main_node.py. Ten moduł jest dostępny do użycia gdy potrzeba bardziej
zaawansowanej korekcji, np. z oczekiwanymi pozycjami z scene.yaml.
"""


class OffsetCorrector:
    """Kalkulator korekcji offsetu pozycji robota.

    Porównuje oczekiwaną pozę markera z zmierzoną
    i zwraca wektor korekcji (dx, dy, dz) w metrach.

    KONWENCJA OSI:
    Osie są zdefiniowane w układzie robota (body frame):
    dx – boczna (+ = lewo), dy – przód/tył (+ = przód), dz – góra/dół (+ = góra)
    """

    def __init__(self, scale_dx: float = 1.0, scale_dy: float = 1.0,
                 scale_dz: float = 1.0):
        """Inicjalizuje korektor z współczynnikami skalowania.

        Współczynniki < 1.0 tłumią korekcję (bezpieczniejsze przy niepewnej kalibracji).
        Współczynniki > 1.0 wzmacniają korekcję (używać ostrożnie).
        """
        self._scale_dx = scale_dx
        self._scale_dy = scale_dy
        self._scale_dz = scale_dz

    def compute_offset(
        self,
        measured_x: float, measured_y: float, measured_z: float,
        expected_x: float = 0.0, expected_y: float = 0.5, expected_z: float = 0.8
    ) -> tuple:
        """Oblicza offset korekcji między pozycją zmierzoną a oczekiwaną.

        WAŻNA UWAGA O UKŁADZIE WSPÓŁRZĘDNYCH:
        Układ kamery i układ robota nie są takie same!
        - Oś Z kamery = głębokość (do przodu) → mapuje na oś Y robota (przód/tył)
        - Oś Y kamery = piksele w dół → mapuje na oś Z robota (góra/dół)
        Stąd: raw_dy = expected_y - measured_z (głębokość kamery → przód robota)
              raw_dz = expected_z - measured_y (Y kamery → góra/dół robota)

        Args:
            measured_x/y/z: Zmierzona pozycja markera w układzie kamery [m].
            expected_x/y/z: Oczekiwana pozycja markera w układzie robota [m].

        Returns:
            Krotka (dx, dy, dz) – korekcja w metrach, po przeskalowaniu.
        """
        raw_dx = expected_x - measured_x
        raw_dy = expected_y - measured_z   # głębokość kamery → przód robota
        raw_dz = expected_z - measured_y   # oś Y kamery → pionowa robota

        dx = raw_dx * self._scale_dx
        dy = raw_dy * self._scale_dy
        dz = raw_dz * self._scale_dz

        return dx, dy, dz

    def scale_offset(self, dx: float, dy: float, dz: float) -> tuple:
        """Skaluje już obliczony offset zgodnie z konfiguracją."""
        return (
            dx * self._scale_dx,
            dy * self._scale_dy,
            dz * self._scale_dz,
        )

    def apply_offset_to_pose(
        self,
        original_pose: dict,
        dx: float, dy: float, dz: float
    ) -> dict:
        """Stosuje offset do oryginalnej pozy sekwencji ruchu.

        Zwraca nową kopię słownika pozy – nie modyfikuje oryginału
        (defensive copy, patrz motion_sequences.apply_offset_to_sequence).
        """
        corrected = dict(original_pose)
        corrected['x'] = original_pose.get('x', 0.0) + dx
        corrected['y'] = original_pose.get('y', 0.0) + dy
        corrected['z'] = original_pose.get('z', 0.0) + dz
        return corrected
