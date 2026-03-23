"""Plik konfiguracyjny pakietu Python dla RoboMVP.

ROLA setup.py W PAKIECIE ROS2:
================================
ROS2 używa dwuwarstwowego systemu budowania: ament_cmake (C++ i konfiguracja)
oraz setuptools Python (instalacja węzłów Python).

Ten plik mówi setuptools:
1. Które pliki Python zainstalować (packages)
2. Gdzie umieścić zasoby statyczne (data_files)
3. Jakie skrypty wykonywalne stworzyć (entry_points)

entry_points to mechanizm, przez który `ros2 run robomvp camera_interface`
działa – ROS2 szuka skryptu o nazwie 'camera_interface' w liście entry_points
i wywołuje wskazaną funkcję (camera_interface:main).
"""

import os
from glob import glob
from os.path import join

from setuptools import setup

package_name = 'robomvp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Rejestracja pakietu w indeksie ament (wymagane przez ROS2)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Plik package.xml – metadane pakietu (zależności, licencja, autor)
        ('share/' + package_name, ['package.xml']),
        # Pliki launch – skrypty uruchamiające węzły
        (join('share', package_name, 'launch'), glob('launch/*.py')),
        # POPRAWKA (błąd #6): Poprzedni kod używał:
        #   glob('../../../config/*.yaml')
        # Ścieżka względna '../../../' jest liczona od katalogu roboczego
        # procesu colcon build, NIE od lokalizacji setup.py. To powodowało
        # że pliki konfiguracyjne nie były kopiowane do install/.
        #
        # Prawidłowe rozwiązanie: ścieżka liczona od lokalizacji setup.py
        # przez os.path.dirname(__file__). Działa niezależnie od tego
        # skąd wywołujemy colcon build.
        (
            join('share', package_name, 'config'),
            glob(join(os.path.dirname(__file__), '..', '..', '..', 'config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboMVP Maintainer',
    maintainer_email='maintainer@robomvp.local',
    description='Minimalna aplikacja ROS2 MVP dla robota humanoidalnego Unitree G1 EDU',
    license='MIT',
    # entry_points definiuje skrypty wykonywalne węzłów ROS2.
    # Format: 'nazwa_skryptu = moduł.python:funkcja_main'
    # Po `colcon build` polecenie `ros2 run robomvp camera_interface`
    # wywołuje robomvp.camera_interface:main()
    entry_points={
        'console_scripts': [
            'camera_interface = robomvp.camera_interface:main',
            'marker_detection = robomvp.marker_detection:main',
            'marker_pose_estimator = robomvp.marker_pose_estimator:main',
            'robomvp_main = robomvp.main_node:main',
        ],
    },
)
