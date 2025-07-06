from setuptools import find_packages, setup

package_name = 'chessmate_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware_nodes.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'pillow',  # For LCD image processing
        'adafruit-circuitpython-ssd1306',
        'adafruit-circuitpython-busio',
        # Alternative: 'luma.oled',  # Uncomment if using luma instead
    ],
    zip_safe=True,
    maintainer='smtuser',
    maintainer_email='rubbotix@gmail.com',
    description='ChessMate hardware interface package for Raspberry Pi GPIO, LCD, and Arduino communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotary_encoder_node = chessmate_hardware.rotary_encoder_node:main',
            'lcd_display_node = chessmate_hardware.lcd_display_node:main',
            'arduino_communication_node = chessmate_hardware.arduino_communication_node:main',
            'game_management_node = chessmate_hardware.game_management_node:main',
            'robot_animation_controller = chessmate_hardware.robot_animation_controller:main',
        ],
    },
)
