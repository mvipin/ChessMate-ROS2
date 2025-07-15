from setuptools import find_packages, setup

package_name = 'chessmate_engine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'chess',  # python-chess library
    ],
    zip_safe=True,
    maintainer='smtuser',
    maintainer_email='rubbotix@gmail.com',
    description='ChessMate chess engine integration with Stockfish',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Production nodes (actively used)
            'topic_chess_engine_server = chessmate_engine.topic_chess_engine_server:main',
            'topic_game_management = chessmate_engine.topic_game_management:main',

            # Legacy nodes (for compatibility)
            'chess_engine_server = chessmate_engine.chess_engine_server:main',
            'chess_game_manager = chessmate_engine.chess_game_manager:main',
            'simple_chess_engine = chessmate_engine.simple_chess_engine:main',
        ],
    },
)
