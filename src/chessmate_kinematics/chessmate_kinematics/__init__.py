# ChessMate Kinematics Package
__version__ = "0.1.0"

from .scara_kinematics import SCARAKinematics
from .chess_coordinate_mapper import ChessBoardMapper

__all__ = ['SCARAKinematics', 'ChessBoardMapper']
