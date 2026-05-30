from abc import ABC, abstractmethod
import numpy as np

class WalkController(ABC):
    """
    Clase abstracta para controladores de marcha.
    Define la interfaz que deben implementar los controladores concretos.
    """

    def __init__(self):
        pass

    @abstractmethod
    def get_motion_command(self):
        """
        Traduce los inputs a un comando de movimiento abstracto.
        Retorna un diccionario estandarizado con las siguientes claves:
            - "direction": np.array([vx, vy]) normalizado
            - "speed": float en [0,1]
            - "fwd_back": traslación en eje X
            - "left_right": traslación en eje Y
            - "up_down": traslación en eje Z
            - "roll", "pitch", "yaw": rotaciones en grados
        """
        pass
