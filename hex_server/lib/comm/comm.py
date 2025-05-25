from abc import ABC, abstractmethod

class Comm(ABC):
    @abstractmethod
    def send_command(self, command: str):
        pass
