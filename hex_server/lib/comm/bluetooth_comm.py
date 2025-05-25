from lib.comm.comm import Comm
import serial
import time

class BluetoothComm(Comm):
    def __init__(self, port: str, baudrate: int = 9600):
        self.bt_serial = serial.Serial(port, baudrate, timeout=1)
        print(f"Conectado a Bluetooth en {port}")

    def send_command(self, command: str):
        try:
            self.bt_serial.write((command + '\n').encode('utf-8'))
            # print(f"→ {command}")

            # time.sleep(0.1)
            # while self.bt_serial.in_waiting:
            #     response = self.bt_serial.readline().decode('utf-8').strip()
            #     print(f"Respuesta: {response}")

        except serial.SerialException as e:
            print(f"Excepción al enviar comando Bluetooth: {e}")
