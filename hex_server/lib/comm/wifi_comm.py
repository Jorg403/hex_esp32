from lib.comm.comm import Comm
import requests

class WifiComm(Comm):
    def __init__(self, ip: str):
        self.ip = ip

    def send_command(self, command: str):
        url = f'http://{self.ip}/?cmd={command}'
        try:
            response = requests.get(url)
            if response.status_code == 200:
                if response.text.strip():
                    print(f"\n{response.text}")
            else:
                print(f"Error al enviar comando (HTTP {response.status_code})")
        except requests.exceptions.RequestException as e:
            print(f"Excepción al enviar comando WiFi: {e}")
