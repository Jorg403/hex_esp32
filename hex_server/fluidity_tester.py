import argparse
import requests
import time
import numpy as np

def send_command(ip, command):
    url = f'http://{ip}/?cmd={command}'
    print(f"Enviando comando: {command}")
    try:
        response = requests.get(url)
        if response.status_code == 200:
            # print(f"→ {command}")
            if response.text.strip():
                print(f"\n{response.text}")
        else:
            print(f"Error al enviar el comando. Estado: {response.status_code}")
    except requests.exceptions.RequestException:
        pass
    print(f"Comando enviado: {command}")

def map_value(value, in_min, in_max, out_min, out_max):
    """Mapea un valor de un rango a otro."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def func_1(i):
    # Sine of i (degrees)
    return np.sin(i * 3.14 / 180) * 90 + 90

def main():
    parser = argparse.ArgumentParser(description="Control en bucle")
    parser.add_argument('--IP', required=True, help="La dirección IP del ESP32 en la red local.")
    args = parser.parse_args()

    ip = args.IP

    print(f"Conectado a {ip}")
        
    angle_x = 90
    angle_y = 90
    delta = 0.05

    i = 0
    # we select func1 as next_angle_x function
    next_angle_x = func_1
    next_angle_y = func_1
    while True:
        time.sleep(delta)
        angle_x = next_angle_x(i)
        send_command(ip, f"set_position 0 {angle_x}")
        time.sleep(delta)
        angle_y = next_angle_y(i)
        send_command(ip, f"set_position 1 {angle_y}")
        i += 1
        if i > 360:
            i = 0

if __name__ == "__main__":
    main()
