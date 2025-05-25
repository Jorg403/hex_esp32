import argparse
import requests
import time
import numpy as np
from comm.comm_constructor import create_comm


def map_value(value, in_min, in_max, out_min, out_max):
    """Mapea un valor de un rango a otro."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def func_1(i):
    # Sine of i (degrees)
    return np.sin(i * 3.14 / 180) * 90 + 90

def main():
    parser = argparse.ArgumentParser(description="Control en bucle")
    parser.add_argument('--IP', required=False, help="La dirección IP del ESP32 en la red local.")
    parser.add_argument('--COM', required=False, help="Puerto COM para la comunicación serial.")
    
    args = parser.parse_args()

    if args.IP is not None:
        # Si se proporciona una IP, se utiliza la comunicación WiFi
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM is not None:
        # Si se proporciona un puerto COM, se utiliza la comunicación Bluetooth
        comm = create_comm('bluetooth', port=args.COM)
    else:
        print("Error: Debe proporcionar una dirección IP o un puerto COM.")
        return

    print(f"Conectado")
        
    angle_x = 90
    angle_y = 90
    delta = 0.

    i = 0
    # we select func1 as next_angle_x function
    next_angle_x = func_1
    next_angle_y = func_1
    while True:
        time.sleep(delta)
        angle_x = next_angle_x(i)
        comm.send_command(f"set_position 0 {angle_x}")
        time.sleep(delta)
        angle_y = next_angle_y(i)
        comm.send_command(f"set_position 1 {angle_y}")
        i += 1
        if i > 360:
            i = 0

if __name__ == "__main__":
    main()
