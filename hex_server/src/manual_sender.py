import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import argparse
import requests
from lib.comm.comm_constructor import create_comm
from lib.robotics.inverse_kinematics import calculate_joint_angles

def main():
    # Configuración de los argumentos del script
    parser = argparse.ArgumentParser(description="Envía comandos al ESP32 a través de WiFi.")
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

    print(f"Conectado\n")
    command = ""
    while command != "exit":
        # Solicitar al usuario que ingrese el comando
        command = input()

        # if command is "sp n n" command = "set_position n n"
        if command.startswith("ang "):
            parts = command.split()
            command = f"set_position {parts[1]} {parts[2]}"
        elif command.startswith("pwm "):
            parts = command.split()
            command = f"set_manualPWM {parts[1]} {parts[2]}"
        elif command.startswith("pos "):
            parts = command.split()
            # import pdb
            # pdb.set_trace()
            (angle0, angle1, angle2) = calculate_joint_angles(np.array((float(parts[1]), float(parts[2]), float(parts[3]))))
            comm.send_command(f"set_position 0 {angle0}")
            comm.send_command(f"set_position 1 {angle1}")
            comm.send_command(f"set_position 2 {angle2}")
            continue
        
        # Enviar el comando
        comm.send_command(command)

if __name__ == "__main__":
    main()
