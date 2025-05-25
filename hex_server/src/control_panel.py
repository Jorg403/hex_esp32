import argparse
import requests
import cv2
from comm.comm_constructor import create_comm

def main():
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

    print(f"Conectado")

    velocidad = 30
    servo0_enabled = True
    servo1_enabled = True

    # Estados de cada tecla toggle
    state_w = False
    state_s = False
    state_a = False
    state_d = False

    cv2.namedWindow("Control")
    cv2.displayOverlay("Control", "Presiona H para ver comandos", 3000)

    while True:
        key = cv2.waitKey(50) & 0xFF
        key_char = chr(key).lower() if key != 255 else None

        if key_char == 'w':
            state_w = not state_w
            state_s = False
            if state_w:
                comm.send_command(f"set_speed 1 {velocidad}")
            else:
                comm.send_command("set_speed 1 0")

        elif key_char == 's':
            state_s = not state_s
            state_w = False
            if state_s:
                comm.send_command(f"set_speed 1 {-velocidad}")
            else:
                comm.send_command("set_speed 1 0")

        elif key_char == 'a':
            state_a = not state_a
            state_d = False
            if state_a:
                comm.send_command(f"set_speed 0 {-velocidad}")
            else:
                comm.send_command("set_speed 0 0")

        elif key_char == 'd':
            state_d = not state_d
            state_a = False
            if state_d:
                comm.send_command(f"set_speed 0 {velocidad}")
            else:
                comm.send_command("set_speed 0 0")

        elif key_char == 'r':
            velocidad += 1
            print(f"Velocidad aumentada: {velocidad}")

        elif key_char == 'f':
            velocidad -= 1
            print(f"Velocidad disminuida: {velocidad}")

        elif key_char == 'z':
            if servo0_enabled:
                comm.send_command("set_manualPWM 0 0")
            else:
                comm.send_command("enable 0")
            servo0_enabled = not servo0_enabled

        elif key_char == 'x':
            if servo1_enabled:
                comm.send_command("set_manualPWM 1 0")
            else:
                comm.send_command("enable 1")
            servo1_enabled = not servo1_enabled

        elif key_char == 'q':
            comm.send_command("print_status")

        elif key_char == 'p':
            n1 = input("Servo: ")
            n2 = input("PWM: ")
            comm.send_command(f"set_manualPWM {n1} {n2}")

        elif key_char == 'm':
            n1 = input("Servo: ")
            n2 = input("Position: ")
            comm.send_command(f"set_position {n1} {n2}")

        elif key_char == 'h':
            print("Comandos disponibles:")
            print("w: Avanzar (toggle)")
            print("s: Retroceder (toggle)")
            print("a: Girar izquierda (toggle)")
            print("d: Girar derecha (toggle)")
            print("r: Aumentar velocidad")
            print("f: Disminuir velocidad")
            print("z: Activar/desactivar servo 0")
            print("x: Activar/desactivar servo 1")
            print("q: Imprimir estado actual")
            print("p: Establecer PWM manualmente")
            print("m: Establecer posición del servo")
            print("h: Mostrar ayuda")

        elif key_char == '\x1b':  # Escape
            print("Saliendo...")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
