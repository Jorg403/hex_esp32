import argparse
import requests
import cv2
import time
import threading

def send_command(ip, command):
    url = f'http://{ip}/?cmd={command}'
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

def map_value(value, in_min, in_max, out_min, out_max):
    """Mapea un valor de un rango a otro."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def main():
    parser = argparse.ArgumentParser(description="Control de servos con ratón (posición).")
    parser.add_argument('--IP', required=True, help="La dirección IP del ESP32 en la red local.")
    parser.add_argument('--size', type=int, default=500, help="Tamaño de la ventana cuadrada (px).")
    args = parser.parse_args()

    ip = args.IP
    window_size = args.size

    print(f"Conectado a {ip}")
    print(f"Ventana de control: {window_size}x{window_size}")

    # Variables compartidas
    mouse_x, mouse_y = window_size // 2, window_size // 2
    inside_window = False
    lock = threading.Lock()

    # Función de callback para ratón
    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_x, mouse_y, inside_window
        if event == cv2.EVENT_MOUSEMOVE:
            with lock:
                mouse_x, mouse_y = x, y
                inside_window = (0 <= x < window_size) and (0 <= y < window_size)

    cv2.namedWindow("Control de Servos")
    cv2.setMouseCallback("Control de Servos", mouse_callback)

    # Hilo de envío periódico
    def send_loop():
        delta = 0.01
        while True:
            time.sleep(delta)
            with lock:
                if inside_window:
                    angle_x = map_value(mouse_x, 0, window_size, 0, 180)
                    angle_y = map_value(mouse_y, 0, window_size, 0, 180)
                    send_command(ip, f"set_position 0 {angle_x}")
                    time.sleep(delta)
                    send_command(ip, f"set_position 1 {180-angle_y}")

    sender_thread = threading.Thread(target=send_loop, daemon=True)
    sender_thread.start()

    
    frame = cv2.cvtColor(cv2.UMat(window_size, window_size, cv2.CV_8UC3), cv2.COLOR_BGR2RGB).get()
    cv2.imshow("Control de Servos", frame)
    while True:

        key = cv2.waitKey(1000) & 0xFF
        if key == 27:  # Escape
            print("Saliendo...")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
