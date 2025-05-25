import threading
import time
import queue
import argparse
import sys
import os

# Adjust sys.path for parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lib.comm.comm_constructor import create_comm
from lib.robotics.inverse_kinematics import calculate_joint_angles

# --- Configuración global ---
NUM_INTERMEDIATE_POINTS = 50
SPEED = 0.08  # segundos entre envíos

# Lista de puntos (cada punto es (x, y, z))
# points = [(12.0, 0.0, -3.0), (12.0, 0.0, 3.0), (21.0, 0.0, 0.0), (10.0, -5.0, -3.0), (10.0, -10.0, 7.0), (1.0, -13.0, -5.0), (10.0, -8.0, 5.0), (12.0, 0.0, -3.0)]
# points = [(12.0,-12.0,6.0),(0.0,-12.0,6.0),(0.0,-12.0,-4.0),(12.0,-12.0,-4.0),(12.0,12.0,-4.0),(12.0,12.0,6.0),(12.0,-12.0,6.0)] # cube
# points = [(12.0, -10.0, -4.0), (12.0, 10.0, -4.0), (12.0, -10.0, -4.0)] # horizontal
# points = [(11.0, 0.0, 2.0), (18.5, 0.0, -8.5), (11.0, 0.0, 2.0)] #oil
# points = [(15.0, 0.0, 5), (15.0, 0.0, -5), (15.0, 0.0, 5)] # vertical
# now with 12 14 -7, 12 0 -7, 12 -14 -7, 12 -14 -4, 12 14 -4, 12 14 -7

x_all = 12.0
y_lims = 10.0
z_down = -10.0
z_up = -8.0
points = [(x_all, -y_lims, z_down), (x_all, y_lims, z_down), (x_all, y_lims, z_up), (x_all, -y_lims, z_up), (x_all, -y_lims, z_down)] # walk


# x_all = 0.0
# y_far = 15.0
# y_near = 8.0
# z_down = -12.0
# z_up = -4.0
# points = [(x_all, -y_far, z_up), (x_all, -y_far, z_down), (x_all, -y_near, z_down), (x_all, -y_near, z_up), ((y_far+y_near)/2, 0, z_up), (x_all, y_far, z_up), (x_all, y_far, z_down), (x_all, y_near, z_down), (x_all, y_near, z_up),  ((y_far+y_near)/2, 0, z_up), (x_all, -y_far, z_up)] # side to side
walk = True
paused = False

def interpolate_points(p1, p2, num_points):
    result = []
    for i in range(1, num_points + 1):
        x = p1[0] + (p2[0] - p1[0]) * i / (num_points + 1)
        y = p1[1] + (p2[1] - p1[1]) * i / (num_points + 1)
        z = p1[2] + (p2[2] - p1[2]) * i / (num_points + 1)
        result.append((x, y, z))
    return result

def sender_task(comm, command_queue):
    global paused, SPEED
    while True:
        if not paused:
            full_path = []
            for i in range(len(points) - 1):
                full_path.append(points[i])
                
                ip = NUM_INTERMEDIATE_POINTS

                if walk and (points[i][2] != points[i+1][2] or points[i][0] != z_down):
                    ip = NUM_INTERMEDIATE_POINTS // 3

                full_path.extend(interpolate_points(points[i], points[i+1], ip))
            full_path.append(points[-1])

            for p in full_path:
                if paused:
                    break
                angle0, angle1, angle2 = calculate_joint_angles(p[0], p[1], p[2])
                comm.send_command(f"set_position 0 {angle0}")
                comm.send_command(f"set_position 1 {angle1}")
                comm.send_command(f"set_position 2 {angle2}")
                time.sleep(SPEED)

        time.sleep(0.1)

def user_input_task(command_queue):
    global paused, SPEED, points
    while True:
        command = input(">> ")
        parts = command.strip().split()

        if parts[0] == "add" and len(parts) == 4:
            points.append((float(parts[1]), float(parts[2]), float(parts[3])))
            print("Punto agregado.")
        elif parts[0] == "remove" and len(parts) == 2:
            idx = int(parts[1])
            if 0 <= idx < len(points):
                points.pop(idx)
                print("Punto eliminado.")
            else:
                print("Índice inválido.")
        elif parts[0] == "pause":
            paused = True
            print("Pausado.")
        elif parts[0] == "resume":
            paused = False
            print("Reanudado.")
        elif parts[0] == "speed" and len(parts) == 2:
            SPEED = max(0.001, float(parts[1]))
            print(f"Velocidad ajustada a {SPEED} s.")
        elif parts[0] == "list":
            for idx, p in enumerate(points):
                print(f"{idx}: {p}")
        elif parts[0] == "exit":
            print("Saliendo...")
            os._exit(0)
        else:
            print("Comandos: add x y z | remove idx | pause | resume | speed s | list | exit")

def main():
    parser = argparse.ArgumentParser(description="Envía puntos al robot con control de usuario.")
    parser.add_argument('--IP', required=False, help="La dirección IP del ESP32.")
    parser.add_argument('--COM', required=False, help="Puerto COM.")
    args = parser.parse_args()

    if args.IP:
        comm = create_comm('wifi', ip=args.IP)
    elif args.COM:
        comm = create_comm('bluetooth', port=args.COM)
    else:
        print("Error: Debe proporcionar --IP o --COM")
        return

    print("Conectado.\n")
    command_queue = queue.Queue()

    threading.Thread(target=sender_task, args=(comm, command_queue), daemon=True).start()
    user_input_task(command_queue)

if __name__ == "__main__":
    main()
