import socket

HOST = '172.20.104.190'       # Escucha en todas las interfaces
PORT = 8080     # El mismo puerto que usa ESP32 en client.connect()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Esperando conexión en el puerto {PORT}...")

    conn, addr = server_socket.accept()
    with conn:
        print(f"Conexión establecida desde {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                print("Conexión cerrada por ESP32")
                break
            print(f"Datos recibidos: {data.decode()}")
