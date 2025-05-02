import argparse
import requests

# Función para enviar un comando al ESP32
def send_command(ip, command):
    url = f'http://{ip}/?cmd={command}'
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print(f"\n{response.text}")
        else:
            print(f"Error al enviar el comando. Estado: {response.status_code}")
    except requests.exceptions.RequestException as e:
        a = 1
        # print(f"Error en la conexión con {ip}: {e}")

def main():
    # Configuración de los argumentos del script
    parser = argparse.ArgumentParser(description="Envía comandos al ESP32 a través de WiFi.")
    parser.add_argument('--IP', required=True, help="La dirección IP del ESP32 en la red local.")
    
    args = parser.parse_args()

    print(f"Conectado a {args.IP}\n")
    command = ""
    while command != "exit":
        # Solicitar al usuario que ingrese el comando
        command = input()
        
        # Enviar el comando
        send_command(args.IP, command)

if __name__ == "__main__":
    main()
