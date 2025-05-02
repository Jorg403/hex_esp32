#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

// Configuración de WiFi
const char* ssid = "Hackback";
const char* password = "biczocho";

// Definir el servidor web
WebServer server(80);

// Variable para almacenar el comando
String _command;

void handleCommand(String cmd);
void update();

// Función para manejar la solicitud HTTP (cuando se reciba un comando)
void handleRoot() {
  if (server.hasArg("cmd")) {
    _command = server.arg("cmd");  // Obtener el comando desde el parámetro "cmd"
    //Serial.println("Comando recibido: " + _command);
    handleCommand(_command);  // Llamar a la función de manejo de comandos
  }
  server.send(200, "text/plain", _command);
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Pequeña pausa para estabilizar Serial
  //Serial.println("Conectando a WiFi...");

  WiFi.begin(ssid, password);

  // Esperar hasta que se conecte a la red WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }

  //Serial.println("");
  //Serial.println("WiFi conectado!");
  //Serial.print("Dirección IP: ");
  //Serial.println(WiFi.localIP());

  // Configurar servidor web para manejar solicitudes GET
  server.on("/", HTTP_GET, handleRoot);

  // Iniciar el servidor web
  server.begin();
}

void loop() {
  // Escuchar las solicitudes HTTP entrantes
  server.handleClient();

  // Aquí puedes agregar más lógica si necesitas realizar otras tareas
  update();
}

void handleCommand(String cmd) {
  // Implementa la lógica para manejar los comandos
  if (cmd == "start") {
    //Serial.println("Ejecutando comando 'start'");
    // Aquí puedes agregar el código para iniciar una acción, como mover los servos.
  } else if (cmd == "stop") {
    //Serial.println("Ejecutando comando 'stop'");
    // Código para detener la acción.
  } else {
    //Serial.println("Comando no reconocido.");
  }
}

void update() {
  delay(10);  // Pequeña pausa para no bloquear el servidor web
}
