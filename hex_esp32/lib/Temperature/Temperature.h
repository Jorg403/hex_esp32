#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "Arduino.h"

class Temperature {
  public:
    Temperature();              // Constructor
    void start_temperature();   // Iniciar lectura
    float get_temperature();    // Obtener temperatura
    void stop_temperature();    // Detener lecturas (opcional)

  private:
    bool started;               // Estado de la lectura
    float last_temperature;     // Última lectura de temperatura
};

#endif
