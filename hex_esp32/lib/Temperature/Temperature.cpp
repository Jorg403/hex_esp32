#include "temperature.h"
#include "driver/adc.h"  // For ADC functions

// Constructor
Temperature::Temperature() {
  started = false;
  last_temperature = 0.0;
}

// Start temperature reading (this is a no-op, just flagging that it's started)
void Temperature::start_temperature() {
  started = true;  // Mark as started
  adc1_config_width(ADC_WIDTH_BIT_12);  // Configure ADC width (12 bits resolution)
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);  // ADC1 channel 0 (GPIO34)
}

// Get the temperature in Celsius
float Temperature::get_temperature() {
  if (!started) {
    return -999.0;  // Return error if not started
  }

  // Read the raw temperature value (in ADC units)
  int raw = adc1_get_raw(ADC1_CHANNEL_0);  // Read from the internal temperature sensor channel

  // Formula to convert ADC value to temperature (linear approximation)
  // This will give a rough estimate of the temperature in Celsius
  float temperature = (raw - 32.0) / 1.8;  // Convert from Fahrenheit to Celsius
  float l_temperature = last_temperature;
  last_temperature = temperature;
  return (temperature + l_temperature) / 2.0;  // Return the average of the last two readings
}

// Stop temperature reading (no real action needed in this case)
void Temperature::stop_temperature() {
  started = false;
}
