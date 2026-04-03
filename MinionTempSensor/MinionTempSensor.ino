#define GPIO_SEL_27 134217728

#define ZB_BEGIN_TIMEOUT_DEFAULT 30000  // 30 seconds

#define ZIGBEE_DEFAULT_ED_CONFIG() \
  { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = false, \
    .nwk_cfg = { \
      .zed_cfg = { \
        .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN, \
        .keep_alive = 3000, \
      }, \
    }, \
  }

//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @brief Zigbee temperature and humidity sensor Zigbee Sleepy End Device.
 *
 * https://tutoduino.fr/tutoriels/esp32c6-zigbee/
 * This code is based on example "Zigbee temperature and humidity sensor Sleepy device" created by Jan Procházka 
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/Zigbee/examples/Zigbee_Temp_Hum_Sensor_Sleepy
 */
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
// Comment or uncomment the following line to display or not debug traces in serial monitor of Arduino IDE
//#define DEBUG_TRACE

#include "Zigbee.h"
#include <Adafruit_BMP280.h>
#include <RunningMedian.h>

#define STATUS_LED 13
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */

#ifdef DEBUG_TRACE
#define TIME_TO_SLEEP 30 // 900         /* Sleep for 15 minutes */
#else
#define TIME_TO_SLEEP 900         /* Sleep for 15 minutes */
#endif

/* Zigbee EP config */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10

/* Zigbee temperature sensor configuration */
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

/* sleep switch */
ZigbeeBinary zbBinary = ZigbeeBinary(TEMP_SENSOR_ENDPOINT_NUMBER + 1);
RTC_DATA_ATTR bool go_to_sleep = true;

/* BMP280 sensor */
Adafruit_BMP280 sensor;

// 3.7 V Li-Ion battery voltage
#define VBAT_ADC_PIN A2
#define NUM_BAT_SAMPLES 63
const float minVoltage = 3.0;
const float maxVoltage = 4.0;
RunningMedian vBatSamples = RunningMedian(NUM_BAT_SAMPLES);
ZigbeeAnalog zbAnalog = ZigbeeAnalog(TEMP_SENSOR_ENDPOINT_NUMBER + 2);

#ifdef DEBUG_TRACE
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
 
  wakeup_reason = esp_sleep_get_wakeup_cause();
 
  // Print if if it was a GPIO or something else
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
 
  // If it is due to the a GPIO pin find out which one/s
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
 
    // Print the raw value returned by esp_sleep_get_ext1_wakeup_status. This is the bitmask of the pin/s that triggered wake up
    Serial.print("Raw bitmask value returned: ");
    Serial.println(GPIO_reason);
 
    // Using log method to work out trigger pin. This is the method used by Random Nerd Tutorials
    Serial.print("GPIO that triggered the wake up calculated using log method: ");
    int wakeupPin = log(GPIO_reason)/log(2);
    Serial.println(wakeupPin);
 
    // Use defined pins bitmask to find which pin/s triggered wakeup
    Serial.print("GPIO that triggered the wake up using built in definitions: ");
    switch (GPIO_reason) 
    {
      case GPIO_SEL_27 : Serial.println("Only GPIO 27"); break; // GPIO 27
      default : Serial.println("Unknown pin"); break;
    }
  }
}
#endif

void goToSleep() {
 
  // Prepare to sleep
  // Set to wake up if any of the set pins go high
  //esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_27, ESP_EXT1_WAKEUP_ANY_HIGH);

  Serial.println("========= Going to sleep ==================================");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

// Map float values to percentage
uint8_t mapFloatToPercentage(float x, float in_min, float in_max) {
  float val;
  val = (x - in_min) * (100) / (in_max - in_min);
  if (val < 0) {
    val = 0;
  } else if (val > 100) {
    val = 100;
  }

  return (uint8_t)val;
}
// Get battery voltage en V
float getVbatt() {
  float k = 2.0592592592592592592592592592593;

  uint32_t Vbatt = 0;
  uint32_t adc_raw;

  vBatSamples.clear();

  for (int i = 0; i < NUM_BAT_SAMPLES; i++) {
    adc_raw = analogReadMilliVolts(VBAT_ADC_PIN);
    vBatSamples.add(adc_raw);
  }

  float vbatavg = vBatSamples.getMedianAverage(31);

#ifdef DEBUG_TRACE
  Serial.printf("ADC Average: %.1f\n", vbatavg);
#endif

  float vbatk = k * vbatavg;
  zbAnalog.setAnalogInput(vbatk);

  return (vbatk / 1000.0);  // Adjust for 1:2 divider and convert to volts
}
// Get data from BMP280 sensor and go to deep sleep mode
void measureAndSleep() {
  // Measure temperature sensor value
  float temperature(NAN);
  if (sensor.takeForcedMeasurement()) {
    temperature = sensor.readTemperature();
  }

  // Measure and battery voltage
  float vBat = getVbatt();
  zbTempSensor.setBatteryVoltage(vBat * 10);  // voltage in 100mV
  uint8_t percentage = mapFloatToPercentage(vBat, minVoltage, maxVoltage);
  zbTempSensor.setBatteryPercentage(percentage);

  //
  zbTempSensor.setTemperature(temperature);

#ifdef DEBUG_TRACE
  Serial.printf("Battery: %.2fV (%d%%)\n", vBat, percentage);
#endif

  // Report values
  zbTempSensor.report();

  //zbTempSensor.reportBatteryPercentage();

#ifdef DEBUG_TRACE
  Serial.printf("Reported temperature: %.2f°C\r\n", temperature);
  flashLED(1);
#endif

  // Add small delay to allow the data to be sent before going to sleep
  delay(500);

  // Put device to deep sleep
#ifdef DEBUG_TRACE
  Serial.printf("Going to sleep for %d seconds\r\n", TIME_TO_SLEEP);
#endif

#ifdef DEBUG_TRACE
  Serial.printf("Sleep switch: %s\r\n", go_to_sleep ? "sleep" : "stay awake");
#endif

  if (digitalRead(BOOT_PIN) == 1 && go_to_sleep == true) {
    esp_deep_sleep_start();
  }
}

#ifdef DEBUG_TRACE
// Internal Led flash (n times)
void flashLED(int n) {
  for (int i = 0; i < n; i++) {
    // Turn on LED for 100ms
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }
}
#endif

void onZbBinaryOutputChange(bool state) {
#ifdef DEBUG_TRACE
  Serial.printf("Binary output state: %s\r\n", state ? "true" : "false");
#endif
  go_to_sleep = state;
}

/********************* Arduino functions **************************/
void setup() {

  Wire.begin(10, 11);

  // Configure builtin LED and turn it OFF (HIGH)
  pinMode(9, INPUT);  // BOOT button
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  // Configure ADC input for reading battery voltage
  pinMode(VBAT_ADC_PIN, INPUT);

#ifdef DEBUG_TRACE
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Minion Zigbee temperature sensor start!");
  // Internal LED flash twice to indicate device start
  flashLED(2);

  print_wakeup_reason();
#endif

  while (!sensor.begin(BMP280_ADDRESS_ALT)) {
#ifdef DEBUG_TRACE
    Serial.println("Could not find BMP280 sensor!");
    flashLED(3);
    delay(1000);
#endif
  }

#ifdef DEBUG_TRACE
  Serial.printf("Found BMP280 sensor. ID = 0x%02X\r\n", sensor.sensorID());
#endif

  sensor.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Brick Wall Hardware", "Minion Bob");
  // Set minimum and maximum temperature measurement value
  zbTempSensor.setMinMaxValue(-20, 80);
  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)

  zbTempSensor.setTolerance(1);
  // Set power source to battery, battery percentage and battery voltage (now 100% and 3.5V for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()

  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, maxVoltage);

  // Add temp sensor endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  if (zbBinary.addBinaryOutput())
  {
    zbBinary.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_SECURITY_ENABLE_CONTROL);
    zbBinary.setBinaryOutputDescription("Enable deep sleep");
    zbBinary.onBinaryOutputChange(onZbBinaryOutputChange);
    Zigbee.addEndpoint(&zbBinary);
  }

  if (zbAnalog.addAnalogInput())
  {
    zbAnalog.setAnalogInputApplication(ESP_ZB_ZCL_AV_APP_TYPE_OTHER);
    zbAnalog.setAnalogInputDescription("Battery voltage");
    zbAnalog.setAnalogInputResolution(1);
    zbAnalog.setAnalogInputMinMax(0,5000);
    Zigbee.addEndpoint(&zbAnalog);
  }

  // Create a default Zigbee configuration for End Device
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();

#ifdef DEBUG_TRACE
  Serial.println("Starting Zigbee");
#endif
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    // If Zigbee does not start with 30s default timeout (ZB_BEGIN_TIMEOUT_DEFAULT) then restart
#ifdef DEBUG_TRACE
    flashLED(10);
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting ESP32!");
#endif
    ESP.restart();  // If Zigbee failed to start, reboot the device and try again
  }
#ifdef DEBUG_TRACE
  Serial.println("Connecting to network");
#endif
  while (!Zigbee.connected()) {
#ifdef DEBUG_TRACE
    Serial.print(".");
    delay(1000);
    flashLED(5);
#endif
  }

#ifdef DEBUG_TRACE
  Serial.println("Successfully connected to Zigbee network");
#endif
  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);
  // Call the function to measure temperature and put the device to deep sleep

  zbBinary.setBinaryOutput(go_to_sleep);
  //zbBinary.reportBinaryOutput();
}

void loop() {
  // No actions are performed in the loop (the ESP32C6 enters the setup function when it exits deep sleep).
  measureAndSleep();
  delay(5000);
}