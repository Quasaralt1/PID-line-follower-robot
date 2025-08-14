#include <BluetoothSerial.h>

String esp = "ESP_TEST";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
 
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial ESP32;

void setup() {
  Serial.begin(115200);
  ESP32.begin(esp);
}

void loop() {
    char DADOS;
DADOS = ESP32.read();
if(DADOS)
  }
