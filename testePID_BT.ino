#include <BluetoothSerial.h>

String esp = "ESP_TEST";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
 
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial ESP32;

#include <QTRSensors.h>
QTRSensors qtr;

//variaveis de quantidade e de leitura dos sensores
const uint8_t sensores = 8;
uint16_t leituras[sensores];

//***************************************************************************************************************************************************
#define resolution 8
#define frequency 5000
//variavel de motor
#define motorE1 17
#define motorE2 23
#define motorD1 13
#define motorD2 16

//variaveis do PID
//ganhos proporcionais,integrais e derivados
float kp = 0.08;
float ki = 0;
float kd = 0.08;



//valores de multiplicacao do pid
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;

//variaveis de calculo de erro e PID
int erro;
int erroAnterior;
int P;
int I;
int D;

//variaveis usadas na multiplicacao dos valores
float Pvalue;
float Ivalue;
float Dvalue;

//variaveis de velocidade base,minima e maxima
int maxspeed = 125;
int minspeed = -125;

int leftspeed;
int rightspeed;

int minbasespeed = 60;  // velocidade base mínima
int maxbasespeed = 75;  // velocidade base máxima

#define Bcalibrate 34
#define Bstart 35

bool Ccalibrate = false;
bool activate_PID = false;
bool calibrate = false;
//***************************************************************************************************************************************************
void BTread(){
    if(ESP32.available(){
        string data = SerialBT.read(StringUntil("\n"));
        data.trim();

        //PID valores 
    if (data.startWith("KP:")) kp = data.substring(3).toInt();
    else if (data.startwith("KI:")) ki = data.substring(3).toInt();
    else if (data.startwith("KD:")) ki = data.substring(3).toInt();

    // valores de multiplicacao do PID
    else if (data.startwith("MP:")) multiP = data.substring(3).toInt();
    else if (data.startwith("MI:")) multiI = data.substring(3).toInt();
    else if (data.startwith("MD:")) multiD = data.substring(3).toInt();

    //velocidade
    else if (data.startwith("Min:")) minspeed = data.substring(4).toInt();
    else if (data.startwith("Max:")) maxspeed = data.substring(4).toInt();

    //velocidade base

    else if (data.startwith("MinB:")) minbasespeed = data.substring(5).toInt();
    else if (data.startWith("MaxB:")) maxbasespeed = data.substring(5).toInt();

    })
}
//***************************************************************************************************************************************************
void calibration() {
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  calibrate = false;
  Ccalibrate = true;
}
void leftmotor(int speed) {
  if (speed < 0) {
    ledcWrite(motorE1, -speed);
    ledcWrite(motorE2, 0);
  } else {
    ledcWrite(motorE1, 0);
    ledcWrite(motorE2, speed);
  }
}
void rightmotor(int speed) {
  if (speed < 0) {
    ledcWrite(motorD1, -speed);
    ledcWrite(motorD2, 0);
  } else {
    ledcWrite(motorD1, 0);
    ledcWrite(motorD2, speed);
  }
}
//***************************************************************************************************************************************************
void stop() {
  ledcWrite(motorE1, 0);
  ledcWrite(motorE2, 0);
  ledcWrite(motorD1, 0);
  ledcWrite(motorD2, 0);
}
//***************************************************************************************************************************************************
void PID() {

  //calculo da posicao do robo com isso definindo o erro
  uint16_t position = qtr.readLineBlack(leituras);
  erro = 3500 - position;

  //calculo do PID
  P = erro;
  I += erro;
  D = erro - erroAnterior;

  //erro vira o erro anterior
  erroAnterior = erro;

  //calculo do PID
  Pvalue = (kp / pow(10, multiP)) * P;
  Ivalue = (ki / pow(10, multiI)) * I;
  Dvalue = (kd / pow(10, multiD)) * D;

  //calculo da velocidade dos motor
  float correction = Pvalue + Ivalue + Dvalue;
  //separando a velocidade esquerda e direita
  // Cálculo da velocidade base variável


  // Ajuste dinâmico da velocidade base com base no erro
  float errorMagnitude = abs(erro);
  float maxerror = 3500.0;                                 // erro máximo esperado (valor da borda)
  float speedscaling = 1.0 - (errorMagnitude / maxerror);  // quanto mais erro, menor velocidade
  int basespeed = minbasespeed + speedscaling * (maxbasespeed - minbasespeed);
  leftspeed = basespeed + correction;
  rightspeed = basespeed - correction;

  // Limitando as velocidades
  //limitando o minimo e maximo da velocidade
  leftspeed = constrain(leftspeed, minspeed, maxspeed);
  rightspeed = constrain(rightspeed, minspeed, maxspeed);

  leftmotor(leftspeed);
  rightmotor(rightspeed);
}

//***************************************************************************************************************************************************
void setup() {
  pinMode(Bcalibrate, INPUT);
  pinMode(Bstart, INPUT);
  //definindo pino do motor
  ledcAttach(motorE1, frequency, resolution);
  ledcAttach(motorE2, frequency, resolution);
  ledcAttach(motorD1, frequency, resolution);
  ledcAttach(motorD2, frequency, resolution);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 4, 5, 15, 18, 19, 27, 32, 33 }, sensores);
  Serial.begin(115200);
    ESP32.begin(esp);
  while (calibration == false) {
    if (digitalRead(Bcalibrate) == HIGH) {
      calibrate = true;
    }
    stop();
    delay(50);
  }
  while (Ccalibrate == true && active_PID == false) {
    if (digitalRead(Bstart) == HIGH) {
      activate_PID = true;
    }
    stop();
    delay(50);  //stop the motors
  }
}

//***************************************************************************************************************************************************
void loop() {
    BTread();
  if (calibrate == true) {
    calibration();
  } else if (activate_PID == true) {
    PID();
  } else {
    stop();
  }
}