#include <BluetoothSerial.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
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
int erro, erroAnterior;
int P, I, D;

//variaveis usadas na multiplicacao dos valores
float Pvalue, Ivalue, Dvalue;

//variaveis de velocidade base,minima e maxima
int maxspeed = 125;
int minspeed = -125;

//variavel de velocidade esquerda e direita
int leftspeed, rightspeed;

int minbasespeed = 60;  // velocidade base mínima
int maxbasespeed = 75;  // velocidade base máxima

#define Bcalibrate 34
#define Bstart 35

bool Ccalibrate = false;
bool activate_PID = false;
bool calibrate = false;
//***************************************************************************************************************************************************
void BTread() {
  if (ESP32.available()) {
    String data = ESP32.readStringUntil('\n');
    data.trim();

    //PID valores
    if (data.startsWith("KP:")) kp = data.substring(3).toInt();
    else if (data.startsWith("KI:")) ki = data.substring(3).toInt();
    else if (data.startsWith("KD:")) kd = data.substring(3).toInt();

    // valores de multiplicacao do PID
    else if (data.startsWith("MP:")) multiP = data.substring(3).toInt();
    else if (data.startsWith("MI:")) multiI = data.substring(3).toInt();
    else if (data.startsWith("MD:")) multiD = data.substring(3).toInt();

    //velocidade
    else if (data.startsWith("Min:")) minspeed = data.substring(4).toInt();
    else if (data.startsWith("Max:")) maxspeed = data.substring(4).toInt();

    //velocidade base

    else if (data.startsWith("MinB:")) minbasespeed = data.substring(5).toInt();
    else if (data.startsWith("MaxB:")) maxbasespeed = data.substring(5).toInt();


    else if (data.startsWith("Ca:")) calibrate = !calibrate;
    else if (data.startsWith("Sa:")) activate_PID = data.substring(3).toInt();
    else if (data.startsWith("So:")) activate_PID = data.substring(3).toInt();
  }
}
//***************************************************************************************************************************************************
void calibration() {
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  calibrate = false;
  Ccalibrate = true;
}
//***************************************************************************************************************************************************
void leftmotor(int speed) {
  //verificacao de sentido do motor com base na velocidade
  speed < 0 ? analogWrite(motorE1, -speed), analogWrite(MotorE2, 0 ) : analogWrite(motorE1, 0), analogWrite(motorE2, speed);
}
void rightmotor(int speed) {
    speed < 0 ? analogWrite(motorD1, -speed), analogWrite(motorD2, 0) : analogWrite(motorD1, 0), analogWrite(motorD2, speed);
  }
}
//***************************************************************************************************************************************************
void stop() {

  //motor esquerdo
  analogWrite(motorE1, 0);
  analogWrite(motorE2, 0);

  //motor direito
  analogWrite(motorD1, 0);
  analogWrite(motorD2, 0);
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
  float maxerror = 3500.0;   // erro máximo esperado
  float speedscaling = 1.0 - (errorMagnitude / maxerror);  // quanto maior o erro, menor a velocidade
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
void BTmonitor() {
  Serial.print("KP: ");
  Serial.print(kp);
  Serial.print(" KI:");
  Serial.print(ki);
  Serial.print(" KD: ");
  Serial.print(kd);
  Serial.print(" Multi P: ");
  Serial.print(multiP);
  Serial.print(" Multi D: ");
  Serial.print(multiI);
  Serial.print(" Multi D: ");
  Serial.print(multiD);
  Serial.print(" min speed: ");
  Serial.print(minspeed);
  Serial.print(" max speed: ");
  Serial.print(maxspeed);
  Serial.print(" min base speed: ");
  Serial.print(minbasespeed);
  Serial.print(" maxbasespeed: ");
  Serial.print(maxbasespeed);
  Serial.println();
}

//***************************************************************************************************************************************************
void setup() {
  pinMode(Bcalibrate, INPUT);
  pinMode(Bstart, INPUT);
  
  //definindo pino do motor
  pinMode(motorE1, OUTPUT);
  pinMode(motorE2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 4, 5, 15, 18, 19, 27, 32, 33 }, sensores);
  Serial.begin(115200);
  if (!ESP32.begin("ESP_TEST")) {
    Serial.println("Erro ao iniciar Bluetooth!");
  } else {
    Serial.println("Bluetooth iniciado com nome: ESP_TEST");
  }
  while (calibrate == false) {
    if (digitalRead(Bcalibrate) == HIGH) {
      calibrate = !calibrate;
    }
    stop();
    delay(50);
  }
  while (Ccalibrate == true && activate_PID == false) {
    if (digitalRead(Bstart) == HIGH) {
      activate_PID = !activate_PID;
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
  } else if (activate_PID == true && calibrate == false && Ccalibrate == true) {
    PID();
  } else {
    stop();
  }
}