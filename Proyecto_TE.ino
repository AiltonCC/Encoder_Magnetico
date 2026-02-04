/*
Chavez Contreras Edgar Ailton Jave
Velazquez Galvan Diego Antonio

  Teoria Electromagnetica
Gordillo Sol Alvaro

Encoder Magnetico
*/
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>          // Para utilizar el LCD de I2C

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
LiquidCrystal_I2C lcd_i2c(0x3f,16,2);    // Creamos el objeto para el LCD

float anguloAnterior = NAN;     // angulo de la muestra previa
float Posicion = 0;           
unsigned long tAnterior = 0;    // tiempo en ms de la muestra previa
float rpm = 0;       
float rpmprom = 0;
bool sensorOK = false;

//Pines para el motor
int dirPin1 = 8;
int dirPin2 = 9;
int speedPin = 10;
const int pot = A0; //Potenciometro
int button = A1;
int speed;
bool modo = false; //Modo

void setup() {
  Serial.begin(9600);
  Wire.begin();

  lcd_i2c.init();
  lcd_i2c.backlight();

  pinMode(button, INPUT_PULLUP);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(speedPin, OUTPUT);
  tAnterior = millis();
}

void loop() {
  int valor;
  valor = analogRead(pot);

  sensorOK = mag.begin();
  if(!sensorOK)   //Prueba de funcionamiento del sensor
  {
    Serial.println(F("No se encontró HMC5883L."));
    while(1);
  }

  sensors_event_t event;
  mag.getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;

  //Control del motor
  if(push(button))  //Detecta cambios en el boton
  {
    modo = !modo;
  }

  if(modo)
  {
    speed = map(valor,0,1023,0,255); //Control de velocidad con pot

    analogWrite(speedPin,speed);
    digitalWrite(dirPin1,1);
    digitalWrite(dirPin2,0);
  }
  else
  {
    analogWrite(speedPin,0);
  }

  // Posicion en grados 
  float nuevoAng = atan2((float)y, (float)x) * 180.0 / PI;   // Obtiene los grados en -180 y 180
  if (nuevoAng < 0) nuevoAng += 360.0;                       // Transforma los -180 en valores >180
  Posicion = nuevoAng;

  // Calculando el cambio de angulo con respecto al tiempo
  if (!isnan(anguloAnterior)) 
  {
    float dtheta = nuevoAng - anguloAnterior;
    
    if (dtheta >  180.0) dtheta -= 360.0; // Corrige saltos −359  0  +359 para evitar errores
    if (dtheta < -180.0) dtheta += 360.0;

    // Calculo de RPM 
    unsigned long tActual = millis();
    float dt = (float)(tActual - tAnterior);
    if (dt > 0) {
      rpm = (dtheta / 360.0) * (60000.0 / dt);
      rpmprom= (rpmprom + rpm)/2; // Se promedia el rpm para mas precision
    }
    tAnterior = tActual;
  }
  anguloAnterior = nuevoAng;   // Guarda para próxima iteración

  Serial.print(F("Ángulo: "));
  Serial.print(Posicion, 1);
  Serial.print(F("°   |   RPM: "));
  Serial.println(abs(rpmprom), 2);
  Serial.print(F("X: "));
  Serial.println(x, 1);
  Serial.print(F("Y: "));
  Serial.println(y, 2);

  lcd_i2c.setCursor(1,0);
  lcd_i2c.print("Angulo: " + String(Posicion));
  lcd_i2c.setCursor(1,1);
  lcd_i2c.print("RMP: " + String(rpmprom));

  delay(100);   // Frecuencia de medicion
}

//Función anti-rebote
bool push(byte input){
  bool state = false;
  if(! digitalRead(input)){
    delay(200);
    while(! digitalRead(input));
    delay(200);
    state = true;
  }      
  return state;   
}
