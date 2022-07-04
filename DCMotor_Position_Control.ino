//Ref: How to control a DC Motor with an encoder, Curio Res

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 16);

#define ENCA 2
#define ENCB 3
#define PWMPin 5
#define IN1Pin 7
#define IN2Pin 8
#define potPin A0
int pos = 0;
long prevT = 0;
long currT;
float deltaT;
float ePrev = 0;
float eIntegral = 0;
int e;
float eDifferential;
float kp = 1;
float kd = 0;
float ki = 0;
int target = 1000;
float u;
float pwr;
int dir;

void readEncoder() {
  (digitalRead(ENCB)>0) ? pos++ : pos--;  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  if(dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, pwmVal);
}

void pid() {
  currT = micros();
  deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;
  e = pos - target;
  eDifferential = (e-ePrev)/deltaT;
  eIntegral = eIntegral + e*deltaT;
  u = kp*e + kd*eDifferential + ki*eIntegral;
  ePrev = e;
}

void setup() {
  Serial.begin(115200);
  //alter pin5 PWM to 62.5kHz
  TCCR0B = 0b00000001; // x1
  TCCR0A = 0b00000011; // fast pwm
  lcd.init();
  lcd.clear();
  lcd.backlight();
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  target = analogRead(potPin);
  pid();
  pwr = fabs(u);
  if(pwr >255) { pwr = 255; }
  (u>0) ? dir=1 : dir = -1; 
  setMotor(dir, pwr, PWMPin, IN1Pin, IN2Pin);
  lcd.clear();
  lcd.setCursor(8,0);
  lcd.print(pos);
}
