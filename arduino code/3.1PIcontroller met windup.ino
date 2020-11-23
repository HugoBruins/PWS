#include <AccelStepper.h>
#include <Wire.h>
#include <MPU6050.h>

//stappenmotor
#define dirPin 3
#define stepPin 2
#define dirPin2 5
#define stepPin2 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

int maxSnelheid = 1000;
int snelheid;

//voor de MPU 6050
MPU6050 mpu;

//voor de timer
long loop_timer;
float tijdstap = 0.5;
unsigned long vorigetijd;

//voor de PID controller
float setpoint = 0;
float Kp = 12.5 / tijdstap;
float Ki = 0.25 / tijdstap;
float Imax = 1000;
float I = 0;

void setup() {
  //stappenmotor 
  stepper.setMaxSpeed(maxSnelheid);
  stepper2.setMaxSpeed(maxSnelheid);

  //mpu6050
  Serial.begin(115200);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Mpu werkt niet :(");
    delay(500);
  }
  loop_timer = micros();
}


void loop() { 
  Vector normAccel = mpu.readNormalizeAccel();
  //bereken hoek
  float hoek = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  
  //voor de P
  float error = setpoint - hoek;
  float P = error * Kp;
  
  //voor de I controller
  I += Ki*error*tijdstap;
  
  //windup bescherming
  if (I > Imax) {
   I = Imax;
  }
  if (I < (-1 * Imax)) {
   I = -1 * Imax; 
  }
  
  //alles bij elkaar
  float PID = P;
  
  //doet I waarde erbij als I niet gelijk is aan Imax
  if (I != Imax || I != (-1*Imax)) {
    PID += I;
  }
  
  //zorgt ervoor dat de PID output nooit groter wordt dan de maximale RPM die we hebben ingesteld
  if (PID > maxSnelheid) {
    PID = maxSnelheid;
  }
  
  if (PID < -1 * maxSnelheid) {
    PID = -1 * maxSnelheid;
  }
  
  float PID2 = PID * -1;
  stepper.setSpeed(PID2);
  stepper.runSpeed();     
  stepper2.setSpeed(PID);
  stepper2.runSpeed(); 

  //timertje hieronder zodat alle code elke tijdstap wordt uitgevoerd.
  
  while(micros() - loop_timer < tijdstap*1000);                               
  loop_timer = micros();//Reset the loop timer
}
