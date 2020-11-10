#include <MPU6050.h>
#include <AccelStepper.h>
#include <Wire.h>

#define dirPin 5
#define stepPin 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(5);
}

  int snelheid;
  int tijdstap = 50;
  unsigned long vorigetijd;

void loop() { 

  unsigned long tijd = millis();  
  
  if((tijd - vorigetijd) >= tijdstap){

  }
    stepper.setSpeed(snelheid);
    stepper.runSpeed();     
}
