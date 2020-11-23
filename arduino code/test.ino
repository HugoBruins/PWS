#include <AccelStepper.h>

#define dirPin 5
#define stepPin 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  int snelheid = 500; 
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(5);
}

void loop() { 
    stepper.setSpeed(snelheid);
    stepper.runSpeed();     
}
