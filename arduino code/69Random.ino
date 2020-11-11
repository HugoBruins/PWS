//voor de stappenmotor

#include <AccelStepper.h>

#define dirPin 3
#define stepPin 2
#define dirPin2 5
#define stepPin2 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

void setup() {
  //stappenmotor 
  stepper.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper.setSpeed(1000);
  stepper2.setSpeed(1000);


void loop() { 
  stepper.runSpeed();
  stepper2.runSpeed();
