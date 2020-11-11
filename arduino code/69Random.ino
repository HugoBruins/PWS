//voor de stappenmotor

#include <AccelStepper.h>

#define dirPin 3
#define stepPin 2
#define dirPin2 5
#define stepPin2 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
float tijdstap = 1;
int maxSnelheid = 1000;
long loop_timer;
float snelheid = 500;

void setup() {
  //stappenmotor 
  stepper.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
}


void loop() { 
  for(int snelheid = 0; snelheid < 1000; snelheid++) {
    stepper.setSpeed(snelheid);
    stepper2.setSpeed(snelheid);
    stepper.runSpeed();
    stepper2.runSpeed();
    
    unsigned long tijd = millis();  
    while(tijd - loop_timer < tijdstap) {                        
      loop_timer = millis();
  }
  }
  for (int snelheid = 1000; snelheid > 0; snelheid++) {
    stepper.setSpeed(snelheid);
    stepper2.setSpeed(snelheid);
    stepper.runSpeed();
    stepper2.runSpeed();
    
    unsigned long tijd = millis();  
    while(tijd - loop_timer < tijdstap) {                        
      loop_timer = millis();
  }
  }  
}
