//voor de stappenmotor

#include <AccelStepper.h>
#include <Wire.h>

#define dirPin 5
#define stepPin 4
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

int snelheid;
int tijdstap = 50;
unsigned long vorigetijd;

//voor de MPU 6050

#include <Wire.h>
//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;



void setup() {
  //stappenmotor 
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(5);
  //mpu6050
  
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;                                                 
  Serial.begin(115200);
  loop_timer = micros();                                               //Reset the loop timer
  
  
}


  
  //


void loop() { 

  unsigned long tijd = millis();  
  
  if((tijd - vorigetijd) >= tijdstap){

  }
    stepper.setSpeed(snelheid);
    stepper.runSpeed();     
}
