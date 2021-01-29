//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Code voor Balanceerrobot (aka BalanceerMeneer) 
//Door Hugo Bruins en Thomas Ritmeester

//De gebruikte code voor de MPU6050 is gemaakt door Joop Brokking (http://www.brokking.net/imu.html).
//De gebruikte code voor de interruptroutine is gemaakt door Amanda Ghassaei (https://www.instructables.com/Arduino-Timer-Interrupts/).

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//*
//*
//*

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//De gebruikte bibliotheken: Wire.h voor de I2C-communicatie met de MPU6050-sensor en AccelStepper.h voor het aansturen van de stappenmotoren
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <AccelStepper.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Enkele variabelen voor het berekenen van de hoek doormiddel van de MPU6050
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float tijdstap = 4000;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Het declareren van de stappenmotoren doormiddel de AccelStepperbibliotheek
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define dirPin 3
#define stepPin 2
#define dirPin2 5
#define stepPin2 4
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

int maxSnelheid = 2000;
int snelheid;
bool start = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Variabelen voor de PID controller
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float setpoint = 0;
float setpointTemp;
float Kp = 150;//150
float Ki = 1.5;//1
float Kd = 230;//50
float I;

float PIDtemp;
float PID;
float PID2;

int maxWaarde = 1000;
float vorigeError;
float stabilisatieWaarde;

void setup() {
  Wire.begin();                                                         //Begin de de I2C voor de MPU6050
  Serial.begin(57600);                                                //Seriële verbinding voor debuggen
  pinMode(LED_BUILTIN, OUTPUT);                                         
  digitalWrite(LED_BUILTIN, HIGH);                                      //Zet het ingebouwde ledje op de Arduino aan om te laten zien dat de kallibratie van de MPU6050 bezig is

  stepper.setMaxSpeed(maxSnelheid);                                     //Stelt de maximale snelheid van de motoren in
  stepper2.setMaxSpeed(maxSnelheid);

  setup_mpu_6050_registers();                                           //Stelt de MPU6050 in op een gegeven bereik.
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Kallibratie van de MPU6050
//Bij het starten hoort de robot stil gehouden te worden totdat het ledje uitgeschakeld wordt.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {                  //Code wordt vijfhonderd keer herhaald
    Serial.println(cal_int);
    read_mpu_6050_data();                                              //Lees de data af van de MPU
    gyro_x_cal += gyro_x;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    gyro_y_cal += gyro_y;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    gyro_z_cal += gyro_z;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    delayMicroseconds(3700);                                           //Delay van 3 ms tussen de aflezingen van de sensor
  }
  gyro_x_cal /= 1000;                                                  //Deel door 500 om de gemiddelde afwijking van de gyrosensor te bepalen
  gyro_y_cal /= 1000;                                                  //Deel door 500 om de gemiddelde afwijking van de gyrosensor te bepalen
  gyro_z_cal /= 1000;                                                  //Deel door 500 om de gemiddelde afwijking van de gyrosensor te bepalen

  digitalWrite(LED_BUILTIN, LOW);                                           

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Stelt een interruptroutine in. Dit stelt concreet gezegd een stuk code in dat met 8kHz (8000 keer/s) wordt uitgevoerd. De routine bevat de code om de motoren te laten draaien met een constant 
//veranderende snelheid.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  cli(); // stop interrupts
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts

  // De timervariabele wordt gelijkgesteld aan de huidige tijd op de timer van de Arduino plus 4000 microseconde. Dit zorgt ervoor dat de tijd dat de volgende loop moet beginnen bijgehouden wordt.
  loop_timer = micros() + 4000; 
}

ISR(TIMER2_COMPA_vect) {
  stepper.runSpeed();
  stepper2.runSpeed();
}

void loop() {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Het uitlezen van de data van de MPU6050 en berekeningen tot een hoek
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 1.20;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 2.19;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  Serial.println(angle_pitch_output);
  Serial.println(angle_roll_output);  

  float hoek = angle_roll_output;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID-berekeningen
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(PID > 10 || PID < -10)error += PID * 0.015; //zorgt dat de robot zich in de andere richting draait bij een duwtje
  
  
  //P-controller
  setpoint -= stabilisatieWaarde;
  float error = setpoint - hoek;
  float P = error * Kp;

  //I-controller
  I += Ki * error;

  //windup bescherming
  if (I > maxWaarde) I = maxWaarde;
  if (I < maxWaarde * -1) I = maxWaarde * -1;

  // D-controller
  //float D = Kd * (error - vorigeError);
  //vorigeError = error;
  float D = -Kd * (vorigeError - hoek);
  vorigeError = hoek;
  
  if(PID < 5 || PID > -5)PID = 0; //voorkomt kleine osscilaties als hij eenmaal gebalanceerd is
  
  PID = P + I + D; //de snelheid van de motor, in stappen per seconde

  if (PID < 0)setpoint -= 0.0015;
  if (PID > 0)setpoint += 0.0015;
      
  PID2 = PID * -1;

  //Serial.println(PID);

  if (hoek < 3 && hoek > -3) {
    start = true;
  }

  if (hoek > 28 || hoek < -28){
    start = false;
    stabilisatieWaarde = 0;
  }
  
  if (start) {
    stepper.setSpeed(PID);
    stepper2.setSpeed(PID2);
  } else {
    stepper.setSpeed(0);
    stepper2.setSpeed(0);
  }

  while (loop_timer > micros());
  loop_timer += 4000;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Enkele functies voor het uitlezen van de MPU6050
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable

}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
