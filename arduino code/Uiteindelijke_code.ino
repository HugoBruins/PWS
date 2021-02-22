//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Code voor Balanceerrobot
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

int maxSnelheid = 4000;
bool start = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Variabelen voor de PID controller
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float setpoint = 0;
float Kp = 140;//150
float Ki = 1.25;//1.5
float Kd = 230;//230
float I;

float PID;
float PID2;

int Imax = 1000;
float vorigeError;
float vorigeHoek;

void setup() {
  Wire.begin();                                                        //Begin de de I2C voor de MPU6050
  Serial.begin(57600);                                                 //Seriële verbinding voor debuggen
  pinMode(LED_BUILTIN, OUTPUT);                                         
  digitalWrite(LED_BUILTIN, HIGH);                                     //Zet het ingebouwde ledje op de Arduino aan om te laten zien dat de kallibratie van de MPU6050 bezig is

  stepper.setMaxSpeed(maxSnelheid);                                    //Stelt de maximale snelheid van de motoren in
  stepper2.setMaxSpeed(maxSnelheid);
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Kallibratie van de MPU6050
//Bij het starten hoort de robot stil gehouden te worden totdat het ledje uitgeschakeld wordt.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  setup_mpu_6050_registers();                                          //Stelt de MPU6050 in op een gegeven bereik.
  
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {                 //Code wordt duizend keer herhaald
    Serial.println(cal_int);
    read_mpu_6050_data();                                              //Lees de data af van de MPU
    gyro_x_cal += gyro_x;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    gyro_y_cal += gyro_y;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    gyro_z_cal += gyro_z;                                              //Lees de gyrodata af van de sensor en tel dit bij de variabele op
    delayMicroseconds(3700);                                           //Delay van 3,7 ms tussen de aflezingen van de sensor
  }
  gyro_x_cal /= 1000;                                                  //Deel door 1000 om de gemiddelde afwijking van de gyrosensor te bepalen
  gyro_y_cal /= 1000;                                                  //Deel door 1000 om de gemiddelde afwijking van de gyrosensor te bepalen
  gyro_z_cal /= 1000;                                                  //Deel door 1000 om de gemiddelde afwijking van de gyrosensor te bepalen

  digitalWrite(LED_BUILTIN, LOW);                                           

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Stelt een interruptroutine in. Dit stelt concreet gezegd een stuk code in dat met 8kHz (8000 keer/s) wordt uitgevoerd. De routine bevat de code om de motoren te laten draaien met een constant 
//veranderende snelheid.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  cli(); // stop de interrupts
  TCCR2A = 0;// zet hele TCCR2A register naar 0
  TCCR2B = 0;// zelfde voor TCCR2B
  TCNT2  = 0;//zet teller naar 0
  // set vergelijking register voor 8khz tussenstappen
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1
  // zet CTC mode aan
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS21);
  TIMSK2 |= (1 << OCIE2A);
  sei(); //sta interrupts toe

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
  read_mpu_6050_data();                                                //Lees de onverwerkte gyro- en versnellingsdata af van de MPU6050

  gyro_x -= gyro_x_cal;                                                //Trek de eerder berekende kalibratiewaarde af van de waarde van de gyrosensor
  gyro_y -= gyro_y_cal;                                                //Trek de eerder berekende kalibratiewaarde af van de waarde van de gyrosensor
  gyro_z -= gyro_z_cal;                                                //Trek de eerder berekende kalibratiewaarde af van de waarde van de gyrosensor

  //Gyrosensor hoekberekening
  //0.0000611 = 1 / 250Hz / 65.5
  angle_pitch += gyro_x * 0.0000611;                                   //Bereken de gedraaide afstand en tel deze op bij de angle_pitch variabele
  angle_roll += gyro_y * 0.0000611;                                    //Bereken de gedraaide afstand en tel deze op bij de angle_pitch variabele

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //Als de sensor een pitchhoek heeft gekregen, stel dan de rollvariabele bij
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //Als de sensor een rollhoek heeft gekregen, stel dan de pithchvariabele bij

  //Accelerometerhoekberekening
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //berkenen de totale vector van de accelerometer
  //57.296 = 1 / (3.142 / 180)
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Bereken de pitchhoek
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Bereken de rollhoek

  //Kalibratiewaarden voor wanneer de sensor niet volledig recht geplaatst is.
  angle_pitch_acc -= 1.20;                                              //Accelerometerkalibratiewaarde voor pitch
  angle_roll_acc -= 2.19;                                               //Accelerometerkalibratiewaarde voor roll

  if (set_gyro_angles) {                                               //Als de gyrosensor gestart is...
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Neem dan de gevonden kalibratiewaarden mee in de berekening
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Neem dan de gevonden offset mee in de berekening
  }
  else {                                                               //De eerste meting
    angle_pitch = angle_pitch_acc;                                     //Stel de pitchhoek van de gyrosensor gelijk aan die van de accelerometer
    angle_roll = angle_roll_acc;                                       //Stel de rollhoek van de gyrosensor gelijk aan die van de accelerometer
    set_gyro_angles = true;                                            //Laat weten dat de sensor begonnen is.
  }

  //Complementaire filter om schokkerige beweging te voorkomen
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Neemt 90% van de vorige waarde en telt er 10% van de gevonden waarde bij op
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Neemt 90% van de vorige waarde en telt er 10% van de gevonden waarde bij op

  Serial.println(angle_pitch_output);
  Serial.println(angle_roll_output);  

  float hoek = angle_roll_output;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID-berekeningen
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  setpoint -= stabilisatieWaarde;
  float error = setpoint - hoek;
  
  if(PID > 150 || PID < -150)error += PID * 0.0007; //zorgt dat de robot zich in de andere richting draait bij een duw
  
  //P-controller
  float P = error * Kp;

  I += Ki * error;

  //windup bescherming
  if (I > Imax) I = Imax;
  if (I < Imax * -1) I = Imax * -1;

  // D-controller
  //float D = Kd * (error - vorigeError);
  //vorigeError = error;
  float D = -Kd * (vorigeHoek - hoek);
  vorigeHoek = hoek;

  PID = P + I + D; //de snelheid van de motor, in stappen per seconde

  if (PID < 0)setpoint -= 0.0013;
  if (PID > 0)setpoint += 0.0013;
      
  PID2 = PID * -1;

  //Serial.println(PID);

  if (hoek < 3 && hoek > -3) {
    start = true;
  }

  if (hoek > 28 || hoek < -28){
    start = false;
    setpoint = 0;
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
void read_mpu_6050_data() {                                           
  Wire.beginTransmission(0x68);                                        //Start het communiceren met de MPU-6050
  Wire.write(0x3B);                                                    //stuur het gevraagde register
  Wire.endTransmission();                                              //eindig de versturing
  Wire.requestFrom(0x68, 14);                                          //vraag 14 bytes van de MPU-6050
  while (Wire.available() < 14);                                       //wacht totdat alle bytes binnen zijn
  acc_x = Wire.read() << 8 | Wire.read();                              //voeg de hoge en lage byte toe aan de acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //voeg de hoge en lage byte toe aan de acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //voeg de hoge en lage byte toe aan de variable
  temperature = Wire.read() << 8 | Wire.read();                        //voeg de hoge en lage byte toe aan de temperatuur variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //voeg de hoge en lage byte toe aan de gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //voeg de hoge en lage byte toe aan de gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //voeg de hoge en lage byte toe aan de gyro_z variable

}

void setup_mpu_6050_registers() {
  //Activeer de MPU-6050
  Wire.beginTransmission(0x68);                                        //Start het communiceren met de MPU-6050
  Wire.write(0x6B);                                                    //stuur het gevraagde startregister
  Wire.write(0x00);                                                    //Stel het gevraagde startregister in
  Wire.endTransmission();                                              //eindig het versturen
  //configureer de versnellingsmeter
  Wire.beginTransmission(0x68);                                        //Start het communiceren met de mpu-6050
  Wire.write(0x1C);                                                    //Stuur de gevraagde startregister
  Wire.write(0x10);                                                    //Stel het gevraagde startregister in
  Wire.endTransmission();                                              //Eindig het versturen
  //configureer de gyro
  Wire.beginTransmission(0x68);                                        //Start het communiceren met de mpu-6050
  Wire.write(0x1B);                                                    //Stuur de gevraagde startregister
  Wire.write(0x08);                                                    //Stel het gevraagde startregister in
  Wire.endTransmission();                                              //Eindig het versturen
}
