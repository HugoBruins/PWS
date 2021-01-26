///////////////////////////////////////////////////////////////////////////////////////
/*Terms of use
  ///////////////////////////////////////////////////////////////////////////////////////
  //THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  //IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  //FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  //AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  //LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  //OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  //THE SOFTWARE.


  ///////////////////////////////////////////////////////////////////////////////////////
  //Support
  ///////////////////////////////////////////////////////////////////////////////////////
  Website: http://www.brokking.net/imu.html
  Youtube: https://youtu.be/4BoIE8YQwM8
  Version: 1.0 (May 5, 2016)

  ///////////////////////////////////////////////////////////////////////////////////////
  //Connections
  ///////////////////////////////////////////////////////////////////////////////////////
  Power (5V) is provided to the Arduino pro mini by the FTDI programmer

  Gyro - Arduino pro mini
  VCC  -  5V
  GND  -  GND
  SDA  -  A4
  SCL  -  A5

  LCD  - Arduino pro mini
  VCC  -  5V
  GND  -  GND
  SDA  -  A4
  SCL  -  A5
*//////////////////////////////////////////////////////////////////////////////////////

//Include LCD and I2C library

#include <Wire.h>
#include <AccelStepper.h>

#define dirPin 3
#define stepPin 2
#define dirPin2 5
#define stepPin2 4
#define motorInterfaceType 1

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
boolean done;


AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
//Initialize the LCD library

int maxSnelheid = 2000;
int snelheid;
bool reverse;

//voor de MPU 6050
float hoekAccel;
float hoekGyro;

float tijdstap = 4000;
float pulseVertraging;
unsigned long vorigetijd;
float aantalPulsen;
float beschikbarePulsen;


//voor de PID controller
float setpoint = 0;
float PID;
float Kp = 50;
float Ki = 0;
float Kd = 0;
int maxWaarde = 400;
float vorigeError;
float I;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                               //Use only for debugging
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  stepper.setMaxSpeed(maxSnelheid);
  stepper2.setMaxSpeed(maxSnelheid);

  //TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  //TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  //TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  //TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  //OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  //TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  //Set the LCD cursor to position to position 0,1
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    Serial.println(cal_int);
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset


  digitalWrite(13, LOW);                                               //All done, turn the LED off

  digitalWrite(LED_BUILTIN, LOW);
  loop_timer = micros() + 4000;
}

void loop() {

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
  angle_pitch_acc -= 0.43;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 3.38;                                               //Accelerometer calibration value for roll

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

  //Serial.println(angle_roll_output);

  float hoek = angle_roll_output;

  ////////////////////////////////////////////////////////
  //PID-berekeningen
  ///////////////////////////////////////////////////////////

  //P-controller
  float error = setpoint - hoek;
  float P = error * Kp;

  //I-controller
  I += Ki * error;

  //windup bescherming
  if (I > maxWaarde) I = maxWaarde;
  if (I < maxWaarde * -1) I = maxWaarde * -1;

  // D-controller
  float D = Kd * (error - vorigeError);
  vorigeError = error;

  float PID = P + I + D; //de snelheid van de motor, in stappen per seconde
  
  //PID = abs(round(PID));
  Serial.println(PID);

  if (PID > maxWaarde)PID = maxWaarde;
  else if (PID < maxWaarde * -1)PID = maxWaarde * -1;

  stepper.setSpeed(PID);
  stepper.runSpeed();
  stepper2.setSpeed(-PID);
  stepper2.runSpeed();

  while (loop_timer > micros());
  loop_timer += 4000;
}
//pulseVertraging = abs((tijdstap/PID)/2);
//aantalPulsen = abs(PID / 250);

//if (!start) {
//beschikbarePulsen += aantalPulsen;
//huidigeVertraging = pulseVertraging;
//}

//Serial.println(beschikbarePulsen);

/*int tijdsDelay = 4000;    //zet alles op 250hz
  long tijd = micros();
  int stapTrillingstijd = tijdsDelay / PID;
  tijd += stapTrillingstijd;

  for (int i = 0; i < PID; i++) {
  if(reverse){
    digitalWrite(3, HIGH);
    digitalWrite(5, LOW);
  }else{
    digitalWrite(5, HIGH);
    digitalWrite(3, LOW);
  }
  digitalWrite(2, HIGH);   //zet puls op HIGH
  digitalWrite(4, HIGH);
  while (tijd > micros());
  tijd += stapTrillingstijd;
  digitalWrite(2, LOW);   //zet puls op HIGH
  digitalWrite(4, LOW);
  }

  while (loop_timer > micros());
  loop_timer += 4000;
  }

  ISR(TIMER2_COMPA_vect){ // Dit stuk code wordt elke 20 microseconden aangeroepen
  if(PID < 0){ // als de waarde PID negatief is, stuur dan een puls naar de drivers om de richting om te draaien
  PORTD |=  B00001000; //zet de pin D3 op HIGH en D5 LOW
  PORTD &= B11101111;
  }else{
  PORTD |=  B0010000; //zet de pin D5 op HIGH en D3 LOW
  PORTD &= B11111011;
  }

  if(beschikbarePulsen > 1 && !done){ //als er pulsen beschikbaar zijn vanuit een eerder deel, voer ze dan uit
  PORTD = PORTD | B00010100; //zet pinnen D2 en D4 op HIGH
  beschikbarePulsen -= 1;
  done =  true;
  }else if(done){
  done = false;
  PORTD = PORTD & B11101011;
  }
  }
*/

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
