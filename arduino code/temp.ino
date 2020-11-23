#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() 
{
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)
}

void loop()
{
  Vector normAccel = mpu.readNormalizeAccel();

  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + 
                                            normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
}
