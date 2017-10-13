/* MPU9250 Serial Output

 Hardware setup:
 MPU9250 Breakout --------- ESP32
 VDD ---------------------- 3.3V
 SDA ----------------------- D22
 SCL ----------------------- D21
 GND ---------------------- GND
 */

//#include "quaternionFilters.h"
#include "MPU9250.h"

#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 2;  // Set up pin 13 led for toggling

MPU9250 myIMU;

void setup()
{
  Wire.begin(22, 21); //ESP32 SDA=GPIO22, SCL=GPIO21 
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

    Serial.println("Data start");
    Serial.println("Acceleration[mg], Gyroscope[deg/s], Magnetometer [mG]");
    Serial.println("ax, ay, az, gx, gy, gz, mx, my, mz");

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s

  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  if (myIMU.delt_t > 500)
  {
    if(SerialDebug)
    {
      Serial.print((int)1000*myIMU.ax); Serial.print(" ,");
      Serial.print((int)1000*myIMU.ay); Serial.print(" ,");
      Serial.print((int)1000*myIMU.az); Serial.print(" ,");

      Serial.print(myIMU.gx, 2); Serial.print(" ,");
      Serial.print(myIMU.gy, 2); Serial.print(" ,");
      Serial.print(myIMU.gz, 2); Serial.print(" ,");

      Serial.print((int)myIMU.mx ); Serial.print(" ,");
      Serial.print((int)myIMU.my ); Serial.print(" ,");
      Serial.println((int)myIMU.mz ); //Serial.print(" ,");

//      Serial.print("q0 = "); Serial.print(*getQ());
//      Serial.print(" qx = "); Serial.print(*(getQ() + 1));
//      Serial.print(" qy = "); Serial.print(*(getQ() + 2));
//      Serial.print(" qz = "); Serial.println(*(getQ() + 3));
    }


    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)
} //loop()
