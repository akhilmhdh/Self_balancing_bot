#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include<PID_v1.h>
//arduino and mpu talk via I2C bus protocol
MPU6050 mpu;
double setpoint=0,Ki=500,Kp=450,Kd=25,input,output;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//PID initailzation
int MPUOffsets[6] = { -1362 , -2060, 1005 , -64, -56 ,-70};
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z] 
VectorFloat gravity;    // [x, y, z] 
float ypr[3];           // [yaw, pitch, roll]
void MPU6050Connect() {
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  Serial.print(F("DMP initializatio failed-Code"));
  Serial.print(devStatus);
  return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  //"Enabling DMP...
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection...Connection between 2 and 
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
}

void GetDMP() {
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    mpu.resetFIFO();return;// clear the buffer and start over
  } 
    mpuInterrupt = false;
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<
}
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  input= (ypr[1] *  180.0 / M_PI);
  Serial.println(input);
}
void setup() {
  // put your setup code here, to run once:
  //setting up the motor controlling pins and functions to obtain quaternion pitch angle.
  Serial.begin(115200); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
    //obtains the necessary pitch and stores in input
    GetDMP();
    //PID computation occurs and PWM values obtained are given
    myPID.Compute();
    if(output<0){
      digitalWrite(13,LOW);
      digitalWrite(12,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(7,LOW);
      analogWrite(11,-output);
      analogWrite(6,-output);
    }else{
      digitalWrite(13,HIGH);
      digitalWrite(12,LOW);
      digitalWrite(8,LOW);
      digitalWrite(7,HIGH);
      analogWrite(11,output);
      analogWrite(6,output);
   }
  }
}
