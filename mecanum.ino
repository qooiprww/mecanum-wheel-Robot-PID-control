#include<AFMotor.h>
#include <Wire.h>
#include <TimerOne.h>
 
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
 
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

#define MIN_MOTOR_SPD 100   //Minium Speed(PWM) for motors
#define MAX_MOTOR_SPD 255   //Maximum Speed(PWM) for motors

struct dataStruct {
  unsigned long _micros;  // to save response times
  int Xposition;          // The Joystick position values
  int Yposition;
  bool switch0;           // The Joystick push-down switch
  bool switch1;           // Rotate Left
  bool switch2;           // Rotate Right
} myData;       

#define M_STOP      0
#define M_FORWARD   1
#define M_BACKWARD  2

#define CMD_Stop              0
#define CMD_moveForward       1
#define CMD_moveBackward      2
#define CMD_moveLeft          3
#define CMD_moveRight         4
#define CMD_moveLeftForward   5
#define CMD_moveRightForward  6
#define CMD_moveLeftBackward  7
#define CMD_moveRightBackward 8
#define CMD_rotateLeft        9
#define CMD_rotateRight       10
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
// Set register address
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.endTransmission();

// Read Nbytes
Wire.requestFrom(Address, Nbytes); 
uint8_t index=0;
while (Wire.available())
Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
// Set register address
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.write(Data);
Wire.endTransmission();
}
 
// Initial time
long int ti;
volatile bool intFlag=false;
AF_DCMotor FrontRight(1);
AF_DCMotor RearRight(2);
AF_DCMotor RearLeft(3);
AF_DCMotor FrontLeft(4);

// Initializations
void setup()
{
// Arduino initializations
    Wire.begin();
    Serial.begin(115200);
    FrontRight.setSpeed(255);
    RearRight.setSpeed(255);
    RearLeft.setSpeed(255);
    FrontLeft.setSpeed(255);
 
// Set accelerometers low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,29,0x06);
// Set gyroscope low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
 
// Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
// Configure accelerometers range
I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
// Set by pass mode for the magnetometers
I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
// Request continuous magnetometer measurements in 16 bits
I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
 
pinMode(13, OUTPUT);
Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt
 
 
// Store initial time
ti=millis();
}
 
// Counter
long int cpt=0;
 
void callback()
{ 
intFlag=true;
digitalWrite(13, digitalRead(13) ^ 1);
}
 
// Main loop, read and display data
void loop()
{
moveLeft();

while (!intFlag);
intFlag=false;
 
// Display time
Serial.print (millis()-ti,DEC);
Serial.print ("\t");
 

 
 
// _____________________
// ::: Magnetometer :::
 
 
// Read register Status 1 and wait for the DRDY: Data Ready
 
uint8_t ST1;
do
{
I2Cread(MAG_ADDRESS,0x02,1,&ST1);
}
while (!(ST1&0x01));
 
// Read magnetometer data 
uint8_t Mag[7]; 
I2Cread(MAG_ADDRESS,0x03,7,Mag);
 
// Create 16 bits values from 8 bits data
 
// Magnetometer
int16_t mx=-(Mag[3]<<8 | Mag[2]);
int16_t my=-(Mag[1]<<8 | Mag[0]);
int16_t mz=-(Mag[5]<<8 | Mag[4]);
 
 
// Magnetometer
Serial.print (mx+200,DEC); 
Serial.print ("\t");
Serial.print (my-70,DEC);
Serial.print ("\t");
Serial.print (mz-700,DEC); 
Serial.print ("\t");
 
// End of line
Serial.println("");
// delay(100); 
}

void STOP() {
  FrontLeft.run(RELEASE);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(RELEASE);
}
void moveForward() {
  FrontLeft.run(FORWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(FORWARD);
}
void moveBackward() {
  FrontLeft.run(BACKWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(BACKWARD);
}
void moveLeft() {
  FrontLeft.run(BACKWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(BACKWARD);
}
void moveRight() {
  FrontLeft.run(FORWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(FORWARD);
}
void moveLeftForward() {
  FrontLeft.run(RELEASE);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(RELEASE);
}
void moveRightForward() {
  FrontLeft.run(FORWARD);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(FORWARD);
}
void moveLeftBackward() {
  FrontLeft.run(BACKWARD);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(BACKWARD);
}
void moveRightBackward() {
  FrontLeft.run(RELEASE);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(RELEASE);
}
void rotateLeft() {
  FrontLeft.run(BACKWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(FORWARD);
}
void rotateRight() {
  FrontLeft.run(FORWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(FORWARD);
  RearRight.run(BACKWARD);
}

