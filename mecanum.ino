#include<AFMotor.h>
#include <Wire.h>
#include <TimerOne.h>
#include <PID_v1.h>

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

double Setpoint, Input, Output;
long int cpt=0;
double consKp=2, consKi=0.02, consKd=0.011;
int16_t imx, imy, imz;
int16_t mx, my, mz;
int con = 0;
// Initial time
long int ti;
volatile bool intFlag=false;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

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
  Serial.begin(9600);
  
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

  while (!intFlag);
  intFlag=false;
  Setpoint = resetcompass();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-125, 130);
}

void loop()
{
  con++;
  if(con == 100){
    Setpoint=resetcompass();
    con = 0;
  }
  while (!intFlag);
  intFlag=false;
 
// _____________________
// ::: Magnetometer :::
 
  mx = 0;
  my = 0;
  mz = 0;
  for(int i = 0; i < 10; i++){
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
 
  // Magnetometer 
    mx += (Mag[3]<<8 | Mag[2]);
    my += (Mag[1]<<8 | Mag[0]);
    mz += (Mag[5]<<8 | Mag[4]);
}

  Input = atan2(my/10,mx/10)*100;
  myPID.SetTunings(consKp, consKi, consKd);
  myPID.Compute();
  double spd = Output;
  Serial.println(Setpoint,DEC);
  Serial.println("");
  Serial.print (Input,DEC); 
  Serial.println("");
  Serial.print (spd,DEC);
  Serial.println("");
  if (Serial.available()) {
    char cmd = Serial.read();
    if(cmd == 'w'){
      moveForward(spd);
    }
    else if(cmd == 's'){
      moveBackward(spd);
    }
    else if(cmd == 'a'){
      moveLeft(spd);
    }
    else if(cmd == 'd'){
      moveRight(spd);
    }
    else if(cmd == 'q'){
      moveLeftForward(spd);
    }
    else if(cmd == 'e'){
      moveRightForward(spd);
    }
    else if(cmd == 'z'){
      moveLeftBackward(spd);
    }
    else if(cmd == 'c'){
      moveRightBackward(spd);
    }
    else if(cmd == '1'){
      rotateLeft();
    }
    else if(cmd == '3'){
      rotateRight();
    }
    else if(cmd == 'x'){
      STOP();
    }
  }
}

void STOP() {
  for(;Serial.read() == 'j';){
  FrontLeft.run(RELEASE);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(RELEASE);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveForward(double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125+s);
  RearRight.setSpeed(125+s);
  RearLeft.setSpeed(125-s); 
  FrontLeft.setSpeed(125-s);
  FrontLeft.run(FORWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(FORWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveBackward(double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125-s);
  RearRight.setSpeed(125-s);
  RearLeft.setSpeed(125+s); 
  FrontLeft.setSpeed(125+s);
  FrontLeft.run(BACKWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(BACKWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveLeft(double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125+s);
  RearRight.setSpeed(125-s);
  RearLeft.setSpeed(125-s); 
  FrontLeft.setSpeed(125+s);
  FrontLeft.run(BACKWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(BACKWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveRight( double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125-s);
  RearRight.setSpeed(125+s);
  RearLeft.setSpeed(125+s); 
  FrontLeft.setSpeed(125-s);
  FrontLeft.run(FORWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(FORWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveLeftForward(double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125+s);
  RearLeft.setSpeed(125-s); 
  FrontLeft.run(RELEASE);
  FrontRight.run(FORWARD);
  RearLeft.run(FORWARD);
  RearRight.run(RELEASE);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveRightForward(double s) {
  for(;Serial.read() == 'j';){
  RearRight.setSpeed(125+s);
  FrontLeft.setSpeed(125-s);
  FrontLeft.run(FORWARD);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(FORWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveLeftBackward(double s) {
  for(;Serial.read() == 'j';){
  RearRight.setSpeed(125-s);
  FrontLeft.setSpeed(125+s);
  FrontLeft.run(BACKWARD);
  FrontRight.run(RELEASE);
  RearLeft.run(RELEASE);
  RearRight.run(BACKWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void moveRightBackward(double s) {
  for(;Serial.read() == 'j';){
  FrontRight.setSpeed(125-s);
  RearLeft.setSpeed(125+s); 
  FrontLeft.run(RELEASE);
  FrontRight.run(BACKWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(RELEASE);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void rotateLeft() {
  for(;Serial.read() == 'j';){
  FrontLeft.run(BACKWARD);
  FrontRight.run(FORWARD);
  RearLeft.run(BACKWARD);
  RearRight.run(FORWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}
void rotateRight() {
  for(;Serial.read() == 'j';){
  FrontLeft.run(FORWARD);
  FrontRight.run(BACKWARD);
  RearLeft.run(FORWARD);
  RearRight.run(BACKWARD);
  if(Serial.read() == 'k'){
    break;
  }
  }
}

double resetcompass(){
  int16_t x=0, y=0, z=0;
  double set;
  for(int i = 0; i < 10; i++){
  uint8_t ST1;
  do
  {
  I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));
  uint8_t Mag[7]; 
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  x+=(Mag[3]<<8 | Mag[2]);
  y+=(Mag[1]<<8 | Mag[0]);
  z+=(Mag[5]<<8 | Mag[4]);
  }
  set = atan2(y/10,x/10)*100;
  return set;
}

void callback()
  { 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

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
 
