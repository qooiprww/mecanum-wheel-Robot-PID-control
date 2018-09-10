mecanum-wheel-Robot-PID-control
====
### Arduino Uno, motor control shield L293D, MPU-9250 sensor mudule, 4 * JGA25-370 motor, jumper wires, 4*dumpers & 3s lipo are used in the actual project.
[MPU-9250 sensor setup on Raspberry Pi 3B](#mpu-9250-sensor-setup-on-raspberry-pi-3b)<br>
[MPU-9250 sensor setup on Arduino Uno](#mpu-9250-sensor-setup-on-arduino-uno)<br>
## MPU-9250 sensor setup on Raspberry Pi 3B
### 1. Build the connection
Connect `SCL` & `SDA` pin to pin5 & pin3 of Raspberry Pi, then connect `Vcc` & `Ground` pin to the 3.3V power pin(pin1) & Ground pin of Raspberry Pi<br>
![connection](https://github.com/qooiprww/mecanum-wheel-Robot-PID-control/blob/master/raspberry-pi-mpu6050-six-axis-gyro-accelerometer-5.jpg "MPU-9250 & Raspberry Pi")
### 2. Setup the Raspberry Pi
First we set up a folder for the project in the /home/pi directory and get the i2c tools installed
```Bash 
mkdir jbc
cd jbc
mkdir scripts
sudo apt-get install i2c-tools
```
Click the Menu in the top Left Corner, go to Prefrences->Raspberry Pi Configuration->Interfaces and enable i2c (this will require a reboot).<br>
### 3. Setup the environment
```Bash
sudo i2cdetect -y 1
   ```
If the number 68 is in the grid that displays, mpu is detected.<br>
Then install cmake for building the sensor's library
```Bash 
sudo apt-get install cmake
```
Install python to setup the develop environment
```Bash 
sudo apt-get install python-dev
```
Install Octave to calibrate the sensor
```Bash 
sudo apt-get install octave
```
Next, get the RTIMULib Library from Github
```Bash 
git clone https://github.com/richards-tech/RTIMULib2.git
cd RTIMULib2/Linux/RTIMULibCal
make -j4
sudo make install
```
The last thing is to setup our working directory
```Bash 
cp -r /home/pi/jbc/RTIMULib2/RTEllipsoidFit/ /home/pi/jbc/
```

### 4. Calibrating the MPU-9250 on the Raspberry Pi
Change to the RTEllipsoidFit folder and run RTIMULibCal:
```Bash 
cd /home/pi/jbc/RTEllipsoidFit/
RTIMULibCal
```
Then follow the instructions to calibrate the sensor.<br>
After finished, copy the sonser data file to our working dictionary
```Bash 
cp RTIMULib.ini /home/pi/jbc/scripts/
```

### 5. Copy the script
Download the [imu2.py](https://github.com/qooiprww/mecanum-wheel-Robot-PID-control/blob/master/imu2.py) file and place it under scripts folder<br>
## MPU-9250 sensor setup on Arduino Uno
### 1. Build the connection
Connect `SCL` & `SDA` pin to A5 & A4 of Arduino Uno, then connect `Vcc` & `Ground` pin to the 3.3V power pin(pin1) & Ground pin of Arduino Uno<br>
![connection](https://github.com/qooiprww/mecanum-wheel-Robot-PID-control/blob/master/arduino-and-mpu9250.png "MPU-9250 & Arduino")<br>
### 2. Download & install Arduino Librarys
Download library zip files from the [Arduino_Library](https://github.com/qooiprww/mecanum-wheel-Robot-PID-control/blob/master/Arduino_Library).<br>
Unpack the folders and paste them to the `library` folder under Arduino root directory.
### 3. Copy the script
Download the [mecanum.ino](https://github.com/qooiprww/mecanum-wheel-Robot-PID-control/blob/master/mecanum.ino) file and load it onto Arduino Uno.
