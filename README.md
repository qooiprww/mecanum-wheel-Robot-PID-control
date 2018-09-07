mecanum-wheel-Robot-PID-control
====
MPU-9250 sensor setup on Raspberry Pi 3B
----
1. Connect SCL & SDA pin to pin5 & pin3 of Raspberry Pi, then connect Vcc & Ground pin to the 3.3V power pin(pin1) & Ground pin of Raspberry Pi<br>
2. sudo apt-get install i2c-tools<br>
3. Click the Menu in the top Left Corner, go to Prefrences->Raspberry Pi Configuration->Interfaces and enable i2c (this will require a reboot).<br>
4. sudo i2cdetect -y 1<br>
If the number 68 is in the grid that displays, mpu is detected\<br>
5. sudo apt-get install cmake<br>
6. sudo apt-get install python-dev<br>
7. 
