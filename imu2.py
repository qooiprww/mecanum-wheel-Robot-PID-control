import math
import time
import RTIMU

SETTINGS_FILE = "/home/pi/jbc/scripts/RTIMULib.ini"
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

t_print = time.time()
t_damp = time.time()
t_fail = time.time()
t_fail_timer = 0.0
t_shutdown = 0

if (not imu.IMUInit()):
    hack = time.time()
    if (hack - t_print) > 1.0:
        print("IMU init failed")
        t_print = hack
        t_shutdown += 1
        if t_shutdown > 9:
            sys.exit(1)
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
poll_interval = imu.IMUGetPollInterval()
while True:
    if imu.IMURead():
        data = imu.getIMUData()
        compass = data["compass"]
        global heading
        x = degrees(compass[0])
        y = degrees(compass[1])
        x = degrees(compass[2])
        heading = math.atan2(x,y)
