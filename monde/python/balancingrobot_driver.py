# Importing Libraries

import serial
from time import time

from stable_baselines3 import PPO
class DriverPPO:
    def __init__(self, path):
        self.model = PPO.load(path)
    
    def control(self, t):
        action, _states = self.model.predict(t)
        return "%.2f" % (0.8*action[0])


from simple_pid import PID
class DriverPID:
    def __init__(self, p, i, d):
        self.pid = PID(p, i, d, setpoint=0.0)
        self.pid.sample_time = 0.02

    def control(self, t):
        return "%.3f" % -self.pid(t[2])


def launch(driver):
    arduino = serial.Serial(port='COM9', baudrate=250000, timeout=.1)

    v = arduino.readline()
    while(v != b'--START--\r\n'):
        print(v)
        v = arduino.readline()
    print(v)
    arduino.write(bytes("0.0", 'utf-8'))


    print("\nMPU6050 Calibration Sketch")
    print("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n")

    angle = 0
    for i in range(500):
        arduino.write(bytes("0.0", 'utf-8'))
        s = arduino.readline()
        v = s.decode("utf-8")
        t = list(map(float,v.split("\t")))
        angle += t[2]
    setpoint = round(angle/500.0,2)

    print("Setpoint : ", setpoint)
    print("calibration finished")
    print("please put your robot vertically !\n Starting soon !")
    arduino.write(bytes("0.0", 'utf-8'))


    control = driver.control([0.0,0.0,0.0,0.0])
    timer = time()
    while True: 
        s = arduino.readline()
        v = s.decode("utf-8")
        t = list(map(float,v.split("\t")))
        t[2] -= setpoint
        
        control = driver.control(t)

        #print(t, "\t", control)
        arduino.write(bytes(str(control) + "\n", 'utf-8'))
        
        if(time() - timer > 0.02):
            assert(False)
        while(time() - timer <= 0.02):
            continue
        timer = time()


#launch(DriverPID(57.2958 * 0.09,57.2958 * 0.005, 57.2958 * 0.00125))
launch(DriverPPO("./ai_models/model3"))