# To be interpreted by Python3
import sys, os
import matplotlib.pyplot as plt
import numpy as np
from time import sleep, time
from random import randint
import math as m
from serial import Serial

from stable_baselines3 import PPO

# Loading PPO model weights:
print("Loading PPO model weights... ", end="")
model_path = "./ai/models/balancingrobot/"
#model_name = "BalancingRobotEnv_CopSim_PPO_21-07-21_11-05-19ZIP/model.zip"
#model_name = "BalancingRobotEnv_CopSim_PPO_21-07-21_12-54-19/ZIP/model.zip"
model_name = "last/ZIP/model.zip"


model = PPO.load(os.path.join(model_path, model_name))
print("done.")

#   Windows : "COM3" or "COM4" or ...
# Connection to serial port:
# Serial port name  (the name is in the title bar of the monitor window):
#   Windows : "COM3" or "COM4" or ...
#   Linux   : "/dev/ttyACM0" (or see the monitor window title bar)
#   Mac OS X: see the monitor window title bar
#
# Parameters: # of data bits -> 8
#             # od STOP bit  -> 1
#             Parity         -> None
#             Speed          -> 9600, 14400, 19200, 28800, 38400,
#                                         57600, or 115200

listUSBports = ["/dev/ttyUSB0", "/dev/ttyUSB1",
                "/dev/ttyACM0",  "/dev/ttyACM1",  "/dev/ttyACM2",
                "COM1", "COM2", "COM3", "COM4", "COM9", "COM10", "COM13"]
serialPort = None
for port in listUSBports:
    try:
        print(f"Trying to open port {port}...", end="")
        serialPort = Serial(port, baudrate=250000, timeout=None)
        print(" succes;")
        break
    except:
        print(" failed.")
        continue
if serialPort is None:
    print("No available port ... sorry !")
    sys.exit()
else:
    serialPortName = port
print(f"Using port {serialPortName} : {serialPort}")

# Open serial port:
sleep(0.5)
if not serialPort.is_open :
    serialPort.open()
    sleep(0.5)
sleep(0.5)


# wait for Arduino ready:
print("Waiting for ARDUINO ....", end="")

# action, _ = model.predict(obs, deterministic=deterministic)
stateEnv =np.zeros((1,4))

data, M = b"", []
while True:
    data = serialPort.readline().decode().strip()
    print(data)
    if "Arduino OK" in data: break
    if "MPU_deg:" in data:
        t, coder_angle = float(data[1]), float(data[3])
        M.append([t, coder_angle])
    
print(" OK !", flush=True)
                                         
'''# wait for Arduino ready:
print("Waiting for ARDUINO ....", end="")
data = b""
while "Arduino OK" not in data.decode().strip() :
    data = serialPort.readline()
    print(data.decode().strip())
print(" OK !", flush=True)    '''

plotWanted = False # you can change this : True or False
stateEnv =np.zeros((4,))

loop_delay = 0.1
t_start = time()

# run some episodes:
for episode in range(10):
    
    # read initial balancingrobot state: this is an equivalent step to the reset()
    # used with Gym environment.
    data = serialPort.readline()   # read serial port is a blocking operation...
    data = data.decode().strip()
    x_pos, x_lin_veloc, theta_, ang_veloc, done  = [float(x) for x in data.split()[1:]]
    stateEnv = [x_pos, x_lin_veloc, theta_, ang_veloc]

    step, M = 0, []
    print(step, stateEnv, end="")

    while not done:
        t0 = time()
        # compute the Network action:
        action = model.predict(stateEnv, deterministic=False)
        action = -float(action [0])
        print(f"  action: {action:.2f}")
        # print("fin PPO: {:.3f} ms".format((time()-t0)*1000))
        # write action for Arduino BalancingRobot environment:
        serialPort.write(bytes(f"{action:.2f}", encoding="ascii"))
        M.append([step, action, data])

        step += 1  
        # now, wait for Arduino BalancingRobot environment response:    
        data = serialPort.readline()   # read a line of data
        data = data.decode().strip()
        data = [float(x) for x in data.split()] # something, x, x_dot, theta, theta_dot, done
        #print("T boucle: {:.1f} ms".format((time()-t0)*1000))
        print(f"{step:03d}", data, end="")
        #print(data[0],data[3]*180/np.pi)
        stateEnv = data[1:-1]
        #done = bool(data[-1])
              
    
    #data = serialPort.readline().decode().strip()
    #print(data)
    
    print("\n\nWaiting for ARDUINO ....", end="")
    data = b""
    while "Arduino OK" not in data.decode().strip() :
        data = serialPort.readline()
        print(data.decode().strip())
    print(" OK !", flush=True)

    
    print(M)
    print("episode [{:02d}] ended after {:03d} steps\n".format(episode,step))

    if (not plotWanted):
        continue

    
M = np.array(M)
if len(M):
    np.set_printoptions(suppress=True)
    plt.figure(figsize=(12,6))
    plt.plot(M[:,0], M[:,2],'.-m', label="CODER [°]")
    plt.legend(loc='lower right')
    plt.show()

serialPort.flushOutput()
sleep(0.5)

serialPort.close()          # close serial port
