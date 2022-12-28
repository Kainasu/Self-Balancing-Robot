# Importing Libraries
import serial
from time import time, sleep
arduino = serial.Serial(port='COM9', baudrate=250000, timeout=.1)

 
v = arduino.readline()
while(v != b'--START--\r\n'):
    print(v)
    v = arduino.readline()
print(v)

arduino.write(bytes("0.01", 'utf-8'))
sleep(1)
timer = time()
while True:
    s = arduino.readline()
    val = round(float(s.decode("utf-8")), 2)
    print(">", val)
    arduino.write(bytes(str(round(val + 0.01, 2)) + "\n", 'utf-8'))
    print("<", round(val + 0.01, 2))

    if(time() - timer > 0.02):
        assert(False)
    while(time() - timer <= 0.02):
        continue
    timer = time()
