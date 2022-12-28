#
# JLC 2021-06-06  jean-luc.charles@member.fsfs.org
#

import os, sys
from time import sleep, time, strftime
from serial import Serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.pyplot as plt


class AduinoReader(object):
    '''This class can interface with an Arduino card using the USB Channel.
       Data printed by Arduino are collected : when the tag "DATA" is present,
       data are stored in a liste that can be processed to extract and plot
       numerical data.'''

    def __init__(self, tag="DATA", baudRate=115200,
                 debug=False, verbose=False,
                 stopKey="END", nbLines=None):
        
        self.__debug      = debug
        self.__verbose    = verbose
        self.__tag        = tag
        self.__br         = baudRate
        self.__serialPort = None
        self.__serialPortName = None
        self.__stopKey    = stopKey
        self.__nb_lines   = nbLines
        self.__DATA       = []
        self.__DATA_num   = None
        self.__dt         = None

    @property
    def serialPort(self): return self.__serialPort

    @property
    def DATA(self): return self.__DATA

    def print(self, message, end="\n"):
        if self.__verbose: print(message, end=end)

    def openUSB(self, verbose=False):
        listUSBports = ["/dev/ttyUSB0", "/dev/ttyACM0",  "/dev/ttyACM1",  "/dev/ttyACM2",
                        "COM1", "COM2", "COM3", "COM4", "COM9", "COM10", "COM13"]
        self.__serialPort = None
        for port in listUSBports:
            try:
                self.print(f"Trying to open port {port}...", end="")
                self.__serialPort = Serial(port, baudrate=115200, timeout=None)
                self.print(" succes;")
                break
            except:
                self.print(" failed.")
                continue
        if self.__serialPort is None:
            self.print("No available port ... sorry !")
            return 0
        else:
            self.__serialPortName = port
            if self.__debug:
                print(f"Using port {self.__serialPortName} : {self.__serialPort}")
            
        # Open serial serial if needed:
        sleep(0.5)
        if not self.__serialPort.is_open :
            self.__serialPort.open()
        sleep(0.5)

        return self.__serialPort

    def readUSB(self, nbLines=None, data_tag=None, send_tag=None, dt=None):

        if nbLines:
            self.__nb_lines = nbLines
            
        if not self.__serialPort.is_open:
            print("ERROR: Serial port is not opened, cannot read date!")
            return

        if self.__nb_lines is None:
            print(f"will read data until I get the keyword {self.__stopKey}")
        else:
            print(f"will read {self.__nb_lines} lines of data")

        if dt is not None :
            self.__dt = dt
            
        line_count = 0  
        self.__DATA = []
        done = False
        
        while True:

            if self.__dt is not None:
                t = self.__dt*line_count
                if t > 15 and not done:
                    print("End init")
                    done=True
                    
            data = self.__serialPort.readline()          # read data
            data = data.decode().strip()
            if self.__debug: print("<{}>".format(data))# debug only

            if send_tag is not None:
                if send_tag in data:
                    mesg = input("\tGive an answer to be sent... ")
                    data_to_send = bytes(mesg, encoding="ascii")
                    self.__serialPort.write(data_to_send)

            if line_count % 100 == 0: print('.', end="", flush=True)
            
            if data_tag is None or (data_tag is not None and data_tag in data):
                self.__DATA.append(data)
                line_count += 1
        
            if line_count >= self.__nb_lines:
                break
        print(" done")
        return self.__DATA
            
    def processData(self, data_cols, data_labels, sep=None):

        if len(data_cols) != len(data_labels):
          print("please give as many data labels as data columns...")
          return
        
        data_cols = list(data_cols)
        DATA = []
        for line in self.__DATA:
            data = np.array(line.split())[data_cols]
            DATA.append(data)
        self.__DATA_num    = np.asarray(DATA, dtype=float)
        self.__data_cols   = data_cols
        self.__data_labels = data_labels
        return self.__DATA_num

    def plotData(self, ylims=None):
        import matplotlib.pyplot as plt
        if self.__dt is not None:
            time = np.arange(len(self.__DATA_num))*self.__dt
        else:
            time = range(len(self.__DATA_num))
                            
        plt.figure(figsize=(10,8))
        for data, label in zip(self.__DATA_num.T, self.__data_labels):
            plt.plot(time, data, label=label)
        plt.legend()
        plt.grid()
        if ylims is not None: plt.ylim(ylims)
        dt = f"dt:{self.__dt*1e3:5.1}_ms" if self.__dt is not None else "dt:None"
        fileName  = 'MPU6050-'+dt+strftime("-%Y_%m_%d_%H_%M")+'.png'
        plt.savefig(fileName)
        plt.show()
        
    def readUSB_and_plot(self, max_lines:int, data_cols:tuple, data_labels:tuple, ylim:tuple,
                         data_tag:str, dt:float, send_tag:str=None, sep:str=None):
        ''' to read lines on the USB link with the ARDUINO, and plot data interactively'''
                   
        if not self.__serialPort.is_open:
            print("Error: Serial port is not opened, cannot read date!")
            return

        if len(data_cols) != len(data_labels):
          print("Error: please give as many data labels as data columns...")
          return

        self.__nb_lines = max_lines
        print(f"will read {max_lines} lines of data")

        self.__dt = dt
            
        self.__DATA = []
        self.__DATA_num    = []
        data_cols = list(data_cols)
        self.__data_cols   = data_cols
        self.__data_labels = data_labels

        X, Y = [], []
        fig = plt.figure(figsize=(10,8))
        ax = plt.axes(xlim=(0, max_lines), ylim=ylim)
        a0, = ax.plot([], [])
        ax.grid(True)
        
        self.line_count = 0  

        # update plot
        def update(frameNum, self, a0):
            
            line = self.__serialPort.readline()          # read data
            line = line.decode().strip()
            if self.__debug: print("<{}>".format(line))# debug only
            
            if send_tag is not None:
                if send_tag in line:
                    mesg = input("\tGive an answer to be sent... ")
                    data_to_send = bytes(mesg, encoding="ascii")
                    self.__serialPort.write(data_to_send)
            
            if data_tag is None or (data_tag is not None and data_tag in line):
                data = np.array(line.split())[data_cols]
                data_num = np.asarray(data, dtype=float)
                Y.append(data_num[0])
                X.append(self.line_count)
                a0.set_data(X, Y)
                self.line_count += 1
                if self.line_count >= 1000: self.anim.pause()
                #if self.line_count >= 1000: self.anim.even-source.stop()

        self.anim = animation.FuncAnimation(fig, update, fargs=(self, a0), interval=0)
        
        plt.show()
            
        print(" done")
        
        
if __name__ == "__main__":

    ard = AduinoReader(verbose=True, debug=True)
    serialPort = ard.openUSB()
    
    '''data = ard.readUSB(600, data_tag="ypr", send_tag="Send", dt=20e-3)
    DATA = ard.processData((1,2,3), ('yaw', 'picth', 'roll'))
    ard.plotData((-90, 90))'''

    ard.readUSB_and_plot(1000, (2,), ('Y',), (-90, 90), data_tag="ypr", send_tag="Send", dt=20e-3)
    #ard.mesureData()
