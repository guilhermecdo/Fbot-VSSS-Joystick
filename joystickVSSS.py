from tkinter import Button
from xbox360controller import Xbox360Controller #pip install xbox360controller
import numpy as np
import time
import serial

wheelRadius=0.03
robotLength=0.07
id=1

controller=Xbox360Controller(0, axis_threshold=0.0001)

s=serial.Serial('/dev/ttyUSB1')# Radio serial Port 
s.baudrate=9600
s.bytesize=serial.EIGHTBITS #number of bits per bytes
s.parity=serial.PARITY_NONE #set parity check: no parity
s.stopbits=serial.STOPBITS_ONE #number of stop bits
s.timeout=0 #non-block read
s.xonxoff=False #disable software flow control
s.rtscts=False #disable hardware (RTS/CTS) flow control
s.dsrdtr=False #disable hardware (DSR/DTR) flow control
s.setRTS(0) # <---- THIS SOLVED THE PROBLEM READING SERIAL DATA WITH APC220

def radioWrite(message:np.array):
        print(message)
        s.write(message)

def messageCreator(leftWheelVelocity:float,rightWheelVelocity:float,x:int,y:int,id:int,vel):
    
    leftWheelVelocity=int((leftWheelVelocity*127/0.12)*vel)
    rightWheelVelocity=int((rightWheelVelocity*127/0.12)*vel)
    
    if y<0:
        leftWheelVelocity+=128
        rightWheelVelocity+=128
    elif y==0:
        if x>0:
            rightWheelVelocity+=128
        elif x<0:
            leftWheelVelocity+=128

    return np.array([id,leftWheelVelocity,rightWheelVelocity],np.uint8)
 
def kinematics(x,y):

    y_corr=np.sqrt(y**2)
    x_corr=np.sqrt(x**2)

    if y_corr >= 0.95:
        u2=1
        u1=0
    elif x_corr >= 0.95:
        u2=0
        u1=1
    elif y_corr < 0.95 and x_corr > 0.35:
        u2=0.6
        u1=0.6
    else:
        u2=0
        u1=0
    
    if y>=0:
        u2=u2
    else:
        u2=u2*-1
    
    if x>=0:
        u1=u1
    else:
        u1=u1*-1
    
    x=u1
    y=u2

    if x==0:
        angularVelocity=0
    else:
        angularVelocity=np.tan(y/x)
    
    linearVelocity=np.sqrt(x**2+y**2)

    leftWheelVelocity=((2*linearVelocity)-(angularVelocity*robotLength))*(2*wheelRadius)
    rightWheelVelocity=((2*linearVelocity)+(angularVelocity*robotLength))*(2*wheelRadius)

    return leftWheelVelocity, rightWheelVelocity

while not controller.button_start.is_pressed:
    if controller.button_a.is_pressed:
        
        x=controller.axis_l.x
        y=controller.axis_l.y*-1

        leftWheelVelocity,rightWheelVelocity=kinematics(x,y)
        message=messageCreator(leftWheelVelocity,rightWheelVelocity,x,y,id,controller.trigger_r.value)
        radioWrite(message)
        time.sleep(1/60)# message sending frequency