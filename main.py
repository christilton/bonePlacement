from machine import Pin, PWM, I2C
import time, struct
import uasyncio as asyncio
import Servo
import Giotto
import ArmDrive
import BlueTooth as bt
import myWifi
import mqtt
import json

myWifi.connect(myWifi.TUFTS)

vacuum = Pin(21, Pin.OUT)
vacuum.off()

message = ""
dc = 1500


# class to drive the car, assumes 360 degree servos
class TankDrive:
    def __init__(self,leftPin,rightPin,myoffset):
        self.leftM = Servo.ContinuousServo(pin_number=leftPin, duty_us =dc, offset = myoffset)
        self.rightM = Servo.ContinuousServo(pin_number=rightPin, duty_us = dc, offset = myoffset)
        self.leftM.set_speed(0)
        self.rightM.set_speed(0)

    def move(self,joyY,joyX):
        leftMotor = -100 * (joyY - joyX)
        rightMotor = 100 * (joyY + joyX)
        if ((joyY + joyX) < .05 and (joyY + joyX) > -.05):
            leftMotor = 0
            rightMotor = 0
        
        self.leftM.set_speed(int(leftMotor))
        self.rightM.set_speed(int(rightMotor))
    
    def surgeryMove(self,forward,backward,fidelity):
        
        if forward:
            self.leftM.set_speed(10)
            self.rightM.set_speed(-10)
            time.sleep(fidelity / 20)
            self.leftM.set_speed(0)
            self.rightM.set_speed(0)
            time.sleep(.1)
        elif backward:
            self.leftM.set_speed(-10)
            self.rightM.set_speed(10)
            time.sleep(fidelity / 20)
            self.leftM.set_speed(0)
            self.rightM.set_speed(0)
            time.sleep(.1)

def gotMessage(topic, msg):
    
    #print("recieved a message")
    #print((topic.decode(), msg.decode()))
    
    info = msg.decode()
    message = info.split(", ")
    inputDict = json.loads(info)
    #print(inputDict)
    
    if inputDict['vac']:
        vacuum.on()
    else:
        vacuum.off()
        
    if inputDict['mode']:
        theCar.move(inputDict['leftJoyY'],inputDict['leftJoyX'])
    else:
        theCar.surgeryMove(inputDict['triangle'],inputDict['circle'],inputDict['fidelity'])
    
        
theCar= TankDrive(26,27,myoffset=-17)

theArm = ArmDrive.ArmDrive(pinA=12,pinB=13,pinZ=14,pinR=15,lenA=100,lenB=100,minX=10)

try:
    fred = mqtt.MQTTClient('MyPico', 'broker.hivemq.com', keepalive=1000)
    print('Connected to MQTT')
    fred.connect()
    fred.set_callback(gotMessage)
except OSError as e:
    print('Failed')
    
fred.subscribe("theVacuum")

try:
    while True:
        fred.check_msg()
        time.sleep(0.05)
        #debug
        #testInput = input("Enter angles: A B Z\n").split(" ")
        #theArm.setAngles(int(testInput[0]),int(testInput[1]),int(testInput[2]))
        #testInput = input("Enter position: X Y Z R\n").split(" ")
        #print(theArm.move(int(testInput[0]),int(testInput[1]),int(testInput[2])))
        #theArm.armR.set_speed(int(testInput[3]))
except Exception as e:
    print(e)
finally:
    fred.disconnect()



