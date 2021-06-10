#!/usr/bin/python

# +-----+-----+---------+------+---+---Pi 3A+-+---+------+---------+-----+-----+
# | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
# +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
# |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
# |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
# |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
# |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
# |     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
# |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
# |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
# |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
# |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
# |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
# |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
# |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
# |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
# |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
# |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
# |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
# |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
# |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
# |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
# |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
# +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
# | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
# +-----+-----+---------+------+---+---Pi 3A+-+---+------+---------+-----+-----+
 # spitest.py
# A brief demonstration of the Raspberry Pi SPI interface, using the Sparkfun
# Pi Wedge breakout board and a SparkFun Serial 7 Segment display:
# https://www.sparkfun.com/products/11629

import time
import spidev
import RPi.GPIO as GPIO

# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 0
CSN = 22

WHO_AM_I_REG  = 0x0F

#[r/w][ms][adress 6:0]
#ms is auto increment

#data rate & power mode register
CTRL_REG1 = 0x20
CTRL_REG2 = 0x21
#resulotion select register
CTRL_REG4 = 0x23

OUT_X_L_REG = 0x28
OUT_X_H_REG = 0x29
OUT_Y_L_REG = 0x2A
OUT_Y_H_REG = 0x2B
OUT_Z_L_REG = 0x2C
OUT_Z_H_REG = 0x2D

#100 hz low power on 
CTRL_REG1_VALUE = 0b01110111
CTRL_REG2_VALUE = 0b10000000
CTRL_REG4_VALUE = 0b00001000

#first data register


def readRegister(address, cont):
    cmdByte  = address
    cmdByte  |= ( 1 << 7)
    
    if(cont == 1):
        cmdByte |= (0<<6)
    else:
        cmdByte |= (1<<6)
    GPIO.output(CSN, 0)
    spi.xfer2([cmdByte])
    DO = spi.xfer2([0])
    GPIO.output(CSN, 1)
    return DO
    
    
def writeRegister(address, value):
    cmdByte  = address
    
    GPIO.output(CSN, 0)
    spi.xfer2([cmdByte])
    DO = spi.xfer2([value])
    GPIO.output(CSN, 1)
    return 
    
def lReadConvertedAccelDataHR():
    
    sumDataX = 0
    sumDataY = 0
    sumDataZ = 0
    outputData = [X,Y,Z]
    
    
    for axes in outputData:
        if(axes == X):
            outputData[axes] = (readRegister(OUT_X_L_REG,1)[0] + (readRegister(OUT_X_H_REG,1)[0] << 8))>>4
        elif(axes == Y):
            outputData[axes] = (readRegister(OUT_Y_L_REG,1)[0] + (readRegister(OUT_Y_H_REG,1)[0] << 8))>>4
        elif(axes == Z):
            outputData[axes] = (readRegister(OUT_Z_L_REG,1)[0] + (readRegister(OUT_Z_H_REG,1)[0] << 8))>>4
        
        
        
        if(outputData[axes] > fullscale/2):
            outputData[axes] -= fullscale
        outputData[axes] *= -1
       
        #outputData[axes] = evenwichtStand - outputData[axes]
        #if(outputData[axes] < -(amplitude)):
        #    outputData[axes] = -amplitude - (outputData[axes] + amplitude)
        #elif(outputData[axes] > amplitude):
        #    outputData[axes] = amplitude - (outputData[axes] - amplitude)
        
    return outputData

#The sensetivity of the sensor is calculated by pointing an axes of interested to the earth and in the opposite direction
#after doing this subtract the larger number by the smaller number and divide it by 2. This will be 1 g
#       X
#       ( 222 - 88 ) /2    =    67
#       Y
#       ( 215 - 85 ) /2    =    65
#       Z
#       ( 1030 - -1048 ) / 2   =    1039
#

nRecordSamples  = (10)

fullscale = (4096)


X               = (0)
Y               = (1)
Z               = (2)

if __name__ == '__main__':
    # Enable SPI
    spi = spidev.SpiDev()
    GPIO.setmode(GPIO.BCM)


    # Open a connection to a specific bus and device (chip select pin)
    spi.open(bus, device)
    GPIO.setup(CSN, GPIO.OUT)

    spi.max_speed_hz = 115200
    
    # Set SPI speed and mode
    spi.mode = 0
    GPIO.output(CSN, 1)
    
    #open file
    dataFile = open("data/acceldata.csv", "w")
    i = 0
    
    reg1output = [0]
    reg2output = [0]
    reg4output = [0]
    
    clkId = time.CLOCK_REALTIME
    currentTime = 0
    
    while(reg1output[0] != CTRL_REG1_VALUE or reg4output[0] != CTRL_REG4_VALUE):
        writeRegister(CTRL_REG1, CTRL_REG1_VALUE)
        writeRegister(CTRL_REG2, CTRL_REG2_VALUE)
        writeRegister(CTRL_REG4, CTRL_REG4_VALUE)
        
        reg1output = readRegister(CTRL_REG1,0)
        reg2output = readRegister(CTRL_REG2,0)
        reg4output = readRegister(CTRL_REG4,0)
        print(f"register 1 value = {reg1output}")
        print(f"register 2 value = {reg2output}")
        print(f"register 4 value = {reg4output}")
    
    # Turn on one segment of each character to show that we can
    # address all of the segments
    try:
        startTime = time.clock_gettime(clkId)
        sumDataX = 0
        sumDataY = 0
        sumDataZ = 0
        lastOutputData = 0
        while i < 3000:
            outputData = lReadConvertedAccelDataHR()
            currentTime = time.clock_gettime(clkId)
            print(outputData)
            dataFile.write("%f,%f,%f,%f\r\n" % (currentTime - startTime, outputData[X], outputData[Y], outputData[Z]))
            
            time.sleep(0.01)
            i = i + 1
            
    except KeyboardInterrupt:
        GPIO.cleanup()
# Clear display again
