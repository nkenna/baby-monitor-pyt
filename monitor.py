#!/usr/bin/env python
# code developed by Steinacoz Platform: www.steinacoz.com
# facebook page: steinacoz
# Baby Monitoring System based on Raspberry Pi by Maleshesh Markus Johnes

import RPi.GPIO as GPIO
import sys
import serial
from time import sleep, strftime, time
import time
import Adafruit_DHT
from sensor import Pulsesensor


# pubnub imports
from pubnub.pubnub import PubNub
# from pubnub.pubnub import pubnub
from pubnub.callbacks import SubscribeCallback
from pubnub.enums import PNOperationType, PNStatusCategory, PNReconnectionPolicy
from pubnub.pnconfiguration import PNConfiguration

print('library import')
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
import time
import random

# Import the ADS1x15 module.
import Adafruit_ADS1x15

p = Pulsesensor()
p.startAsyncBPM()

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

from numpy import interp

import subprocess
import multiprocessing



# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)



# Initialize library.
disp.begin()

# Clear display.
print('start display')
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
font = ImageFont.load_default()



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


temp_sensormodel = Adafruit_DHT.AM2302  # setup am2302 sensor
temp_sensorpin = 4  # setup am2302 sensor pin

pin_networkLed = 23
pin_PIR = 24
pin_buzzer = 25
MIC = 26

GPIO.setup(pin_buzzer , GPIO.OUT)
GPIO.setup(pin_networkLed , GPIO.OUT)
GPIO.setup(pin_PIR, GPIO.IN)  # PIR 1
GPIO.setup(MIC, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

GAIN = 1

BP = 0

lastTime = int(time.time()*1000)

# pubnub channels

MODE = 'MODE'
SENSOR_DATA = 'SENSOR_DATA'
SENSORS_TEMP = 'SENSORS_TEMP'
SENSORS_HUMID = 'SENSORS_HUMID'
FEEDBACKS = 'FEEDBACKS'
MUSIC = 'MUSIC'
PHOTO = 'PHOTO'  
NOISE = 'NOISE'
SENSORS_PULSE = 'SENSORS_PULSE'
BATTERY = 'BATTERY'

sensors_Data = 'get data'

music_on = 'music on'
music_off = 'music off'

take_photo = 'take photo'

#feedbacks
publish_success = 'publish success'
mode_change_auto = 'auto mode'
mode_change_manual = 'manual mode'

noise_count = 0
sound = 0
Signal  = 0



# initialize pubnub
pnconfig = PNConfiguration()
pnconfig.subscribe_key = '*****************'
pnconfig.publish_key = '*******************'
pnconfig.reconnect_policy = PNReconnectionPolicy.LINEAR

pubnub = PubNub(pnconfig)




# detecting motion
MOTION_PIR = 'MOTION DETECTED'

class SecurityMode():
    securityStatus = False
    
    def setSecurityStatus(self, status):
        self.securityStatus = status
        
    def getSecurityStatus(self):
        return self.securityStatus
    
    
sm = SecurityMode()
print(sm.securityStatus)

class SoundMode():
    sound = 0
    
    def setSound(self, ss):
        self.sound = ss
        
    def getSound(self):
        return self.sound
    
    
so = SoundMode()
print(so.sound)

def micISR(ev=None):
	print ("voice in...")
	analogVal = adc.read_adc(0, gain=1)
	print ('res = %d' % analogVal)
	s = interp(analogVal,[0,32767],[0,100])
	print('percent sound from 0: ' + str(s))
	so.setSound(s)
	print('percent sound from class: ' + str(so.getSound()))
	#draw.text((x, top+32), "Sound: " + str(sound) + "%",  font=font, fill=255)


def MOTION(pin_PIR):
  if GPIO.input(pin_PIR):
      print('motion  detected')
      if sm.securityStatus:
          GPIO.output(pin_buzzer, GPIO.LOW)
      else:
          GPIO.output(pin_buzzer, GPIO.HIGH)
      draw.text((x, top+25), 'motion  detected',  font=font, fill=255)
      pubnub.publish().channel(SENSOR_DATA).message(MOTION_PIR).async(my_publish_callback)
     # pubnub.publish().channel(SENSORS_PULSE).message(BP).async(my_publish_callback)
      pubnub.publish().channel(NOISE).message(int(so.getSound())).async(my_publish_callback)
  else:
      print('motion detection ended')
      draw.text((x, top+25), 'motion  detection ends',  font=font, fill=255)
      GPIO.output(pin_buzzer, GPIO.LOW)

GPIO.add_event_detect(pin_PIR, GPIO.BOTH, callback=MOTION)
GPIO.add_event_detect(MIC, GPIO.FALLING, callback=micISR)

  

  

def createNumber():
    num = random.randint(1, 5)
    return num
  


def buzz(pitch, duration):
    sec = random.randint(1, 5)
    if(pitch==0):
        sleep(duration)
        return
    
    period = sec / pitch     #in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
    delay = period / 2     #calcuate the time for half of the wave  
    cycles = int(duration * pitch)   #the number of waves to produce is the duration times the frequency
  
    for i in range(cycles):    
          GPIO.setup(pin_buzzer , GPIO.OUT)
          GPIO.output(pin_buzzer, True)   #set pin 18 to high
          sleep(delay)    #wait with pin 18 high
          GPIO.output(pin_buzzer, False)    #set pin 18 to low
          sleep(delay)    #wait with pin 18 low
          
def play(num):
    if num == 1:
        pitches=[2,330,392,5,106, 9,9,56,9]
        duration=0.1

        for p in pitches:
            buzz(p, duration)
            sleep(duration *0.5)

        for p in reversed(pitches):
            buzz(p, duration)
            sleep(duration *0.5)
            
    elif num == 2:
        pitches=[267,330,392,5,106, 9,9,56,9, 900, 567, 78]
        duration=random.random()

        for p in pitches:
            buzz(p, duration)
            sleep(duration *0.5)

        for p in reversed(pitches):
            buzz(p, duration)
            sleep(duration *0.5)
            
    elif num == 3:
            pitches=[2,330,392,57,106, 988,119,256,879]
            duration=0.1

            for p in pitches:
                buzz(p, duration)
                sleep(duration *0.5)

            for p in reversed(pitches):
                buzz(p, duration)
                sleep(duration *0.5)
                
    elif num == 4:
            pitches=[267,300,392,56,106, 900,19,56,9]
            duration=0.1

            for p in pitches:
                buzz(p, duration)
                sleep(duration *0.5)

            for p in reversed(pitches):
                buzz(p, duration)
                sleep(duration *0.5)
                
    elif num == 5:
              pitches=[200,330,392,5,106, 19,9,569,9]
              duration=0.1

              for p in pitches:
                  buzz(p, duration)  
                  sleep(duration *0.5)

              for p in reversed(pitches):
                  buzz(p, duration)
                  sleep(duration *0.5)           

#pubnub publish callback
def my_publish_callback(envelope, status):
    # Check whether request successfully completed or not
    if not status.is_error():
        pass 
        #pubnub.publish().channel(FEEDBACKS).message(publish_success).async(my_publish_callback)
    else:
        pass  # Handle message publish error. Check 'category' property to find out possible issue
       

class MySubscribeCallback(SubscribeCallback):
    def status(self, pubnub, status):
        if status.operation == PNOperationType.PNSubscribeOperation \
                or status.operation == PNOperationType.PNUnsubscribeOperation:
            
            if status.category == PNStatusCategory.PNConnectedCategory:
                GPIO.output(pin_networkLed, GPIO.HIGH)
                pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
                pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)
               # pubnub.publish().channel(SENSORS_PULSE).message(BP).async(my_publish_callback)
                pubnub.publish().channel(NOISE).message(int(so.getSound())).async(my_publish_callback)
                print(status.category)
                
            elif status.category == PNStatusCategory.PNReconnectedCategory:
                GPIO.output(pin_networkLed, GPIO.HIGH)
                pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
                pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)
                #pubnub.publish().channel(SENSORS_PULSE).message(BP).async(my_publish_callback)
                pubnub.publish().channel(NOISE).message(int(so.getSound())).async(my_publish_callback)
                print(status.category)
                
            elif status.category == PNStatusCategory.PNDisconnectedCategory:
                print(status.category)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNUnexpectedDisconnectCategory:
                print(status.category)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNAccessDeniedCategory:
                print(status.category)
                
            elif status.category == PNStatusCategory.PNNetworkIssuesCategory:
                print(status.category)
                pubnub.reconnect()
                
            elif status.category == PNStatusCategory.PNTimeoutCategory:
                print(status.category)
                pubnub.reconnect()    
            else:
                print(status.category)
                pubnub.reconnect()
        elif status.operation == PNOperationType.PNSubscribeOperation:
            if status.is_error():
                print(status.operation)
                pubnub.reconnect()
            else:
                print(status.operation)
                # Heartbeat operation was successful
        else:
            print(status.operation)
            pubnub.reconnect()
            # Encountered unknown status type
            
            
    def message(self, pubnub, message):
        print(message.channel)
        print(message.message)
        
        if message.channel == PHOTO:
            if message.message == take_photo:
                cmd = "raspistill -vf -o /home/pi/Pictures/image" + str(random.randint(1, 999999)) + ".jpeg"
                subprocess.call(cmd, shell=True)
        
        if message.channel == MUSIC:
            if message.message == music_on:
                play(createNumber())
            if message.message == music_off:      
                GPIO.output(pin_buzzer, GPIO.LOW) 
        
        if message.channel == MODE:
            if message.message == MODE_AUTO:
                sm.securityStatus = True
                sm.setSecurityStatus(sm.securityStatus)
                pubnub.publish().channel(FEEDBACKS).message(mode_change_auto).async(my_publish_callback)
                print(message.message)
                
            if message.message == MODE_MANUAL:
                sm.securityStatus = False
                sm.setSecurityStatus(sm.securityStatus)
                pubnub.publish().channel(FEEDBACKS).message(mode_change_manual).async(my_publish_callback)
                
              
        if message.message == sensors_Data:
            pubnub.publish().channel(SENSORS_TEMP).message(read_temp_humidity()[1]).async(my_publish_callback)
            pubnub.publish().channel(SENSORS_HUMID).message(read_temp_humidity()[0]).async(my_publish_callback)

            

    
    def presence(self, pubnub, presence):
        pass  # handle incoming presence data
    

pubnub.add_listener(MySubscribeCallback())
pubnub.subscribe().channels(SENSOR_DATA).execute()
pubnub.subscribe().channels(SENSORS_TEMP).execute()
pubnub.subscribe().channels(SENSORS_HUMID).execute()
pubnub.subscribe().channels(FEEDBACKS).execute()
pubnub.subscribe().channels(MUSIC).execute()
pubnub.subscribe().channels(PHOTO).execute()
pubnub.subscribe().channels(BATTERY).execute()
pubnub.subscribe().channels(SENSORS_PULSE).execute()
pubnub.subscribe().channels(NOISE).execute()



# function to read temperature and humidity
def read_temp_humidity():
    sleep(3)
    humidity, temperature = Adafruit_DHT.read_retry(temp_sensormodel, temp_sensorpin)
    print(humidity)
    print(temperature)
    return humid_value(humidity), temp_value(temperature) 



# get temp value
def temp_value(temp):
    temp = '{0:0.1f}'.format(float(temp))
    return temp

# get humidity value
def humid_value(humid):
    humid = '{0:0.1f}'.format(float(humid))
    return humid


def continueScript(sound):
    b = 1
    try:
        while True:
            #sound = sound
            b =+ 1
            noise_count = 0
      

       
           # print('Sensor value: ' + str(Signal))
            print('Raw Noise Value: ' + str(sound))
           # print('Noise Value: ' + str(calcSound(sound)))
          
            
            print('percent sound: ' + str(sound))
            
           
            
            if sound > 65:
                play(createNumber())
                GPIO.output(pin_buzzer, GPIO.HIGH)
                pubnub.publish().channel(NOISE).message(sound).async(my_publish_callback)
                noise_count += 1
            else:
                GPIO.output(pin_buzzer, GPIO.LOW)
             
            if noise_count > 10:
                 play(createNumber())
                 pubnub.publish().channel(NOISE).message(sound).async(my_publish_callback)
             
            
                    
                
                # Draw a black filled box to clear the image.
            draw.rectangle((0,0,width,height), outline=0, fill=0)
            
            # Write two lines of text.

            draw.text((x, top),       "IOT Baby Monitor",  font=font, fill=255)   
            draw.text((x, top+8), "Temp: " + str(read_temp_humidity()[1]),  font=font, fill=255)
            draw.text((x, top+16), "humidity: " + str(read_temp_humidity()[0]),  font=font, fill=255)
            draw.text((x, top+32), "Sound: " + str(int(so.getSound())) + "%",  font=font, fill=255)
           
    

            # Display image.
            disp.image(image)
            disp.display()
      
            sleep(1)    

         
    except KeyboardInterrupt:
        GPIO.cleanup()
        scriptProcess.terminate()
        p.stopAsyncBPM()
        sleep(1)
        sys.exit(1)        

    

 
scriptProcess = multiprocessing.Process(target=continueScript(so.getSound()), args=()).start()
