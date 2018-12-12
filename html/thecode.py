'''
LiTeRo, the (Li)ttle (Te)lepresense (Ro)bot 

This python sketch is used in conjunction with phraser.php and litero.html to make a little RasPi-powered rover scoot around and do stuff.

Features included by default:
	- MJPEG Streamer
		- 160-175deg Wide Angle Camera Module with White/IR LED modules For Raspberry Pi (Banggood)
	- RPi.GPIO
		- HG7881 (L9110) Dual Channel Motor Driver (GPIO Pins 17, 18, 22, 27)
		- Navigation LED (2N2222 mod for White/IR LED module) (GPIO Pin 23)
	- eSpeak Text-To-Speech
		- LM4871 audio amp conected to GPIO pin #13 (see: PWM1 on GPIO #13 (ALT0) for RasPi Zero W) 
	- MinIMU-9 (I2C x2)
	- ADS1115 ADC (Refer to Adafruit tutorials for setup) (I2C)
	- sSMTP
	
Basic operation:
	This program reads from a short text file in RAM (/dev/shm/input.txt) at about 60Hz with parameters 
	that are set at will by a PHP script. The PHP script is being called as the client(s) send in new 
	requests (ex: ./phraser.php?ID=546939&CT=1521479887886&M1=90&M2=90). The client in return gets an XML 
	response that this Python program creates in RAM (/dev/shm/output.txt) along with an ongoing live view 
	from the bot using MJPEG-Streamer. 

More information can be found on thestuffwebuild.com
- Michael H.
'''

############## Begin Library Imports and User Setup ##############

# General purpose imports
import RPi.GPIO as GPIO
import os
import time
import math
import re
#import soul #Removed due to strange behavour.

# Import and config for MinIMU-9 comunication
import smbus 
bus = smbus.SMBus(1) #i2c buss these are on. 
LIS3MDL_M_ADDRESS		= 0x1E #LIS3MDL address
LSM6DS33_M_ADDRESS		= 0x6b #LSM6DS33 address
headingOffset = 180 # imu mount orientation offset. 
tippedOverFwdAngle = 60 # Max angle to determin if bot is propped up on its end.
tippedOverBakAngle = -60 # Min angle to determin if bot is propped up on its end.
useIMU = True #Change this to 'False' if you do not want to use an IMU

# Import and config for the ADS1x15 module.
import Adafruit_ADS1x15
adc = Adafruit_ADS1x15.ADS1115() # Create an ADS1115 ADC (16-bit) instance.
vCal = 1.062 # Manual calibration factor for volt readings:
aCal = 0.994 # Manual calibration factor for amp readings:
ADCAlpha = 0.1 # for running avgerage. Set to 1 to disable.
lowBattThresh = 3.5 #battery threshold for warnings
critBattThresh = 3.3 #shutdown if volts is below this for >15sec
useADC = True #Change this to 'False' if you do not want to use an ADC

# Audio and speech settings for systemSpeak() function
volume = 100
speechPitch=50 #pitch 0-99 (default = 50)
speechSpeed= 175 #speed 80wpm to 500wpm (default = 175wpm)
speechVoice= "en+m1" #try "en-f2". default = "en+m1"

# Email to deliever initialization and distress emails to:
ssmtp_To_email = "thestuffwebuild@gmail.com"

# Do you want to send an email on boot?
sendEmailOnBoot = False

# Motor behavour
motorAlpha = 0.3 # Accelration parameter. Applies only if enableMotorAccel = True
motorMin = 15 #minimm PWM% of motor. prevents motionless pwm noise. anlyhting less turns motors off.
enableMotorAccel = True	
gracefulStop = True
reverseMotorOutputs = True #use this if you need to reverse all motor outputs.
reverseLeftRight = True #use this if you need to swap valuse for left and right motors.

# Aspect ratio of your rpi camera sensor (4/3 = 1.33333)
camAspectRatio = 1.33333 

# Debug level for terminal output
debugLevel = 0 # 0 = only important stuff, higher is more stuff. -1 disables all output (may de-debug).
def printLog(text, level=0):
	if level <= debugLevel:
		print(str("%.2f" % time.time()) + " - " + text)		

############## End Library Imports and User Setup ##############

############## Begin GPIO Setup ############### 

# Define GPIO pins
MC1A = 17 #motor controller pin 1A connection to this GPIO pin
MC1B = 18 #motor controller pin 1B connection to this GPIO pin
MC2A = 27 #motor controller pin 2A connection to this GPIO pin 
MC2B = 22 #motor controller pin 2B connection to this GPIO pin
NAVLED_PIN = 23 #led trigger pin connection to this GPIO pin
# LM4871 audio amp conected to GPIO pin #13 (see: PWM1 on GPIO #13 (ALT0) for RasPi Zero W) 
# SERVO_PIN = 19 #optional servo signal pin connected this GPIO pin

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(MC1A,GPIO.OUT) #motor controller pin 1A
GPIO.setup(MC1B,GPIO.OUT) #motor controller pin 1B
GPIO.setup(MC2A,GPIO.OUT) #motor controller pin 2A
GPIO.setup(MC2B,GPIO.OUT) #motor controller pin 2B
a = GPIO.PWM(MC1A, 1000)    # create an object p for PWM on port 25 at 50 Hertsz 
b = GPIO.PWM(MC1B, 1000)    # create an object p for PWM on port 25 at 50 Hertz 
c = GPIO.PWM(MC2A, 1000)    # create an object p for PWM on port 25 at 50 Hertz 
d = GPIO.PWM(MC2B, 1000)    # create an object p for PWM on port 25 at 50 Hertz 
a.start(0)             # start the PWM on 0 percent duty cycle (off)
b.start(0)             # start the PWM on 0 percent duty cycle (off)
c.start(0)             # start the PWM on 0 percent duty cycle (off)
d.start(0)             # start the PWM on 0 percent duty cycle (off)

GPIO.setup(NAVLED_PIN,GPIO.OUT) #navled pin
GPIO.output(NAVLED_PIN,False) # Switch off pin NAVLED_PIN

#GPIO.setup(SERVO_PIN,GPIO.OUT) # servo pwm pin
#pwm=GPIO.PWM(SERVO_PIN, 50) # servo pwm rate
#pwm.start(0) # start pwm at zero

############## End GPIO Setup ##############

############## Begin Global vars ##############
# Flag lets a routine run once
runFirstLoop = True	

#Default camera specs on boot
oldcamY = 144
oldcamFPS = 15
oldcamQ = 10

#mag offsets pulled from file
MagXoffs=0
MagYoffs=0
MagZoffs=0

#IMU placeholder
xAngle=0
yAngle=0
heading=0

# smoothed motor values
m1Real=0
m2Real=0

#previous values
oldm1 = 0
oldm2 = 0
oldnavLED = 0
oldclientTime = 0
oldservoPos = 0
oldspeakPhr = ""

#past timing values
lastMoveTime = 0
lastDataOutTime = 0
preclientTime = 0
lastheartbeatTime = 0
lastLBattCheckTime = 0
lastLBattSpeakTime = 0
lastLBattEmailTime = 0
lastRSSICheckTime = 0
lastServoUpdate = 0

#low battery flags
lowBatt = 0 
lowBattStrikes = 0

#this is sent to client and used for other things
camHASH=0
RSSI=0
RAamps=0
RAvolts=0

#user id control.
IDBase=["C137"]
UserID = 0
lastPurgeTime=0

#camera control
ranOnce = False
cameraKilled=0
lastIdleCheckTime=0

#open or make an input file in RAM
try:
	with open("/dev/shm/input.txt", "r") as filestream:
		printLog("Input file found",1)
except:
	os.system("echo 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, > /dev/shm/input.txt")
	os.system("sudo chown www-data:www-data /dev/shm/input.txt")
	printLog("Input file created",1)

############## End Global Vars ##############

############## Begin Function Defines ##############

#Text To Speech function
def systemSpeak(phrase, wait=0): #if second var is == 1, system will wait for phrase to finish before continuing (useful for initialization params)
	espeakCmds = "-v" + str(speechVoice)+" -p"+str(speechPitch)+" -s"+str(speechSpeed)
	
	printLog("Now speaking: "+str(phrase), 1) 
	
	if wait == 0:
		os.system("espeak "+str(espeakCmds)+" \""+str(phrase)+"\" 2>/dev/null&")
	else:
		os.system("espeak "+str(espeakCmds)+" \""+str(phrase)+"\" 2>/dev/null")
	#more espeak commands here: http://espeak.sourceforge.net/commands.html

# Magnetometer stuff
magInit = 0
def readMag(axis=''):
	# initialize mag if first run
	global magInit
	if magInit == 0:
		magInit = 1
		
		# initMag -- Sets up the magnetometer to begin reading.
		LIS3MDL_CTRL_REG1_M		= 0x20
		LIS3MDL_CTRL_REG2_M		= 0x21
		LIS3MDL_CTRL_REG3_M		= 0x22
		LIS3MDL_CTRL_REG4_M		= 0x23
		LIS3MDL_REG_CTL_1_TEMP_EN 	= 0x80
		LIS3MDL_REG_CTL_2_RESET 	= 0x04
		
		#User Register Reset Function
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG2_M, LIS3MDL_REG_CTL_2_RESET)
		#Temperature Sensor Enabled
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, LIS3MDL_REG_CTL_1_TEMP_EN )
		#Ultra High Performance Mode Selected for XY Axis
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, 0x60)
		#Ultra High Performance Mode Selected for Z Axis
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG4_M, 0x0C)
		#Output Data Rate of 80 Hz Selected
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, 0x1C)
		#Continous Conversion Mode,4 wire interface Selected 
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG3_M, 0x00)
		# 16 guass Full Scale 
		bus.write_byte_data(LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG2_M, 0x60)
		
	if axis=='x' or axis == 'X':
		# Reading the  Magnetometer X-Axis Values from the Register 
		LIS3MDL_OUT_X_L_M = 0x28
		LIS3MDL_OUT_X_H_M = 0x29
		Mag_l = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_X_L_M)
		Mag_h = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_X_H_M)
		Mag_total = (Mag_l | Mag_h <<8)
		return Mag_total  if Mag_total < 32768 else Mag_total - 65536
	
	elif axis=='y' or axis == 'Y':
		# Reading the  Magnetometer Y-Axis Values from the Register
		LIS3MDL_OUT_Y_L_M = 0x2A
		LIS3MDL_OUT_Y_H_M = 0x2B
		Mag_l = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Y_L_M)
		Mag_h = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Y_H_M)
		Mag_total = (Mag_l | Mag_h <<8)
		return Mag_total  if Mag_total < 32768 else Mag_total - 65536
	
	elif axis=='z' or axis == 'Z':
		# Reading the  Magnetometer Z-Axis Values from the Register
		LIS3MDL_OUT_Z_L_M = 0x2C
		LIS3MDL_OUT_Z_H_M = 0x2D
		Mag_l = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Z_L_M)
		Mag_h = bus.read_byte_data(LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Z_H_M)
		Mag_total = (Mag_l | Mag_h <<8)
		return Mag_total  if Mag_total < 32768 else Mag_total - 65536
	
	else:
		printLog("Function readMag() incorrectly called.")
		return 0

# Calibrate mag. can be called any time
def magCalibration(): #reads mag for 30 sec and writes avgeraged max and min offsets to file	
	systemSpeak("Calibrating magnetometer. Please rotate me on all axis for 30 seconds.", 1)
	
	calStartTime = time.time()
	lastMagReadTime = 0
	
	xList = []
	yList = []
	zList = []
	
	while time.time() - calStartTime < 30: 
		if time.time() - lastMagReadTime > 0.0125: #limit to 80hz 0.0125
		
			#test if reading is 
			xList.extend([int(readMag('x'))])
			yList.extend([int(readMag('y'))])
			zList.extend([int(readMag('z'))])
		
			lastMagReadTime = time.time()
			
	#collect min and max		
	magXmax = max(xList)
	magYmax = max(yList)
	magZmax = max(zList)
	
	magXmin = min(xList)
	magYmin = min(yList)
	magZmin = min(zList)
	
	# calculate offsets (use n -= nOffset)
	MagXoffs = (magXmin + magXmax) /2 ;
	MagYoffs = (magYmin + magYmax) /2 ;
	MagZoffs = (magZmin + magZmax) /2 ;
	
	#write to calibration file to write.
	try:
		with open("/var/www/html/mag_calibration.txt", "w") as magCal_out:
			magCal_out.write(str(MagXoffs)+","+str(MagYoffs)+","+str(MagZoffs)+",(X_offset | Y_Offset | Z_Offset)")
			printLog("Magnetometer calibration written to file.")
	except:
		printLog("Can not save magnetometer calibration data.")
	
	#clear lists (for ram?)
	xList = []
	yList = []
	zList = []
	
	systemSpeak("Magnetometer calibration complete. Applying offsets.",1)
	magApplyOffsets()

# Apply mag offsets or callibrate and apply if no cal file present
def magApplyOffsets():
	global MagXoffs
	global MagYoffs
	global MagZoffs
	#read mag offsets file and save variables
	try:
		with open("/var/www/html/mag_calibration.txt", "r") as magCal_in:
			for line in magCal_in:
				currentline = line.split(",")
				try:
					MagXoffs = int(currentline[0])		
					MagYoffs = int(currentline[1])	
					MagZoffs = int(currentline[2])	
					printLog("Magnetometer offsets applied as (X | Y | Z): "+str(MagXoffs)+","+str(MagYoffs)+","+str(MagZoffs))	
				except:
					printLog("Bad magnetometer calibration file. No offsets applied.")
					
	except:
		printLog("No magnetometer calibration file found. Starting magCalibration().")
		magCalibration()
if useIMU: magApplyOffsets()

# Accelrometer stuff
accInit = 0
def readAcc(axis=''):
	# initialize acc if first run
	global accInit
	if accInit == 0:
		accInit = 1

		# initAcc -- Sets up the Accnetometer to begin reading.
		LSM_CTRL1_XL          = 0x10  # [+] Acceleration sensor control
		LSM_CTRL2_G           = 0x11  # [+] Angular rate sensor (gyroscope) control
		LSM_CTRL3_C           = 0x12  # [+] Device/communication settings

		bus.write_byte_data(LSM6DS33_M_ADDRESS, LSM_CTRL3_C, 0x00)

		# Accelerometer
		# 1.66 kHz / +/- 4g
		# 01011000b
		bus.write_byte_data(LSM6DS33_M_ADDRESS, LSM_CTRL1_XL, 0x58)

		#turn off gyro, we don't need it.
		bus.write_byte_data(LSM6DS33_M_ADDRESS, LSM_CTRL2_G, 0x00)


	if axis=='x' or axis == 'X':
		# Reading the  Accnetometer X-Axis Values from the Register
		LSM6DS33_OUT_X_L_M		= 0x28
		LSM6DS33_OUT_X_H_M		= 0x29
		Acc_l = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_X_L_M)
		Acc_h = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_X_H_M)
		Acc_total = (Acc_l | Acc_h <<8)
		return Acc_total  if Acc_total < 32768 else Acc_total - 65536

	elif axis=='y' or axis == 'Y':
		# Reading the  Accnetometer Y-Axis Values from the Register
		LSM6DS33_OUT_Y_L_M		= 0x2A
		LSM6DS33_OUT_Y_H_M		= 0x2B
		Acc_l = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_Y_L_M)
		Acc_h = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_Y_H_M)
		Acc_total = (Acc_l | Acc_h <<8)
		return Acc_total  if Acc_total < 32768 else Acc_total - 65536

	elif axis=='z' or axis == 'Z':
		# Reading the  Accnetometer Z-Axis Values from the Register
		LSM6DS33_OUT_Z_L_M		= 0x2C
		LSM6DS33_OUT_Z_H_M		= 0x2D
		Acc_l = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_Z_L_M)
		Acc_h = bus.read_byte_data(LSM6DS33_M_ADDRESS,LSM6DS33_OUT_Z_H_M)
		Acc_total = (Acc_l | Acc_h <<8)
		return Acc_total  if Acc_total < 32768 else Acc_total - 65536

	else:
		printLog("Function readAcc() incorrectly called.")
		return 0

# Gyro stuff
gyroInit = 0
def readGyro(axis=''):
	# Gyroscope dps/LSB for 1000 dps full scale
	GYRO_GAIN       = 0.035
	LSM6DS33_ADDR   = 0x6b
	
	# initialize gyro if first run
	global gyroInit
	if gyroInit == 0:
		gyroInit = 1
		LSM6DS33_CTRL2_G           = 0x11  # Angular rate sensor (gyroscope) control
		bus.write_byte_data(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x58)
		
		
	if axis=='x' or axis == 'X':
		#read gyro x
		LSM6DS33_OUTX_L_G          = 0x22  # Gyroscope pitch axis (X) output, low byte
		LSM6DS33_OUTX_H_G          = 0x23  # Gyroscope pitch axis (X) output, high byte
		Gyro_l = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTX_L_G)
		Gyro_h = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTX_H_G)
		Gyro_total = (Gyro_l | Gyro_h <<8)
		Gyro_total=Gyro_total   if Gyro_total < 32768 else Gyro_total - 65536
		return Gyro_total * GYRO_GAIN
	
	elif axis=='y' or axis == 'Y':
		#read gyro y
		LSM6DS33_OUTY_L_G          = 0x24  # Gyroscope roll axis (Y) output, low byte
		LSM6DS33_OUTY_H_G          = 0x25  # Gyroscope roll axis (Y) output, high byte
		Gyro_l = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTY_L_G)
		Gyro_h = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTY_H_G)
		Gyro_total = (Gyro_l | Gyro_h <<8)
		Gyro_total=Gyro_total   if Gyro_total < 32768 else Gyro_total - 65536
		return Gyro_total * GYRO_GAIN
	
	elif axis=='z' or axis == 'Z':
		#read gyro z
		LSM6DS33_OUTZ_L_G          = 0x26  # Gyroscope yaw axis (Z) output, low byte
		LSM6DS33_OUTZ_H_G          = 0x27  # Gyroscope yaw axis (Z) output, high byte
		Gyro_l = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTZ_L_G)
		Gyro_h = bus.read_byte_data(LSM6DS33_ADDR,LSM6DS33_OUTZ_H_G)
		Gyro_total = (Gyro_l | Gyro_h <<8)
		Gyro_total=Gyro_total   if Gyro_total < 32768 else Gyro_total - 65536
		return Gyro_total * GYRO_GAIN
	
	else:
		printLog("Function readGyro() incorrectly called.")
		return 0

# Temporarly disables GPIO if needed
def idle():
	a.ChangeDutyCycle(0)  
	b.ChangeDutyCycle(0)  
	c.ChangeDutyCycle(0)  
	d.ChangeDutyCycle(0)  
	GPIO.output(NAVLED_PIN,False)

# Email notification function for boot-up status and alarms	
lastEmailSentTime = 0 
def sendAnEmail():
	idle() # this could take a while. lets stop the motors for now.
	global lastEmailSentTime
	if time.time() - lastEmailSentTime > 60: #Don't send more than one email a minute
		printLog("Writing email...",1)
		bodytext = ""
		
		subtext = "Your little robot friend has something to say. - "+str(time.time())
		
		if lowBatt == 1:
			subtext = "HELP! Your little robot friend is in Distress! - "+str(time.time())
			bodytext += "My battery is pretty low now. I might shut down soon...\n\n" 
		
		bodytext += "Let me tell you about myself...\n\n"
		
		#write ADC vars
		bodytext += "Battery Volts: " + str(round(volts,2)) + "V\n" 
		bodytext += "Battery Amps: " + str(round(amps,2)) + "A\n\n" 
		
		# Write IMU data
		bodytext += "Accelorometer X angle (horizon=90deg): " + str(round(xAngle,2)) + "deg\n" 
		bodytext += "Accelorometer Y angle (horizon=90deg): " + str(round(yAngle,2)) + "deg\n" 
		bodytext += "Magnetometer heading: " + str(round(heading,2)) + "deg\n\n" 

		#write cpu temp
		netStat = os.popen("/opt/vc/bin/vcgencmd measure_temp").read()
		bodytext += "CPU " + str(netStat) + "\n"
		
		#write uptime
		netStat = os.popen("uptime").read()
		bodytext += "Uptime: " + str(netStat) + "\n" 

		#write lan status
		bodytext += "RSSI: " + str(RSSI) + "\n" 
		netStat = os.popen("ifconfig wlan0").read()
		bodytext += "LAN Status:\n" + str(netStat) + "\n\n"

		os.system("printf \"To: "+str(ssmtp_To_email)+"\nFrom: My little robot's name\nSubject: "+str(subtext)+"\n\n"+str(bodytext)+"\n\" | ssmtp -t")

		printLog("Email sent!")
		lastEmailSentTime = time.time()
	else:
		printLog("Email not sent. Please wait a bit before sending another.")

# Find a value between two strings
def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""

# Servo controller (not used by default)
def setServoAngle(angle): # an angle <0 turns off servo to avoid jitter.
	if angle >= 0:
		duty = angle / 18 + 2
		GPIO.output(SERVO_PIN, True)
		pwm.ChangeDutyCycle(duty)
	else: 
		GPIO.output(SERVO_PIN, False)
		pwm.ChangeDutyCycle(0)

############## End Function Defines ##############

################ Main Loop ################

while True:	
	
	#adc sampling
	if useADC:
		a0 = adc.read_adc(0, gain=1)
		a1 = adc.read_adc(1, gain=1)
		#a2 = adc.read_adc(2, gain=1) 
		#a3 = adc.read_adc(3, gain=1)
	
		# ADC volts math
		volts = (a0 / 32767.0 ) * 4.096
		volts = volts * 2 * vCal
		RAvolts = ADCAlpha * volts + (1-ADCAlpha) * RAvolts
		printLog("volts: " + str(RAvolts), 4)
		
		# ADC amps math
		aonevolts = (a1 / 32767.0 ) * 4.096
		aonevolts = aonevolts * 2 * aCal
		amps = (aonevolts - 2.5) / 0.185
		RAamps = ADCAlpha * amps + (1-ADCAlpha) * RAamps
		printLog("amps: " + str(RAamps), 4)
		
		# Monitor battery voltage alarms every 10 seconds
		if time.time() - lastLBattCheckTime > 10:
			# Low battery
			if critBattThresh < RAvolts < lowBattThresh and amps < 0.1: # if volts in threshold, AND if not charging (negitive is charging)
				lowBatt = 1
				#have XML status say low battery if below 3.3v 
				if time.time() - lastLBattSpeakTime > 20: # only speak once every 20sec 
					systemSpeak("Low battery. Please return me to my charging station.")
					printLog("Low battery.")
					lastLBattSpeakTime = time.time()
				if time.time() - lastLBattEmailTime > 300: # Only send an email warning every 5 minutes.
					sendAnEmail()
					lastLBattEmailTime = time.time()
			else:
				lowBatt = 0
			
			#Critical battery
			if 1 < RAvolts <= critBattThresh: 
				lowBattStrikes += 1
				if lowBattStrikes >= 3:
					lastEmailSentTime = 0 #force email, it will be the last.
					sendAnEmail()
					systemSpeak("My battery, is dead. Shutting down.")
					printLog("I am dead.")
					idle()
					time.sleep(60)
					os.system("shutdown")
					time.sleep(86400)
			else:
				lowBattStrikes = 0
			lastLBattCheckTime = time.time()
		
	# Monitor rssi every 5 seconds for XML output.
	if time.time() - lastRSSICheckTime > 5:
		RSSI = os.popen("awk 'NR==3{print $4}' /proc/net/wireless").read()
		RSSI = RSSI.translate(None, '.')
		lastRSSICheckTime = time.time()
		
	# IMU sampling
	if useIMU:
		# Read magnetometer and apply offsets
		Magx = readMag('x') - MagXoffs
		Magy = readMag('y') - MagYoffs
		Magz = readMag('z') - MagZoffs
		
		# Calculate mag heading
		heading = 180 * math.atan2(Magy,Magx)/3.14159
		heading -= headingOffset # imu mount orientation offset.
		if heading < 0: heading += 360
		
		# Read accelerometer
		accX = readAcc('x')
		accY = readAcc('y')
		accZ = readAcc('z')
		
		#Calculate x and y acc angle
		xAngle = 180 * math.atan2(accZ,accX)/3.14159
		xAngle -= 90 
		if xAngle < -180: xAngle += 360	
		yAngle = 180 * math.atan2(accZ,accY)/3.14159
		yAngle -= 90
		if yAngle < -180: yAngle += 360
		
		#read gyro DPS
		gyroX = readGyro('x')
		gyroY = readGyro('y')
		gyroZ = readGyro('z')
		
		#calculate absolute angles
		#TBd
		
		#anti-tip countermeasures
		#tbd (will have to go after we read client input?)
	
	# Read main input file from ram about 60 times a second
	if time.time() - lastMoveTime > 0.016 :
		reading = True
		while reading == True:
			with open("/dev/shm/input.txt", "r") as filestream:
				for line in filestream:
					line = str(line)
					#print line
					#try to get data, if it is bad (someone's writing to it), use last good.
					try:
						try: m1 = int(find_between(line, "M1=", " - "))		# Motor 1 speed/pwm (0-100)
						except: m1=None
						try: m2 = int(find_between(line, "M2=", " - "))			# Motor 2 speed/pwm (0-100)
						except: m2=None
						try: camY = int(find_between(line, "CAY=", " - "))		# stream Y dimension
						except: camY = None
						try: camFPS = int(find_between(line, "CAF=", " - "))		# stream fps
						except: camFPS = None
						try: camQ = int(find_between(line, "CAQ=", " - "))			# stream quality (compression)
						except: camQ = None
						try: navLED = int(find_between(line, "LED=", " - "))				# navigation LED on/off
						except: navLED = None
						try: speakPhr = str(find_between(line, "TTS=", " - "))		# espeak/soundboard phrase select
						except: speakPhr = ""
						try: clientTime = int(find_between(line, "CT=", " - "))			# heartbeat
						except: clientTime = None
						try: servoPos = int(find_between(line, "SRV=", " - "))			# servo position command 
						except: servoPos = None
						try: UserID = int(find_between(line, "ID=", " - "))			#unique random number from client
						except: UserID = None
						
						reading = False #we got data, so break
						
					except: 
						m1 = oldm1 
						m2 = oldm2 
						camY = oldcamY
						camFPS = oldcamFPS
						camQ = oldcamQ
						navLED = oldnavLED 
						speakPhr = oldspeakPhr 
						clientTime = oldclientTime 
						servoPos= oldservoPos 
						printLog("Corrupt input file. Using stale values.")
						reading = False
						
		lastMoveTime = time.time()
	
	#if input vars are blank, it means we don't want to update them, so use old values.
	m1 = oldm1 if m1 == "" else m1
	m2 = oldm2 if m2 == "" else m2
	camY = oldcamY if camY == "" else camY
	camFPS = oldcamFPS if camFPS == "" else camFPS
	camQ = oldcamQ if camQ == "" else camQ
	navLED = oldnavLED if navLED == "" else navLED
	#speakPhr = oldspeakPhr if speakPhr == "" else speakPhr #A blank string from client is used to reset
	clientTime = oldclientTime if clientTime == "" else clientTime
	servoPos = oldservoPos if servoPos == "" else servoPos
	#UserID = oldUserID if UserID == "" else UserID=UserID
	
	if clientTime == 0:
		preclientTime = clientTime
		
	# If clienttime hasn't updated in deathTimeout seconds, assume client(s) has froze/left and stop moving
	deathTimeout=8
	if clientTime == preclientTime:
		if time.time() - lastheartbeatTime > deathTimeout: #no running client for this many seconds
			m1 = 0
			m2 = 0
			navLED = 0
			printLog("No client heart beat!",3) #client has no heartbeat
			#lastDataOutTime = time.time() # dont bother writing XML data to output file.
			
			del IDBase[:]
			UserID = 0
	else:
		lastheartbeatTime=time.time()
	preclientTime = clientTime
	
	# If no clients for 1 minute, turn off camera.	
	if len(IDBase) == 0:
		if time.time() - lastIdleCheckTime > 60 and cameraKilled == 0:
			os.system("killall mjpg_streamer")
			printLog("mjpg_streamer killed by inactivity.")
			cameraKilled = 1;
	else:
		lastIdleCheckTime = time.time()
		if cameraKilled == 1:
			ranOnce = False
			cameraKilled = 0
	 
	# Extend client's id to user database
	if UserID in IDBase:
		#I know you
		pass
	else:
		if UserID:
			IDBase.extend([UserID])
			
	# If inputs are out of range, use old values
	if m1 <-100 or m1 >100:
		m1 = oldm1
	if m2 <-100 or m2 >100:
		m2 = oldm2
	if camY < 30 or camFPS < 2 or camQ < 1 or camQ > 100 or camFPS > 60 or camY > 1944:
		camY = oldcamY
		camFPS = oldcamFPS
		camQ = oldcamQ
	if navLED < 0:
		navLED = oldnavLED
	if servoPos <0 or servoPos >180:
		servoPos = oldservoPos
	
	# If camera settings changed, push to camera
	if oldcamY != camY or oldcamFPS != camFPS or oldcamQ != camQ or ranOnce == False:
		ranOnce = True
		camX = camY * camAspectRatio
		
		# Here python is issuing terminal commands		
		os.system("killall mjpg_streamer")
		os.system("/home/pi/mjpg-streamer/mjpg-streamer-experimental/mjpg_streamer -b -o \"/home/pi/mjpg-streamer/mjpg-streamer-experimental/output_http.so\" -i \"/home/pi/mjpg-streamer/mjpg-streamer-experimental/input_raspicam.so -rot 180 -x "+ str(int(round(camX))) + " -y " + str(camY) + " -quality " + str(camQ) + " -fps " + str(camFPS) + "\"") 
		
		printLog("Updated camera settings.")
		
		camHASH = time.time()
		
		# Update old vars
		oldcamY = camY
		oldcamFPS = camFPS
		oldcamQ = camQ
	
	# Locomotion control
	if True:
		# If bot is propped up on its end, disable motor acceleration so user can try to right the bot with fast fwd/back movements.
		if xAngle>= tippedOverFwdAngle or xAngle  <= tippedOverBakAngle:
			Alpha = 1
		else:
			Alpha = motorAlpha
		
		# Motor acceleration maths (if enabled). Compute a running average, but quick stop
		if enableMotorAccel == True:
			if gracefulStop != True:
				if m1Real < 0 <= m1 or m1Real > 0 >= m1:
					m1Real = 0
				if m2Real < 0 <= m2 or m2Real > 0 >= m2:
					m2Real = 0
					
			m1Real = Alpha * m1 + (1-Alpha) * m1Real
			m2Real = Alpha * m2 + (1-Alpha) * m2Real
		else:
			m1Real = m1
			m2Real = m2
			
		#Basic user config options
		if reverseLeftRight == True:
			m1Out = m2Real
			m2Out = m1Real
		else:
			m1Out = m1Real
			m2Out = m2Real
		if reverseMotorOutputs == True:
			m1Out *= -1
			m2Out *= -1
			
		# Operate motor #1 GPIO
		if m1Out > motorMin:
			a.ChangeDutyCycle(round(abs(m1Out)))  
			b.ChangeDutyCycle(0)  
		elif m1Out < -motorMin:
			a.ChangeDutyCycle(0)  
			b.ChangeDutyCycle(round(abs(m1Out)))  
		else: # if your motor controller supports breaking, set these two high?
			a.ChangeDutyCycle(0)  
			b.ChangeDutyCycle(0)
			
		# Operate motor #2 GPIO
		if m2Out > motorMin:
			c.ChangeDutyCycle(round(abs(m2Out)))  
			d.ChangeDutyCycle(0)  
		elif m2Out < -motorMin:
			c.ChangeDutyCycle(0)  
			d.ChangeDutyCycle(round(abs(m2Out)))  
		else: # if your motor controller supports breaking, set these two high?
			c.ChangeDutyCycle(0)  
			d.ChangeDutyCycle(0) 
			
		printLog("Motor commands: ("+str(round(m1Out))+", "+str(round(m2Out))+")",3)
		
		oldm1 = m1
		oldm2 = m2
	
	# Update navigation LED
	if navLED == 1:
		GPIO.output(NAVLED_PIN,True) # Switch on pin NAVLED_PIN (auto)
	else:
		GPIO.output(NAVLED_PIN,False) # Switch on pin NAVLED_PIN (off)
	oldnavLED = navLED
	
	# Say espeak phrases 
	#only english is allowed and max length
	re.sub(r'[^a-zA-Z.,?!0-9\s]', '', speakPhr)
	speakPhr = speakPhr[:140] 
	if oldspeakPhr != speakPhr:
		oldspeakPhr = speakPhr
		if speakPhr != "":
			systemSpeak(speakPhr)
	
	# Update servo position (disabled due to feature creeping, also disabled in xml output)
	'''
	# At this point in the program you have a value servoPos that is between 0 and 180.
	# setServoAngle(servoPos) can ba called to send the servo to that angle.
	# Remeber to disable the servo to avoid jitter with setServoAngle(-1)
	if oldservoPos != servoPos:
		oldservoPos = servoPos
		setServoAngle(servoPos)
		lastServoUpdate = time.time()
	# Give servo about 1 second to reach desitnation before turning off.
	if time.time() - lastServoUpdate > 1:
		setServoAngle(-1)
	'''
	
	# Print XML data to a file in ram two times a second
	if time.time() - lastDataOutTime > 0.5:
		with open("/dev/shm/output.txt", "w") as out_file:
			#XML headers will be taken care of in php
			#out_file.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
			#out_file.write("<data>")
			out_file.write("<Vo>"+str(round(RAvolts,2))+"</Vo>")
			out_file.write("<Am>"+str(round(RAamps,2))+"</Am>")
			out_file.write("<He>"+str(round(heading,1))+"</He>")
			out_file.write("<SS>"+str(RSSI)+"</SS>")
			out_file.write("<LED>"+str(navLED)+"</LED>")
			out_file.write("<Us>"+str(len(IDBase))+"</Us>")
			out_file.write("<CH>"+str(camHASH)+"</CH>") # image hash
			#out_file.write("<SV>"+str(servoPos)+"</SV>") # current servo position
			#out_file.write("</data>")
		printLog("XML data written to output file.",2)
		lastDataOutTime = time.time()

	# Clear list of clients every 0.5 minutes. This purges timed-out clients. Active clients will continuew to send their ID and current time.
	if time.time() - lastPurgeTime > 30:
		del IDBase[:]
		lastPurgeTime=time.time()

	# This operates only once and only after above program is complete.
	if runFirstLoop == True:
		runFirstLoop = False
		
		if sendEmailOnBoot == True:
			sendAnEmail()
		
		#set volume (this can be called later if need be.)
		os.system("amixer -q sset 'PCM' "+str(volume)+"% 2> /dev/null &")
		
		#speak ipv4 address
		netIP = os.popen("hostname -I").read().split()
		systemSpeak("Greetings. My local IP address is: "+str(netIP[0]))
		
		printLog("LiTeRo Ready.")




	
