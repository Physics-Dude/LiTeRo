# LiTeRo
**LiTeRo, the (Li)ttle (Te)lepresense (Ro)bot  **

This python sketch is used in conjunction with phraser.php and litero.html to make a little RasPi-powered rover scoot around and do stuff.

**Features included by default:**
	- MJPEG Streamer
		- 160-175deg Wide Angle Camera Module with White/IR LED modules For Raspberry Pi (Banggood)
	- RPi.GPIO
		- HG7881 (L9110) Dual Channel Motor Driver (GPIO Pins 17, 18, 22, 27)
		- Navigation LED (2N2222 mod for White/IR LED module) (GPIO Pin 23)
    - Servo control!
	- eSpeak Text-To-Speech
		- LM4871 audio amp conected to GPIO pin #13 (see: PWM1 on GPIO #13 (ALT0) for RasPi Zero W) 
	- MinIMU-9 (I2C x2)
	- ADS1115 ADC (Refer to Adafruit tutorials for setup) (I2C)
	- sSMTP
	
**Basic operation:**
	theprogram.py reads from a short text file in RAM (/dev/shm/input.txt) at about 60Hz with parameters 
	that are set at will by a PHP script. The PHP script is being called as the client(s) send in new 
	commands (ex: ./phraser.php?ID=546939&CT=1521479887886&M1=90&M2=90). The client in return gets an XML 
	response that this Python program creates in RAM (/dev/shm/output.txt) along with an ongoing live view 
	from the bot using MJPEG-Streamer. 
  
  LiTeRo by default will speak its local ip address on bootup. It will also guide you through the magnetic compass calibration when mag_calibration.txt is removed before bootup.

**Things to prepare your Raspberry Pi or Raspberry Pi Zero**
		- For the Raspberry Pi Zero, you should enable audio out on one of the GPIO pins in order to use TTS (see: PWM1 on GPIO #13 (ALT0) for RasPi Zero W). 
    - Make sure Apache, PHP, Python, mjpeg-streamer, eSpeak, and the Adafruit_ADS1x15 Python library are installed for FULL functionality.
  
  
  
  **More information can be found on thestuffwebuild.com**
