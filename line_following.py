import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import cv2
import picamera
from dronekit import *
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1)

def circle(duration):
    vehicle.channels.overrides['1'] = 1200
    time.sleep(duration)
    vehicle.channels.overrides['1'] = 1500
    time.sleep(0.5)

def move_right(duration):
    vehicle.channels.overrides['3'] = 1500
    vehicle.channels.overrides['1'] = 1800
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100
    vehicle.channels.overrides['1'] = 1500
    time.sleep(0.5)

def move_forward(duration):
    vehicle.channels.overrides['3'] = 1500
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100
    time.sleep(0.5)

def move_left(duration):
    vehicle.channels.overrides['3'] = 1500
    vehicle.channels.overrides['1'] = 1200
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100
    vehicle.channels.overrides['1'] = 1500
    time.sleep(0.5)

camera = picamera.PiCamera()
camera.resolution = (192,108)
camera.framerate = 20
camera.zoom = (0.1, 0.1, 0.6, 0.1)
rawCapture = PiRGBArray(camera,size=(192,108))

time.sleep(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	cv2.imshow('img',image)
	key = cv2.waitKey(1) & 0xFF
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray,(5,5),0)
	ret,thresh1 = cv2.threshold(blur,100,255,cv2.THRESH_BINARY_INV)
	mask = cv2.erode(thresh1, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	contours, hierarchy = cv2.findContours(mask.copy(),1,cv2.CHAIN_APPROX_NONE)

	if len(contours) > 0:
		c = max(contours, key = cv2.contourArea)
		M = cv2.moments(c)
		cx = int(M['m10']/M['m00'])

		if cx >= 150:
			GPIO.output(12, GPIO.LOW)
			GPIO.output(21, GPIO.HIGH)
			print("Поворот вправо")
			move_right(0.5)

		if cx < 150 and cx > 40:
			GPIO.output(12, GPIO.HIGH)
			GPIO.output(21, GPIO.HIGH)
			print("Движение прямо")
			move_forward(0.5)

		if cx <= 40:
			GPIO.output(12, GPIO.HIGH)
			GPIO.output(21, GPIO.LOW)
			print("Поворот влево")
			move_left(0.5)

	if key == ord("q"):
            break
	rawCapture.truncate(0)

GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(21, GPIO.LOW)

GPIO.cleanup()
