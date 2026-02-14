import RPi.GPIO as GPIO
import time

heating_wire = 10
print("start")

GPIO.setmode(GPIO.BCM)
GPIO.setup(heating_wire, GPIO.OUT)

print("fire")

GPIO.output(heating_wire, GPIO.HIGH)

time.sleep(3)

GPIO.output(heating_wire, GPIO.LOW)

print("done")
