import serial
import time
import math
import threading
import datetime
import pigpio
import csv
import os
import cv2
import RPi.GPIO as GPIO

# import wiringpi as pi
from libs import BNO055
from libs import BMP085
from libs.micropyGPS import MicropyGPS
from libs import detector as dc
from picamera2 import Picamera2

# import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import sys


# 定数　上書きしない
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.260871333333334  # Goal座標
TARGET_LNG = 140.85392616666667
TARGET_ALTITUDE = 20
DATA_SAMPLING_RATE = 0.00001
ALTITUDE_CONST1 = 30
ALTITUDE_CONST2 = 5
HIGH = 1
LOW = 0
# Pin number
heating_wire = 10
M1A = 13  # Motor
M1B = 19
M4A = 6
M4B = 5


# 変数
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0  # from GPS sensor
lng = 0.0
alt = 0.0
maxAlt = 0.0
minAlt = 0.0
pres = 0.0
distance = 0
angle = 0.0
azimuth = 0.0
direction = 0.0
frequency = 50
phase = 0
gps_detect = 0
cone_direction = 0
cone_probability = 0
restTime = 0.0
diff_rot = 1
upside_down_Flag = 0  # judge the upside down by acc(bmx)
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1

bmx = BNO055.BNO055()
bmp = BMP085.BMP085()

nowTime = datetime.datetime.now()
fileName = "./log/testlog_" + nowTime.strftime(format="%Y-%m%d-%H%M%S") + ".csv"


def main():
    global phase
    global restTime
    global start
    global gps_detect
    # Flag
    searching_Flag = False
    camera_failed = False
    count_cone_lost = False
    # measure time
    time_start_searching_cone = 0
    time_searching_cone = 0
    time_camera_start = 0
    time_camera_detecting = 0
    time_phase2 = 0

    GPIO.setwarnings(False)
    Setup()
    phase = 0

    while True:
        if phase == 0:  # 投下
            print("phase0 : falling")
            start = time.time()
            while True:
                getBmxData()
                # print(fall)
                if fall > 30:
                    print("para released")
                    time.sleep(10)
                    break
                if time.time() - start > 5 * 60:  # ********  fix later **********
                    print("failed to detect falling")
                    break
                # time.sleep(0.1)
            phase = 2  # TODO: skip parachute separation. must be fixed later. change to phase 1 after test.

        elif phase == 1:  # パラ分離
            print("phase1 : remove para")
            print("fire")
            GPIO.output(heating_wire, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(heating_wire, GPIO.LOW)
            print("done")
            phase = 2

        elif phase == 2:  # パラ回避フェーズ
            print("phase2 : parachute detection")
            detector.detect_cone_flag = False  # parachute detection
            if detector.is_parachute_detected is None:
                try:
                    detector.detect()
                except Exception as e:
                    print("Camera detection failed: ", e)
                    camera_failed = True

            if not camera_failed and detector.is_parachute_detected:
                if time_phase2 == 0:
                    print("parachute detected.")
                    time_phase2 = time.time()
                else:
                    # run reversely for 3 seconds to get away from the parachute
                    if time.time() - time_phase2 > 3:
                        phase = 3
            else:
                phase = 3

        elif phase == 3:
            print("phase3 : GPS start")
            print("angle: ", angle)
            print("azimuth", azimuth)
            print("direction", direction)

            if camera_failed == False:
                if distance < 5.0:
                    phase = 4
            elif camera_failed == True:  # not using camera mode and get closer
                if distance < 1.0:
                    phase = 6  # goal

        elif phase == 4:
            print("phase4 : camera start")
            try:
                cone_detect()
            except Exception as e:
                print("Camera detection failed: ", e)
                camera_failed = True
                phase = 3
                continue  # skip the rest of phase 4 and go back to phase 3
            if searching_Flag == False:
                searching_Flag = True
                time_start_searching_cone = time.time()
            elif searching_Flag == True:
                time_searching_cone = time.time()
                # something is wrong with the camera
                if time_searching_cone - time_start_searching_cone >= 10:
                    camera_failed = True
                    searching_Flag = False
                    phase = 3  # restart GPS mode and get closer
            if cone_probability < 1:
                phase = 5

        elif phase == 5:
            print("phase5")
            time_camera_start = time.time()
            count_cone_lost = 0
            while True:
                try:
                    cone_detect()
                except Exception as e:
                    print("Camera detection failed: ", e)
                    camera_failed = True
                    phase = 3
                    break

                time_camera_detecting = time.time()

                if time_camera_detecting - time_camera_start >= 10:
                    if detector.is_detected == False:
                        count_cone_lost += 1
                        print("count_cone_lost", count_cone_lost)
                    if count_cone_lost >= 10:  # camera lost the cone for a long time
                        phase = 4  # restart at phase4
                        break

                if (
                    time_camera_detecting - time_camera_start >= 45
                ):  # camera has been detected the cone for a long time, but Goal has not been judged.
                    print("already reached and being stuck")
                    phase = 6
                    break
                if detector.is_reached:  # if rover reached cone
                    print("reached")
                    phase = 6
                    break

        elif phase == 6:
            print("phase6 : Goal")
            time.sleep(10000)
        time.sleep(0.1)


def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    global detector
    bmx.setUp()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(heating_wire, GPIO.OUT)

    with open(fileName, "a") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "MilliTime",
                "Phase",
                "AccX",
                "AccY",
                "AccZ",
                "GyroX",
                "GyroY",
                "GyroZ",
                "MagX",
                "MagY",
                "MagZ",
                "LAT",
                "LNG",
                "ALT",
                "Distance",
                "Azimuth",
                "Angle",
                "Direction",
                "Fall",
                "cone direction",
                "cone probability",
            ]
        )

    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    detector = dc.detector()
    detector.set_roi_img()

    print("Setup OK")


def getBmxData():  # get BMX data
    global acc
    global gyro
    global mag
    global fall
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
    # mag[1] = mag[1]
    # mag[2] = mag[2]
    fall = math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
    # for i in range(3):
    #    mag[i] = (mag[i] - calibBias[i]) / calibRange[i]


def getBmpData():
    global alt
    global pres
    alt = bmp.read_altitude()
    pres = bmp.read_pressure()

    alt = alt + 60  # calibration


def flying():  # 落下検知関数 :飛んでいるときはTrueを返し続ける
    # この関数は何回も繰り返されることを想定
    global maxAlt
    global minAlt

    if maxAlt < alt:
        maxAlt = alt
    if minAlt > alt:
        minAlt = alt
    subAlt = maxAlt - minAlt
    absAlt = abs(alt - minAlt)

    if subAlt > ALTITUDE_CONST1 and absAlt < ALTITUDE_CONST2:
        print("bmp : reached ground")
        time.sleep(10)
        return False

    else:
        True


def upside_down():
    global upside_down_Flag
    global acc
    if acc[0] > 0:
        upside_down_Flag = 1


def calibration():  # calibrate BMX raw data
    global calibBias
    global calibRange
    max = [0.0, 0.0, 0.0]
    min = [0.0, 0.0, 0.0]
    mag = bmx.getMag()
    max[0] = mag[0]
    max[1] = mag[1]
    # max[2] = mag[2]
    min[0] = mag[0]
    min[1] = mag[1]
    # min[2] = mag[2]

    complete = False
    while complete == False:
        before = currentMilliTime()
        after = before
        while (after - before) < CALIBRATION_MILLITIME:
            mag = bmx.getMag()
            if max[0] < mag[0]:
                max[0] = mag[0]
            elif min[0] > mag[0]:
                min[0] = mag[0]
            elif max[1] < mag[1]:
                max[1] = mag[1]
            elif min[1] > mag[1]:
                min[1] = mag[1]
            after = currentMilliTime()
        if (max[0] - min[0]) > 3 and (max[1] - min[1] > 3):
            print("calibration(): Complete!")
            complete = True
            time.sleep(1)
            calibBias[0] = (max[0] + min[0]) / 2
            calibBias[1] = (max[1] + min[1]) / 2

            calibRange[0] = (max[0] - min[0]) / 2
            calibRange[1] = (max[1] - min[1]) / 2
            print("calibBias", calibBias, "calibRange", calibRange)
            time.sleep(1)


def calcDistanceAngle():  # 距離・角度計算関数
    global distance
    global angle

    EARTH_RADIUS = 6378137.0
    dx = (
        math.radians(TARGET_LNG - lng)
        * EARTH_RADIUS
        * math.cos(math.radians(TARGET_LAT))
    )
    dy = math.radians(TARGET_LAT - lat) * EARTH_RADIUS
    distance = math.hypot(dx, dy)
    angle = 90 - math.degrees(math.atan2(dy, dx))
    angle %= 360.0


def calcAzimuth():  # 方位角計算用関数
    global azimuth

    azimuth = 90 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1  # 上のazimuthはCanSatからみた北の方位
    azimuth %= 360


def GPS_thread():  # GPSモジュールを読み、GPSオブジェクトを更新する
    global lat
    global lng
    global gps_detect

    s = serial.Serial("/dev/serial0", 115200)
    s.readline()  # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    gps = MicropyGPS(9, "dd")

    while True:
        sentence = s.readline().decode("utf-8")  # GPSデーターを読み、文字列に変換する

        if s.in_waiting > 64:  # バッファを削除
            s.reset_input_buffer()
        if sentence[0] != "$":  # 先頭が'$'でなければ捨てる
            continue
        for (
            x
        ) in (
            sentence
        ):  # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
        lat = gps.latitude[0]
        lng = gps.longitude[0]

        if lat > 0:
            gps_detect = 1
        elif lat == 0.0:
            gps_detect = 0


def cone_detect():
    global detector
    global cone_direction
    global cone_probability

    detector.detect_cone_flag = True  # raise flag
    detector.detect()
    try:
        cone_direction = 1 - detector.cone_direction
        cone_probability = detector.probability
    except:
        cone_direction = 0.5
        cone_probability = 10


def setData_thread():
    while True:
        getBmxData()
        calcDistanceAngle()
        calcAzimuth()
        set_direction()
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    currentMilliTime(),
                    round(phase, 1),
                    acc[0],
                    acc[1],
                    acc[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[0],
                    mag[1],
                    mag[2],
                    lat,
                    lng,
                    alt,
                    distance,
                    azimuth,
                    angle,
                    direction,
                    fall,
                    cone_direction,
                    cone_probability,
                ]
            )
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    global phase
    slow = 1.0  # Slow down the motor in camera mode
    GPIO.setmode(GPIO.BCM)

    #   pi.wiringPiSetupGpio()
    GPIO.setup(M1A, GPIO.OUT)
    GPIO.setup(M1B, GPIO.OUT)
    GPIO.setup(M4A, GPIO.OUT)
    GPIO.setup(M4B, GPIO.OUT)

    motor_right_f = GPIO.PWM(M1A, frequency)
    motor_right_b = GPIO.PWM(M1B, frequency)
    motor_left_f = GPIO.PWM(M4A, frequency)
    motor_left_b = GPIO.PWM(M4B, frequency)

    motor_right_f.start(0)
    motor_right_b.start(0)
    motor_left_f.start(0)
    motor_left_b.start(0)

    while True:
        if phase == 5:
            slow = 0.5
        else:  # camera mode
            slow = 1.0
        if direction == 360.0:  # stop
            motor_right_f.ChangeDutyCycle(0)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(0)
            motor_left_b.ChangeDutyCycle(0)
        elif direction == -360.0:  # forward
            motor_right_f.ChangeDutyCycle(40 * slow)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(40 * slow)
            motor_left_b.ChangeDutyCycle(0)
        elif direction == 400:  # counterclockwise rotate
            motor_right_f.ChangeDutyCycle(50)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(15)
            motor_left_b.ChangeDutyCycle(0)
        elif direction == -400.0:  # clockwise rotate
            motor_right_f.ChangeDutyCycle(15)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(50)
            motor_left_b.ChangeDutyCycle(0)
        elif direction > 0.0 and direction <= 180.0:  # left
            motor_right_f.ChangeDutyCycle(25)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(15)
            motor_left_b.ChangeDutyCycle(0)
        elif direction < 0.0 and direction >= -180.0:  # right
            motor_right_f.ChangeDutyCycle(15)
            motor_right_b.ChangeDutyCycle(0)
            motor_left_f.ChangeDutyCycle(25)
            motor_left_b.ChangeDutyCycle(0)


def set_direction():  # -180<direction<180  #rover move to right while direction > 0
    global detector
    global direction
    global phase
    global object_distance_Flag
    global upside_down_Flag

    if phase == 0:  # 投下
        direction = 360

    elif phase == 1:
        direction = 360

    elif phase == 2:  # キャリブレーション
        # direction = -400.0  # right
        if detector is None or detector.parachute_direction is None:
            direction = 360
        elif detector.parachute_direction > 0.5:
            direction = +400  # counterclockwise rotate
        else:
            direction = -400  # clockwise rotate

    elif phase == 3:
        direction = azimuth - angle
        direction %= 360
        if direction > 180:
            direction -= 360
        if abs(direction) < 5.0:
            direction = -360
    elif phase == 4:
        direction = -400.0

    elif phase == 5:
        if cone_direction > 0.7:
            direction = -180
        elif cone_direction <= 0.7 and cone_direction >= 0.3:
            direction = -360
        elif cone_direction < 0.3:
            direction = 180

    elif phase == 6:
        direction = 360
    elif phase == -1:
        for _ in range(4):
            direction = 500
            time.sleep(0.3)
            direction = 600
            time.sleep(0.3)
        direction = 90
        time.sleep(2)
        direction = -360
        time.sleep(2)
    elif phase == -2:
        direction = 700
        time.sleep(3)
        upside_down_Flag = 0


if __name__ == "__main__":
    main()
    time.sleep(100)
