from libs import detector as dc
import cv2
import time

detector = dc.detector()

detector.set_roi_img()

try:
    while True:
        detector.detect()
        if detector.is_detected:
            print(f"cone direction: {detector.cone_direction}")
        if detector.is_parachute_detected:
            print(f"parachute direction: {detector.parachute_direction}")

        if detector.is_reached:
            print("--------------------------------")
            print("cone reached!!")
            print("--------------------------------")
        time.sleep(0.5)
except Exception as e:
    print(e)
    exit(-1)
