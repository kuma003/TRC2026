import asyncio
import math
import websockets
from libs import BNO055
import serial
from libs.micropyGPS import MicropyGPS
import threading
import json
import time
import os
from pathlib import Path
from libs import detector as dc
import cv2
import base64


def load_env_file(env_path: Path) -> None:
    if not env_path.exists():
        print(f"env file not found: {env_path} (using defaults)")
        return

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].lstrip()
        if "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            continue

        if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")
        ):
            value = value[1:-1]

        os.environ.setdefault(key, value)


ENV_PATH = Path(__file__).with_name("server.env")
load_env_file(ENV_PATH)

port = int(os.getenv("SERVER_PORT", "8756"))
SERVER_URL = os.getenv("SERVER_URL", "10.156.221.98")
WS_SCHEME = os.getenv("WS_SCHEME", "ws")
URI = f"{WS_SCHEME}://{SERVER_URL}:{port}"
TARGET_LAT = float(os.getenv("TARGET_LAT", "38.266285"))
TARGET_LNG = float(os.getenv("TARGET_LNG", "140.855498"))
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "70"))  # 圧縮品質 (0-100)
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

bmx = BNO055.BNO055()
bmx.setUp()

lat = 0.0
lng = 0.0
gps_detect = 0
distance = 0.0
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]

detector = dc.detector()

detector.set_roi_img()

encoded_img_txt = None


def get_BMX055_data():
    global acc
    global gyro
    global mag
    global bmx

    try:
        acc = bmx.getAcc()
        gyro = bmx.getGyro()
        mag = bmx.getMag()
    except IOError as e:
        print(f"I/O error({e.errno}): {e.strerror} was occurred")
        bmx.__del__()  # remove instance
        bmx = BNO055.BNO055()
        bmx.setUp()


def calc_distance():
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)


async def send_data():
    global lat
    global lng
    global gps_detect

    counter = 0

    while True:
        try:
            print("Connecting to server")
            async with websockets.connect(URI) as websocket:
                while True:
                    get_BMX055_data()  # 関数を使用してセンサーデータを取得
                    calc_distance()
                    current_time = time.time_ns()
                    data = {
                        "time": current_time,
                        "lat": lat,
                        "lng": lng,
                        "acc": acc,
                        "gyro": gyro,
                        "mag": mag,
                        "azimuth": 0,
                        "angle": 0,
                        "distance": distance,
                        "gps_detect": gps_detect,
                    }
                    if counter % 10 == 0:
                        detect_cone()
                        data.update(
                            {
                                "img": encoded_img_txt,
                                "cone_direction": detector.cone_direction,
                                "cone_probability": detector.probability,
                                "cone_occupancy": detector.occupancy,
                                "cone_detected": detector.detected.tolist(),
                                "is_detected": bool(
                                    detector.is_detected
                                ),  # bool型に変換
                            }
                        )
                    counter += 1

                    data = json.dumps(data)
                    try:
                        await websocket.send(data)
                    except Exception as e:
                        print(f"データ送信中にエラー発生: {e}")
                        break  # 内部ループを抜ける
                    await asyncio.sleep(0.00001)  # 非同期で待機
        except websockets.exceptions.ConnectionClosedOK:
            print("接続が正常に閉じられました。1秒後に再接続します...")
            await asyncio.sleep(1)
        except (
            websockets.exceptions.ConnectionClosedError,
            ConnectionRefusedError,
        ) as e:
            print(f"Connection lost: {e}, retrying in 1 second...")
            await asyncio.sleep(1)
        except Exception as e:
            print(f"予期せぬエラーが発生しました: {e}")
            await asyncio.sleep(1)


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

        if lat == 0.0:
            gps_detect = 0
            print("None gnss value")
            continue
        gps_detect = 1


def detect_cone():
    global encoded_img_txt
    detector.detect_cone()
    _, buffer = cv2.imencode(".jpg", detector.input_img, ENCODE_PARAM)
    encoded_img_txt = base64.b64encode(buffer).decode("utf-8")
    # print(cone_direction, cone_probability, cone_occupancy)
    # if detector.is_detected:
    #     print(f"occupied: {detector.occupancy}")

    # if detector.is_reached:
    #     print("--------------------------------")
    #     print("cone reached!!")
    #     print("--------------------------------")


async def async_main():
    global lat, lng, gps_detect
    count = 0

    try:
        while True:
            count += 1
            print(f"async_main実行回数: {count}")
            print("lat: ", lat, "lng: ", lng)
            await asyncio.sleep(1)  # 非同期sleep
    except Exception as e:
        print(f"async_main内でエラーが発生: {e}")
        # エラー発生後も継続するため再帰呼び出し
        await async_main()


if __name__ == "__main__":
    # GPSのスレッドの立ち上げ
    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    # 非同期I/O
    loop = asyncio.get_event_loop()
    # 複数のコルーチンを同時に実行
    tasks = [send_data(), async_main()]
    loop.run_until_complete(asyncio.gather(*tasks))
