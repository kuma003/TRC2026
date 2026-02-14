import time
import RPi.GPIO as GPIO
import readchar

# モーターピン定義
left_motor_forward = 6
left_motor_backward = 5
right_motor_forward = 13
right_motor_backward = 19

# PWMオブジェクト
pwm_left_forward = None
pwm_left_backward = None
pwm_right_forward = None
pwm_right_backward = None

# 初期速度（0-100%）
speed = 20


# GPIOの設定
def setup():
    global pwm_left_forward, pwm_left_backward, pwm_right_forward, pwm_right_backward

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # ピンをセットアップ
    GPIO.setup(left_motor_forward, GPIO.OUT)
    GPIO.setup(left_motor_backward, GPIO.OUT)
    GPIO.setup(right_motor_forward, GPIO.OUT)
    GPIO.setup(right_motor_backward, GPIO.OUT)

    # PWMオブジェクトを作成（周波数100Hz）
    pwm_left_forward = GPIO.PWM(left_motor_forward, 100)
    pwm_left_backward = GPIO.PWM(left_motor_backward, 100)
    pwm_right_forward = GPIO.PWM(right_motor_forward, 100)
    pwm_right_backward = GPIO.PWM(right_motor_backward, 100)

    # PWM信号を0%で開始
    pwm_left_forward.start(0)
    pwm_left_backward.start(0)
    pwm_right_forward.start(0)
    pwm_right_backward.start(0)

    stop()


# 前進
def forward():
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(speed)
    pwm_right_backward.ChangeDutyCycle(0)
    print(f"前進（速度: {speed}%）")


# 後退
def backward():
    return
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_left_backward.ChangeDutyCycle(speed)
    pwm_right_forward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(speed)
    print(f"後退（速度: {speed}%）")


# 左折
def turn_left():
    pwm_left_forward.ChangeDutyCycle(speed / 3.0 + 2.0)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(speed)
    pwm_right_backward.ChangeDutyCycle(0)
    print(f"左折（速度: {speed}%）")


# 右折
def turn_right():
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(speed / 3.0 + 2.0)
    pwm_right_backward.ChangeDutyCycle(0)
    print(f"右折（速度: {speed}%）")


# 停止
def stop():
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(0)
    print("停止")


# 速度を上げる
def speed_up():
    global speed
    if speed < 100:
        speed += 2
        if speed > 100:
            speed = 100
    print(f"速度アップ: {speed}%")
    return speed


# 速度を下げる
def speed_down():
    global speed
    if speed > 0:
        speed -= 2
        if speed < 0:
            speed = 0
    print(f"速度ダウン: {speed}%")
    return speed


# メインプログラム
def main():
    setup()
    print("モータPWM制御開始")
    print("w: 前進, s: 後退, a: 左折, d: 右折, x: 停止")
    print("+: 速度アップ, -: 速度ダウン, q: 終了")
    print(f"現在の速度: {speed}%")

    try:
        current_direction = None
        while True:
            key = readchar.readkey()

            if key == "w":
                current_direction = "forward"
                forward()
            elif key == "s":
                current_direction = "backward"
#                backward()
            elif key == "a":
                current_direction = "left"
                turn_left()
            elif key == "d":
                current_direction = "right"
                turn_right()
            elif key == "x":
                current_direction = None
                stop()
            elif key in ["+", "="]:  # '+'か'='で速度アップ
                speed_up()
                # 現在の方向を維持
                if current_direction == "forward":
                    forward()
                elif current_direction == "backward":
                    #backward()
                    pass
                elif current_direction == "left":
                    turn_left()
                elif current_direction == "right":
                    turn_right()
            elif key == "-":  # '-'で速度ダウン
                speed_down()
                # 現在の方向を維持
                if current_direction == "forward":
                    forward()
                elif current_direction == "backward":
                    #backward()
                    pass
                elif current_direction == "left":
                    turn_left()
                elif current_direction == "right":
                    turn_right()
            elif key == "q":
                stop()
                break

    except KeyboardInterrupt:
        pass
    finally:
        # PWM信号を停止
        pwm_left_forward.stop()
        pwm_left_backward.stop()
        pwm_right_forward.stop()
        pwm_right_backward.stop()
        GPIO.cleanup()
        print("プログラム終了")


if __name__ == "__main__":
    main()
