# motor_control.py
import RPi.GPIO as GPIO
import asyncio

# 핀 정의
IN1 = 17
IN2 = 27
ENA = 22

# GPIO 초기화 함수
def setup_motor():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(ENA, GPIO.OUT)
    pwm = GPIO.PWM(ENA, 1000)
    pwm.start(0)
    return pwm

# 모터 작동 함수
async def run_motor(pwm, duration=5, power=80):
    print(f"💧 수분 펌프 작동: {duration}초")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(power)
    await asyncio.sleep(duration)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)
    print("💧 수분 펌프 정지 완료")

# GPIO 정리
def cleanup_motor(pwm):
    pwm.stop()
    GPIO.cleanup()
