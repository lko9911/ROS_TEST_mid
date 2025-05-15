# pollination_motor_gpiozero.py
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from signal import pause
import asyncio

# 핀 설정 (BCM 번호)
IN1 = DigitalOutputDevice(17)
IN2 = DigitalOutputDevice(27)
ENA = PWMOutputDevice(18, frequency=1000)

# 모터 작동 함수 (비동기)
async def run_motor(duration=5, power=0.8):
    print(f"💧 수분 펌프 작동: {duration}초 (출력: {int(power * 100)}%)")

    # 정방향 회전
    IN1.on()
    IN2.off()
    ENA.value = power  # 0.0 ~ 1.0

    await asyncio.sleep(duration)

    # 정지
    #ENA.value = 0
    ENA.off()
    IN1.off()
    IN2.off()

    await asyncio.sleep(duration+10)

    print("💧 수분 펌프 정지 완료")

# 메인
async def main():
    await run_motor(duration=5, power=0.75)

# 실행
if __name__ == "__main__":
    asyncio.run(main())