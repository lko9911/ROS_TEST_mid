import lgpio
import asyncio

# 핀 정의
IN1 = 17
IN2 = 27
ENA = 18

# 핀 모드 설정 및 PWM 초기화
def setup_motor():
    h = lgpio.gpiochip_open(0)  # /dev/gpiochip0 열기
    lgpio.gpio_claim_output(h, IN1, 0)
    lgpio.gpio_claim_output(h, IN2, 0)
    lgpio.gpio_claim_output(h, ENA, 0)

    # PWM 시작 (채널 0, dutyCycle은 0~1 사이)
    lgpio.tx_pwm(h, ENA, 1000, 0)  # 1kHz, duty 0%
    return h

# 모터 작동 함수
async def run_motor(h, duration=5, power=80):
    print(f"💧 수분 펌프 작동: {duration}초")

    lgpio.gpio_write(h, IN1, 1)
    lgpio.gpio_write(h, IN2, 0)
    lgpio.tx_pwm(h, ENA, 1000, power / 100)  # dutyCycle: 0~1

    await asyncio.sleep(duration)

    lgpio.gpio_write(h, IN1, 0)
    lgpio.gpio_write(h, IN2, 0)
    lgpio.tx_pwm(h, ENA, 1000, 0)

    print("💧 수분 펌프 정지 완료")

# GPIO 정리
def cleanup_motor(h):
    lgpio.gpiochip_close(h)


async def main():
    h = setup_motor()
    try:
        await run_motor(h, duration=5, power=80)
    finally:
        cleanup_motor(h)

if __name__ == "__main__":
    asyncio.run(main())