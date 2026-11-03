#!/usr/bin/env python3
"""水弹波箱火力进程（由 gimbal 主程序 fork/exec，经 stdin 管道收命令）。

命令协议（stdin，每行一条）:
    '1'  开火（按住扳机，进入 GRIP/GAP 节拍循环）
    '0'  停火（立即拉低）
    'q'  退出（停火 + GPIO.cleanup）
EOF（父进程崩溃/退出）-> 自动停火退出。

节拍参数（环境变量，单位 ms）:
    FIRE_GRIP_MS  每次通电脉宽，默认 80
    FIRE_GAP_MS   两发之间低电平间隔，默认 120
    FIRE_VERBOSE  1 = 每次脉冲切换打印时间戳到 stderr（无示波器时看节拍）

用法: python3 fire_worker.py <BOARD 引脚号>  （默认 12）
"""

import os
import select
import sys
import time

import Jetson.GPIO as GPIO


def main() -> int:
    pin = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("FIRE_PIN", "12"))
    grip_s = float(os.environ.get("FIRE_GRIP_MS", "80")) / 1000.0
    gap_s = float(os.environ.get("FIRE_GAP_MS", "120")) / 1000.0
    verbose = os.environ.get("FIRE_VERBOSE", "0") == "1"

    def log(msg: str) -> None:
        if verbose:
            print(f"[fire] {time.monotonic():.3f} {msg}", file=sys.stderr, flush=True)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    log(f"init pin={pin} grip={grip_s * 1000:.0f}ms gap={gap_s * 1000:.0f}ms")

    hold = False     # 扳机按住
    on = False       # 当前脉冲处于高电平
    next_t = 0.0

    try:
        while True:
            # 非阻塞地看一眼 stdin（5ms 超时，兼顾节拍精度与响应）
            r, _, _ = select.select([sys.stdin], [], [], 0.005)
            if r:
                line = sys.stdin.readline()
                if not line:
                    break                      # EOF：父进程没了
                cmd = line.strip()
                if cmd == "1":
                    hold = True
                    log("trigger ON")
                elif cmd == "0":
                    hold = False
                    if on:                     # 松手立即停，不等脉冲走完
                        GPIO.output(pin, GPIO.LOW)
                        on = False
                    log("trigger OFF")
                elif cmd == "q":
                    break

            now = time.monotonic()
            if on and now >= next_t:           # 高电平脉宽结束 -> 拉低
                GPIO.output(pin, GPIO.LOW)
                on = False
                next_t = now + gap_s
                log("pulse LOW")
            elif hold and not on and now >= next_t:  # 间隔结束 -> 拉高
                GPIO.output(pin, GPIO.HIGH)
                on = True
                next_t = now + grip_s
                log("pulse HIGH")
    finally:
        GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        log("cleanup, exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
