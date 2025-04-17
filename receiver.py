import signal
import struct
import sys
import time

import pigpio
import serial


class Receiver:
    def __init__(self):
        try:
            self.ser = serial.Serial("/dev/serial0", 115200, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Error: シリアルポートを開けませんでした -> {e}")
            exit(1)

        self.packet_size = struct.calcsize("4h")
        self.data = None
        self.running = True

        self.steering = 1500
        self.throttle = 1000
        self.brake = 1948

        self.steering_pin = 19
        self.throttle_pin = 12
        self.brake_pin = 13
        self.drive_pin = 16
        self.reverse_pin = 26

        self.pi = pigpio.pi()
        self.pi.set_mode(self.steering_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.throttle_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.brake_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.drive_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.reverse_pin, pigpio.OUTPUT)

        self.pi.set_servo_pulsewidth(self.steering_pin, self.steering)
        self.pi.set_servo_pulsewidth(self.throttle_pin, self.throttle)
        self.pi.set_servo_pulsewidth(self.brake_pin, self.brake)
        self.pi.write(self.drive_pin, 0)
        self.pi.write(self.reverse_pin, 0)

    def run(self):
        while self.running:
            try:
                self.data = self.ser.read(self.packet_size)

                if len(self.data) != self.packet_size:
                    print("Error: 不完全なデータを受信")
                    continue

                (self.steering, self.throttle, self.brake, self.state) = struct.unpack(
                    "4h", self.data
                )

                print(f"{self.steering}, {self.throttle}. {self.brake}, {self.state}")
                self.steering = min(1835, max(self.steering, 1139))
                self.throttle = min(2000, max(self.throttle, 1000))
                self.brake = min(1948, max(self.brake, 1135))

                self.write()

            except struct.error:
                print(f"Error: データの構造が正しくありません -> {self.data.hex()}")
                continue

    def write(self):
        self.pi.set_servo_pulsewidth(self.steering_pin, self.steering)
        self.pi.set_servo_pulsewidth(self.throttle_pin, self.throttle)
        self.pi.set_servo_pulsewidth(self.brake_pin, self.brake)

        if self.state == 0:
            self.pi.write(self.drive_pin, 1)
            self.pi.write(self.reverse_pin, 0)
        elif self.state == 1:
            self.pi.write(self.drive_pin, 0)
            self.pi.write(self.reverse_pin, 0)
        elif self.state == 2:
            self.pi.write(self.drive_pin, 0)
            self.pi.write(self.reverse_pin, 1)
        else:
            print(f"Warning: 未知の状態 {self.state} を受信しました")

    def shutdown(self, signum, frame):
        print("シャットダウン中...")
        self.running = False
        self.pi.set_servo_pulsewidth(self.steering_pin, 1500)
        self.pi.set_servo_pulsewidth(self.throttle_pin, 1000)
        self.pi.set_servo_pulsewidth(self.brake_pin, 1948)
        self.pi.write(self.drive_pin, 0)
        self.pi.write(self.reverse_pin, 0)
        self.pi.stop()
        sys.exit(0)


if __name__ == "__main__":
    receiver = Receiver()

    # Ctrl+Cでshutdownを呼び出すためのシグナルハンドラ設定
    signal.signal(signal.SIGINT, receiver.shutdown)

    # 受信ループ開始
    receiver.run()
