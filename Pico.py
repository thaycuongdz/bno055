import os
import time
from machine import Pin, I2C, UART
from bno055 import BNO055

uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(13))

i2c = I2C(1, scl=Pin(27), sda=Pin(26))
imu = BNO055(i2c)
time.sleep(1)

led = Pin(0, Pin.OUT)
CALIBRATION_FILE = "calib.dat"

def save_calibration(filename, data):
    with open(filename, "wb") as f:
        f.write(data)

def load_calibration(filename):
    with open(filename, "rb") as f:
        return f.read()

def calibrate_sensor():
    print(" Đang tự hiệu chuẩn sensor...")
    while True:
        sys, gyro, accel, mag = imu.cal_status()
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print(" Sensor đã hiệu chuẩn đủ.")
            save_calibration(CALIBRATION_FILE, imu.get_calibration())
            uart.write("ready\n")
            break
        else:
            print(" Đợi hiệu chuẩn... SYS={}, GYRO={}, ACCEL={}, MAG={}".format(sys, gyro, accel, mag))
        time.sleep(0.5)


try:
    if CALIBRATION_FILE in os.listdir():
        print(" Tìm thấy file hiệu chuẩn, đang nạp...")
        imu.set_calibration(load_calibration(CALIBRATION_FILE))
        time.sleep(1)
        sys, gyro, accel, mag = imu.cal_status()
        if not (sys == 3 and gyro == 3 and accel == 3 and mag == 3):
            print(" Hiệu chuẩn cũ không ổn, cần tự hiệu chuẩn lại.")
            calibrate_sensor()
        else:
            uart.write("ready\n")
    else:
        print(" Không tìm thấy file hiệu chuẩn.")
        calibrate_sensor()
except Exception as e:
    print(" Lỗi khi nạp hiệu chuẩn:", e)
    calibrate_sensor()


sample_seq = 0

while True:
    try:
        quat = imu.quaternion()
        if quat:
            w, x, y, z = quat
            timestamp = time.ticks_us()
            message = ("{:.4f},{:.4f},{:.4f},{:.4f},{},{}\n".format(
                w, x, y, z, timestamp, sample_seq
            ))
            uart.write(message)
            sample_seq = (sample_seq + 1) & 0xFFFFFFFF
            # print("Đã gửi:", message.strip()) # Tắt in ra để giảm lag
            led.toggle()
        time.sleep_ms(10) # Tăng tốc độ lấy mẫu từ 25Hz (40ms) lên 100Hz (10ms)
    except Exception as e:
        print("⚠️ Lỗi trong vòng lặp:", e)
        time.sleep(1)
