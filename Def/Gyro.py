import time
from Def import VarGlobal
from mpu6050 import mpu6050

sensor = mpu6050(0x68,bus=0)

gyro_bias = {'x': 0, 'y': 0, 'z': 0}

num_calibration_samples = 100
print("Calibrating gyroscope...")
def Setup_Gyro():
    global num_calibration_samples,gyro_bias
    for i in range(num_calibration_samples):
        gyro_data = sensor.get_gyro_data()
        gyro_bias['x'] += gyro_data['x']
        gyro_bias['y'] += gyro_data['y']
        gyro_bias['z'] += gyro_data['z']
        time.sleep(0.01)

    gyro_bias['x'] /= num_calibration_samples
    gyro_bias['y'] /= num_calibration_samples
    gyro_bias['z'] /= num_calibration_samples

    print("Calibration complete.")

def get_gyro():
    global sensor
    VarGlobal.yaw = 0
    prev_time = time.time()
    while True:
        gyro_data = sensor.get_gyro_data()

        gyro_data['y'] -= gyro_bias['y']

        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        delta_yaw = gyro_data['y'] * dt
        VarGlobal.yaw += delta_yaw

        # print(f"Yaw: {yaw:.2f} degrees")
        time.sleep(0.01)

