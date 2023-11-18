import numpy as np

# Only generate data for the x-axis

gyro_alpha = 0.25
accel_alpha = 0.01

gyro_drift = []
accelerometer_noise = []

gyro_true = [0]
accel_true = [0]

for i, time in enumerate(np.arange(0, 10, 0.01).tolist()):

    # Generate gyro drift noise
    gyro_drift += [i*0.001]

    # Generate accelerometer noise
    accelerometer_noise += [20*(np.random.rand(1)[0]-0.5)]

    # Generate gyro & accelerometer true value
    accel_pitch = 0
    gyro_pitch = 0
    if time > 7.5:
        # at 7.5 seconds the IMU is tipped to zero degrees
        accel_pitch = 0
        gyro_pitch = 0
    elif time > 5.0:
        # at 5.0 seconds the IMU is tipped to -45 degrees
        accel_pitch = -45
        gyro_pitch = -45
    elif time > 2.5:
        # at 2.5 seconds the IMU is tipped to 45 degrees
        accel_pitch = 45
        gyro_pitch = 45
    else:
        # all other times the IMU is 0 degrees
        accel_pitch = 0
        gyro_pitch = 0

    accel_true += [accel_pitch*accel_alpha + (1.0-accel_alpha)*accel_true[-1]]
    gyro_true += [gyro_pitch*gyro_alpha + (1.0-gyro_alpha)*gyro_true[-1]]


with open('KalmanTestData.csv', 'w+') as f:
    for i in range(len(gyro_drift)):
        f.write('{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(gyro_drift[i], accelerometer_noise[i], gyro_true[i], accel_true[i]))
