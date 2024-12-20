from controller import Robot

TIME_STEP = 32

# Fungsi Kalman Filter
def kalman_filter(z, u, x, P, process_noise=0.1, measurement_noise=1):
    """
    Kalman filter untuk memperbarui posisi estimasi robot.

    Args:
    z: Pengukuran sensor (jarak).
    u: Estimasi pergerakan dari encoder.
    x: Posisi estimasi sebelumnya.
    P: Ketidakpastian estimasi sebelumnya.
    process_noise: Noise proses (default 0.1).
    measurement_noise: Noise pengukuran (default 1).

    Returns:
    x: Posisi estimasi yang diperbarui.
    P: Ketidakpastian yang diperbarui.
    """
    # Prediksi langkah
    x_pred = x + u
    P_pred = P + process_noise  # Tambahkan noise proses

    # Koreksi langkah
    K = P_pred / (P_pred + measurement_noise)  # Gain Kalman
    x = x_pred + K * (z - x_pred)  # Perbarui posisi estimasi
    P = (1 - K) * P_pred  # Perbarui ketidakpastian
    return x, P

# Inisialisasi robot
robot = Robot()

# Motor roda
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Set motor ke mode velocity
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoder roda
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Sensor jarak
distance_sensor = robot.getDevice("ps0")
distance_sensor.enable(TIME_STEP)

# Variabel untuk Kalman Filter
x = 0.0  # Posisi awal
P = 1.0  # Ketidakpastian awal

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Ambil nilai encoder
    left_distance = left_encoder.getValue()
    right_distance = right_encoder.getValue()

    # Estimasi pergerakan robot (input u)
    u = (left_distance + right_distance) / 2.0

    # Ambil pengukuran sensor jarak (z)
    z = distance_sensor.getValue()

    # Terapkan Kalman Filter
    x, P = kalman_filter(z, u, x, P)

    # Cetak estimasi posisi
    print(f"Estimasi Posisi Robot: {x}")
