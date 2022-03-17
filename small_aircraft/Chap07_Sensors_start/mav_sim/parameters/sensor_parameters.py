"""sensor_parameters.py defines the parameters and noise characteristics of the different sensors
"""
import numpy as np

#-------- Accelerometer --------
accel_sigma: float = 0.0025*9.81  # standard deviation of accelerometers in m/s^2

#-------- Rate Gyro --------
gyro_x_bias: float = 0#np.radians(5*np.random.uniform(-1, 1))  # bias on x_gyro
gyro_y_bias: float = 0#np.radians(5*np.random.uniform(-1, 1))  # bias on y_gyro
gyro_z_bias: float = 0#np.radians(5*np.random.uniform(-1, 1))  # bias on z_gyro
gyro_sigma: float = np.radians(0.13)  # standard deviation of gyros in rad/sec

#-------- Pressure Sensor(Altitude) --------
abs_pres_bias: float = 0. # Bias on the absolute pressure measurement
abs_pres_sigma: float = 0.01*1000  # standard deviation of absolute pressure sensors in Pascals

#-------- Pressure Sensor (Airspeed) --------
diff_pres_bias: float = 0. # Bias on the differential pressure
diff_pres_sigma: float = 0.002*1000  # standard deviation of diff pressure sensor in Pascals

#-------- Magnetometer --------
mag_dec: float = np.radians(12.5) # Declination of the magnetic to inertial frame
mag_inc: float = np.radians(-66) # Inclination of the magnetic to inertial frame
mag_sigma: float = np.radians(0.03)

#-------- GPS --------
ts_gps: float = 0.2
gps_k: float = 1. / 1100.  # 1 / s
gps_n_sigma: float = 0.05
gps_e_sigma: float = 0.05
gps_h_sigma: float = 0.15
gps_Vg_sigma: float = 0.005
gps_course_sigma: float = gps_Vg_sigma / 20
