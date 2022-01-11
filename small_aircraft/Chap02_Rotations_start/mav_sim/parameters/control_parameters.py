"""control_parameter.py Defines the parameters used in control of the MAV
"""
import mav_sim.chap5.model_coef as TF
import mav_sim.parameters.aerosonde_parameters as MAV

gravity: float = MAV.gravity  # gravity constant
rho: float = MAV.rho  # density of air
sigma: float = 0.05  # low pass filter gain for derivative
Va0: float = TF.Va_trim

#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll: float = 10.0 #20 #7
zeta_roll: float = 0.707
roll_kp: float = wn_roll**2/TF.a_phi2
roll_kd: float = (2.0 * zeta_roll * wn_roll - TF.a_phi1) / TF.a_phi2

#----------course loop-------------
wn_course: float = wn_roll / 20.0
zeta_course: float = 1.0
course_kp: float = 2.0 * zeta_course * wn_course * Va0 / gravity
course_ki: float = wn_course**2 * Va0 / gravity

#----------yaw damper-------------
yaw_damper_p_wo: float = 0.45  # (old) 1/0.5
yaw_damper_kr: float = 0.2  # (old) 0.5

#----------pitch loop-------------
wn_pitch: float = 15.0 #24.0
zeta_pitch: float = 0.707
pitch_kp: float = (wn_pitch**2 - TF.a_theta2) / TF.a_theta3
pitch_kd: float = (2.0 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
K_theta_DC: float = pitch_kp * TF.a_theta3 / (TF.a_theta2 + pitch_kp * TF.a_theta3)

#----------altitude loop-------------
wn_altitude: float = wn_pitch / 30.0
zeta_altitude: float = 1.0
altitude_kp: float = 2.0 * zeta_altitude * wn_altitude / K_theta_DC / Va0
altitude_ki: float = wn_altitude**2 / K_theta_DC / Va0
altitude_zone: float = 10.0  # moving saturation limit around current altitude

#---------airspeed hold using throttle---------------
wn_airspeed_throttle: float = 1.5 #3.0
zeta_airspeed_throttle: float = 2.0  # 0.707
airspeed_throttle_kp: float = (2.0 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
airspeed_throttle_ki: float = wn_airspeed_throttle**2 / TF.a_V2
