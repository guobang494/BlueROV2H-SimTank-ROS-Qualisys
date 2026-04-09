#!/usr/bin/env python3

import rospy
import ast
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64

G_TO_KGF = 9.80665

# 16 V T200 normalized 7th-order fit from local_folder/T200-Public-Performance-Data-10-20V-September-2019.xlsx
# x_n = (PWM - pwm_norm_center) / pwm_norm_scale
# Thrust (kgf) = c0*x_n^7 + c1*x_n^6 + ... + c6*x_n + c7
DEFAULT_PWM_NORM_CENTER = 1500.0
DEFAULT_PWM_NORM_SCALE = 400.0
DEFAULT_POLY_COEFFS = [
    3.0009567411800813e+00,   # x_n^7
    3.5134822376186342e-01,   # x_n^6
    -7.6848330731177734e+00,  # x_n^5
    -6.5273863450617953e-01,  # x_n^4
    7.7496263278239033e+00,   # x_n^3
    9.1143646200164341e-01,   # x_n^2
    1.7012170055475782e+00,   # x_n^1
    1.0925224743988280e-03,   # x_n^0
]


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def polyval(coeffs, x):
    y = 0.0
    for c in coeffs:
        y = y * x + c
    return y


def thrust_kgf_from_pwm(pwm, coeffs, pwm_norm_center, pwm_norm_scale):
    x_n = (pwm - pwm_norm_center) / pwm_norm_scale
    return polyval(coeffs, x_n)


def invert_thrust_kgf_to_pwm(thrust_kgf, coeffs, pwm_norm_center, pwm_norm_scale, pwm_min, pwm_max, pwm_neutral):
    """
    Numerically invert thrust->PWM using bisection inside [pwm_min, pwm_max].
    Assumes the model is monotonic in the configured interval.
    """
    t_lo = thrust_kgf_from_pwm(pwm_min, coeffs, pwm_norm_center, pwm_norm_scale)
    t_hi = thrust_kgf_from_pwm(pwm_max, coeffs, pwm_norm_center, pwm_norm_scale)
    increasing = t_hi >= t_lo

    # Clamp target thrust to achievable range in this PWM window.
    thrust_kgf = clamp(thrust_kgf, min(t_lo, t_hi), max(t_lo, t_hi))

    lo = float(pwm_min)
    hi = float(pwm_max)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        t_mid = thrust_kgf_from_pwm(mid, coeffs, pwm_norm_center, pwm_norm_scale)
        if increasing:
            if t_mid < thrust_kgf:
                lo = mid
            else:
                hi = mid
        else:
            if t_mid > thrust_kgf:
                lo = mid
            else:
                hi = mid

    pwm = int(round(0.5 * (lo + hi)))
    if pwm < pwm_min or pwm > pwm_max:
        return int(pwm_neutral if pwm_neutral >= pwm_min and pwm_neutral <= pwm_max else clamp(pwm, pwm_min, pwm_max))
    return pwm


def publish_pwm_dict(pub, ch_pwm):
    msg = OverrideRCIn()
    msg.channels = [0] * 18
    for ch, pwm in ch_pwm.items():
        idx = int(ch) - 1
        if 0 <= idx < 18:
            msg.channels[idx] = int(pwm)
    pub.publish(msg)


current_cmd = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0, 'mz': 0.0}
cmd_ready = {'fx': False, 'fy': False, 'fz': False, 'mz': False}


def cb_fx(msg):
    current_cmd['fx'] = msg.data
    cmd_ready['fx'] = True


def cb_fy(msg):
    current_cmd['fy'] = msg.data
    cmd_ready['fy'] = True


def cb_fz(msg):
    current_cmd['fz'] = msg.data
    cmd_ready['fz'] = True


def cb_mz(msg):
    current_cmd['mz'] = msg.data
    cmd_ready['mz'] = True


def force_n_to_pwm(force_n, force_min_n, force_max_n, pwm_min, pwm_max, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral):
    f_clamped = clamp(force_n, force_min_n, force_max_n)
    thrust_kgf = f_clamped / G_TO_KGF

    thrust_at_pwm_min = thrust_kgf_from_pwm(pwm_min, coeffs, pwm_norm_center, pwm_norm_scale)
    thrust_at_pwm_max = thrust_kgf_from_pwm(pwm_max, coeffs, pwm_norm_center, pwm_norm_scale)
    thrust_min = min(thrust_at_pwm_min, thrust_at_pwm_max)
    thrust_max = max(thrust_at_pwm_min, thrust_at_pwm_max)
    thrust_kgf = clamp(thrust_kgf, thrust_min, thrust_max)

    pwm = invert_thrust_kgf_to_pwm(thrust_kgf, coeffs, pwm_norm_center, pwm_norm_scale, pwm_min, pwm_max, pwm_neutral)
    return int(clamp(pwm, pwm_min, pwm_max)), f_clamped, thrust_kgf


def torque_nm_to_pwm(torque_nm, torque_min_nm, torque_max_nm, pwm_min, pwm_max, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral, tam_mz_sum_abs):
    t_clamped = clamp(torque_nm, torque_min_nm, torque_max_nm)
    if tam_mz_sum_abs <= 1e-9:
        # Fallback to neutral if TAM coefficient is invalid.
        return int(pwm_neutral), t_clamped, thrust_kgf_from_pwm(pwm_neutral, coeffs, pwm_norm_center, pwm_norm_scale)

    # TAM-based equivalent per-horizontal-thruster force for pure yaw:
    # Mz = sum_i(tam_mz_i * T_i). For T_i = sign(tam_mz_i) * T_eq -> Mz = T_eq * sum(abs(tam_mz_i))
    # Therefore T_eq = Mz / sum(abs(tam_mz_i)).
    thrust_n = t_clamped / tam_mz_sum_abs
    thrust_kgf = thrust_n / G_TO_KGF

    thrust_at_pwm_min = thrust_kgf_from_pwm(pwm_min, coeffs, pwm_norm_center, pwm_norm_scale)
    thrust_at_pwm_max = thrust_kgf_from_pwm(pwm_max, coeffs, pwm_norm_center, pwm_norm_scale)
    thrust_kgf = clamp(thrust_kgf, min(thrust_at_pwm_min, thrust_at_pwm_max), max(thrust_at_pwm_min, thrust_at_pwm_max))

    pwm = invert_thrust_kgf_to_pwm(thrust_kgf, coeffs, pwm_norm_center, pwm_norm_scale, pwm_min, pwm_max, pwm_neutral)
    return int(clamp(pwm, pwm_min, pwm_max)), t_clamped, thrust_kgf


def main():
    rospy.init_node("force_to_rc_override")

    topic_fx = rospy.get_param('~topic_fx', '/bluerov2_heavy/cmd_velocity/linear/x')
    topic_fy = rospy.get_param('~topic_fy', '/bluerov2_heavy/cmd_velocity/linear/y')
    topic_fz = rospy.get_param('~topic_fz', '/bluerov2_heavy/cmd_velocity/linear/z')
    topic_mz = rospy.get_param('~topic_mz', '/bluerov2_heavy/cmd_velocity/angular/z')
    invert_fz = bool(rospy.get_param('~invert_fz', False))

    force_min_x = rospy.get_param('~min_force_x', -10.0)
    force_max_x = rospy.get_param('~max_force_x', 10.0)
    force_min_y = rospy.get_param('~min_force_y', -10.0)
    force_max_y = rospy.get_param('~max_force_y', 10.0)
    force_min_z = rospy.get_param('~min_force_z', -15.0)
    force_max_z = rospy.get_param('~max_force_z', 15.0)
    torque_min_z = rospy.get_param('~min_torque_z', -12.0)
    torque_max_z = rospy.get_param('~max_torque_z', 12.0)
    tam_mz_sum_abs = float(rospy.get_param('~tam_mz_sum_abs', 0.6830651506258189))

    pwm_min_x = int(rospy.get_param('~min_pwm_x', 1450))
    pwm_max_x = int(rospy.get_param('~max_pwm_x', 1550))
    pwm_min_y = int(rospy.get_param('~min_pwm_y', 1450))
    pwm_max_y = int(rospy.get_param('~max_pwm_y', 1550))
    pwm_min_z = int(rospy.get_param('~min_pwm_z', 1450))
    pwm_max_z = int(rospy.get_param('~max_pwm_z', 1550))
    pwm_min_yaw = int(rospy.get_param('~min_pwm_yaw', 1460))
    pwm_max_yaw = int(rospy.get_param('~max_pwm_yaw', 1540))
    pwm_neutral = int(rospy.get_param('~pwm_neutral', 1500))

    pwm_norm_center = float(rospy.get_param('~pwm_norm_center', DEFAULT_PWM_NORM_CENTER))
    pwm_norm_scale = float(rospy.get_param('~pwm_norm_scale', DEFAULT_PWM_NORM_SCALE))
    poly_coeffs = rospy.get_param('~fit_poly_coeffs', DEFAULT_POLY_COEFFS)
    if isinstance(poly_coeffs, str):
        # roslaunch args often pass list-looking values as strings.
        try:
            parsed = ast.literal_eval(poly_coeffs)
        except Exception:
            # Fallback: comma-separated string without brackets.
            parsed = [v.strip() for v in poly_coeffs.strip().strip('[]').split(',') if v.strip()]
        poly_coeffs = parsed
    coeffs = [float(v) for v in poly_coeffs]

    ch_forward, ch_lateral, ch_vertical, ch_yaw = 5, 6, 3, 4
    rate_hz = float(rospy.get_param('~publish_rate_hz', 20.0))

    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    rospy.Subscriber(topic_fx, Float64, cb_fx, queue_size=10)
    rospy.Subscriber(topic_fy, Float64, cb_fy, queue_size=10)
    rospy.Subscriber(topic_fz, Float64, cb_fz, queue_size=10)
    rospy.Subscriber(topic_mz, Float64, cb_mz, queue_size=10)

    rospy.loginfo('Waiting for force/torque commands from bluerov2_motion_control...')
    while not rospy.is_shutdown():
        if any(cmd_ready.values()):
            break
        rospy.sleep(0.1)
    if rospy.is_shutdown():
        return

    rospy.loginfo('Starting PWM loop using 16V T200 polynomial thrust fit (order %d)', len(coeffs) - 1)
    rospy.loginfo('Normalization: x_n=(PWM-%.3f)/%.3f', pwm_norm_center, pwm_norm_scale)
    rospy.loginfo('Polynomial coeffs (descending on x_n): %s', coeffs)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        fx = current_cmd['fx']
        fy = current_cmd['fy']
        fz_raw = current_cmd['fz']
        fz = -fz_raw if invert_fz else fz_raw
        mz = current_cmd['mz']

        pwm_forward, fx_c, kgf_x = force_n_to_pwm(
            fx, force_min_x, force_max_x, pwm_min_x, pwm_max_x, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral)
        pwm_lateral, fy_c, kgf_y = force_n_to_pwm(
            fy, force_min_y, force_max_y, pwm_min_y, pwm_max_y, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral)
        pwm_vertical, fz_c, kgf_z = force_n_to_pwm(
            fz, force_min_z, force_max_z, pwm_min_z, pwm_max_z, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral)
        pwm_yaw, mz_c, kgf_yaw = torque_nm_to_pwm(
            mz, torque_min_z, torque_max_z, pwm_min_yaw, pwm_max_yaw, coeffs, pwm_norm_center, pwm_norm_scale, pwm_neutral, tam_mz_sum_abs)

        publish_pwm_dict(pub, {
            ch_forward: pwm_forward,
            ch_lateral: pwm_lateral,
            ch_vertical: pwm_vertical,
            ch_yaw: pwm_yaw,
        })

        rospy.loginfo_throttle(
            2.0,
            (
                f'Cmd raw: Fx={fx:.2f}N Fy={fy:.2f}N Fz={fz_raw:.2f}N Mz={mz:.2f}Nm | '
                f'clamped: Fx={fx_c:.2f}N Fy={fy_c:.2f}N Fz={fz_c:.2f}N Mz={mz_c:.2f}Nm'
            ),
        )
        rospy.loginfo_throttle(
            2.0,
            (
                f'Model thrust: X={kgf_x:.3f}kgf Y={kgf_y:.3f}kgf Z={kgf_z:.3f}kgf Yaw={kgf_yaw:.3f}kgf | '
                f'PWM: ch{ch_forward}={pwm_forward} ch{ch_lateral}={pwm_lateral} '
                f'ch{ch_vertical}={pwm_vertical} ch{ch_yaw}={pwm_yaw}'
            ),
        )

        rate.sleep()

    rospy.loginfo('Sending neutral PWM...')
    for _ in range(5):
        publish_pwm_dict(pub, {
            ch_forward: pwm_neutral,
            ch_lateral: pwm_neutral,
            ch_vertical: pwm_neutral,
            ch_yaw: pwm_neutral,
        })
        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
