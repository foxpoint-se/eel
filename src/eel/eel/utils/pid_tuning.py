# Ziegler Nichols procedure:
# 1. Start with Kp=small, Ki=0, Kd=0
# 2. Increase Kp until neutral stability (regular oscillation around target)
# 3. Save this Kp and call it Ku (=Kp at neutral stability)
# 4. Save the period of oscillation and call it Tu (seconds between peaks around the target)
# 5. Look up Kp, Ti, Td in a table, using values above
# 6. Compute Ki and Kd from Ki=Kp/Ti, Kd=Kp*Td
# 7. Now you have Kp, Ki, Kd to feed into your PID controller

# Kp = proportional gain
# Ki = integral gain
# Kd = derivative gain

# Ku = proportional gain at neutral stability
# Tu = period of oscillation when using Ku (in seconds)
# Ti = integral time when using Ku
# Td = derivative time when using Ku

# ======================================================
# Lookup table, Ziegler Nichols
#                       Kp          Ti          Td
# ------------------------------------------------------
# Classic PID           0.6Ku       Tu/2        Tu/8
# P                     0.5Ku
# PI                    0.45Ku      Tu/1.2
# PD                    0.8Ku                   Tu/8
# Pessen integration    0.8Ku       2Tu/5       3Tu/20
# Some overshoot        Ku/3        Tu/2        Tu/3
# No overshoot          0.2Ku       Tu/2        Tu/3

# ======================================================
# Measured values in simulation mode
# ======================================================
# Depth
# Kp = 10.0     Ki = 0      Kd = 0
# ------------------------------------------------------
# Ku = 10.0
# Tu = 43.0
# ======================================================
# Pitch
# Kp = 0.5     Ki = 0      Kd = 0
# ------------------------------------------------------
# Ku = 0.5
# Tu = 28.0
# ======================================================


def get_simulation_pid_settings():
    depth_Ku = 10.0
    depth_Tu = 43.0
    pitch_Ku = 0.5
    pitch_Tu = 28.0
    return depth_Ku, depth_Tu, pitch_Ku, pitch_Tu


def get_Ki(Kp, Ti):
    return Kp / Ti


def get_Kd(Kp, Td):
    return Kp * Td


def lookup_zieglernichols_gains(Ku, Tu, type):
    if type == "classic_PID":
        Kp = 0.6 * Ku
        Ti = Tu / 2
        Td = Tu / 8
        Ki = get_Ki(Kp, Ti)
        Kd = get_Kd(Kp, Td)
        return Kp, Ki, Kd
    elif type == "P":
        Kp = 0.5 * Ku
        return Kp, 0, 0

    elif type == "PI":
        Kp = 0.45 * Ku
        Ti = Tu / 1.2
        Ki = get_Ki(Kp, Ti)
        return Kp, Ki, 0

    elif type == "PD":
        Kp = 0.8 * Ku
        Td = Tu / 8
        Kd = get_Kd(Kp, Td)
        return Kp, 0, Kd

    elif type == "pessen_integration":
        Kp = 0.8 * Ku
        Ti = 2 * Tu / 5
        Td = 3 * Tu / 20
        Ki = get_Ki(Kp, Ti)
        Kd = get_Kd(Kp, Td)
        return Kp, Ki, Kd

    elif type == "some_overshoot":
        Kp = Ku / 3
        Ti = Tu / 2
        Td = Tu / 3
        Ki = get_Ki(Kp, Ti)
        Kd = get_Kd(Kp, Td)
        return Kp, Ki, Kd

    elif type == "no_overshoot":
        Kp = 0.2 * Ku
        Ti = Tu / 2
        Td = Tu / 3
        Ki = get_Ki(Kp, Ti)
        Kd = get_Kd(Kp, Td)
        return Kp, Ki, Kd

    raise TypeError("not a valid type")
