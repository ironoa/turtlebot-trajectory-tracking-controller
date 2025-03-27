import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

#
# STEP 1) Define Robot/Motor Parameter
#
Ta = 0.150  # motor lag time constant (seconds).

#
# STEP 2) Define the Plant for one of the axis (x-axis)
#
#  Gx(s) = 1 / [s * (1 + s*Ta)]
#
num_Gx = [1.0]             # Numerator = 1
den_Gx = [Ta, 1.0, 0.0]    # Denominator = s*(1 + s*Ta) => Ta*s^2 + s

Gx = ctrl.tf(num_Gx, den_Gx)

#
# STEP 3) Define your PI Controller
#
#  Cx(s) = Kp_x + Ki_x/s
#        = Kp_x (1 + 1/(Ti_x s))  where Ki_x = Kp_x / Ti_x
#

# Kp_x = 13.0
# Ti_x = 4.0 
# VERY GOOD
# === Open-Loop Margins ===
# Gain margin    = inf at ω = nan rad/s
# Phase margin   = 37.36 deg at ω = 8.20 rad/s
# === Closed-Loop ===
# Rise Time     = 0.16 s
# Settling Time = 1.28 s
# Overshoot     = 33.48 %
# Closed-loop -3 dB bandwidth ~ 13.278 rad/s

Kp_x = 8.0
Ti_x = 5.0 
# === Open-Loop Margins ===
# Gain margin    = inf at ω = nan rad/s
# Phase margin   = 46.26 deg at ω = 5.97 rad/s
# === Closed-Loop ===
# Rise Time     = 0.21 s
# Settling Time = 1.74 s
# Overshoot     = 23.62 %
# Closed-loop -3 dB bandwidth ~ 9.705 rad/s
# EVEN BETTER (less overshoot, higher phase margin)


Ki_x = Kp_x / Ti_x

Cx = ctrl.tf([Kp_x, Ki_x], [1, 0])  
# That means (Kp*s + Ki)/s

#
# STEP 4) Form the open-loop transfer function
#
Lx = ctrl.series(Cx, Gx)
# Lx(s) = Cx(s)*Gx(s)

#
# STEP 5) Bode Plot
#
mag, phase, omega = ctrl.bode(Lx, Plot=False)

# We'll plot magnitude & phase ourselves for clarity
plt.figure(figsize=(9, 6))

# Magnitude (in dB)
plt.subplot(2, 1, 1)
plt.semilogx(omega, 20*np.log10(mag))
plt.title("Bode Plot of Lx(s) = Cx(s)*Gx(s)")
plt.ylabel("Magnitude (dB)")
plt.grid(True, which="both", linestyle="--")

# Phase (in degrees)
plt.subplot(2, 1, 2)
plt.semilogx(omega, np.degrees(phase))
plt.xlabel("Frequency (rad/s)")
plt.ylabel("Phase (deg)")
plt.grid(True, which="both", linestyle="--")
plt.tight_layout()
# plt.show()

#
# STEP 6) Calculate and Print Margins
#
gm, pm, w_gm, w_pm = ctrl.margin(Lx)
print("=== Open-Loop Margins ===")
print(f"Gain margin    = {gm:0.2f} at ω = {w_gm:0.2f} rad/s")
print(f"Phase margin   = {pm:0.2f} deg at ω = {w_pm:0.2f} rad/s")

#
# STEP 7) Check the Closed-Loop Step
#  T_cl(s) = Lx(s) / [1 + Lx(s)]
#
T_cl = ctrl.feedback(Lx, 1)

mag, phase, omega = ctrl.bode(T_cl, Plot=False)  # T_cl is the closed-loop system
# mag is the magnitude (as a ratio, not dB)
# phase in radians
# omega is the frequency array

# Convert to dB:
mag_db = 20*np.log10(mag)

# The low-frequency magnitude (in dB) is near mag_db[0] 
# (assuming the first frequency is very low).
# We want to find the frequency where it is 3 dB down from that plateau.

low_freq_level = mag_db[0]  # approximate
target_db = low_freq_level - 3.0

bw_index = None
for i in range(len(omega)):
    if mag_db[i] <= target_db:
        bw_index = i
        break

print("=== Closed-Loop ===")
tr, ts = ctrl.step_info(T_cl)['RiseTime'], ctrl.step_info(T_cl)['SettlingTime']
os = (ctrl.step_info(T_cl)['Overshoot'])
print(f"Rise Time     = {tr:.2f} s")
print(f"Settling Time = {ts:.2f} s")
print(f"Overshoot     = {os:.2f} %")
if bw_index is not None:
    bw_freq = omega[bw_index]
    print(f"Closed-loop -3 dB bandwidth ~ {bw_freq:.3f} rad/s")
else:
    print("Did not find a -3 dB point in the given frequency range.")

t, y = ctrl.step_response(T_cl)
plt.figure()
plt.plot(t, y, label="CL Step Resp")
plt.title("Closed-Loop Step Response (x-axis)")
plt.xlabel("Time (s)")
plt.ylabel("Position (or Error) Output")
plt.grid(True)
plt.legend()

plt.show()
