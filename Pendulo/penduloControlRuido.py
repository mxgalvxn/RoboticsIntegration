import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def single_pendulum(t, y, L, m, g, Kp, Kd):
    theta, theta_dot = y
    theta_ddot = (-g * np.sin(theta)) / L
    theta_eq = 0.0
    theta_dot_eq = 0.0
    u = Kp * (theta_eq - theta) + Kd * (theta_dot_eq - theta_dot)
    theta_ddot += u
    return [theta_dot, theta_ddot]

def add_noise(y0, noise_level):
    y0_with_noise = y0.copy()
    y0_with_noise[0] += np.random.normal(scale=noise_level)
    y0_with_noise[1] += np.random.normal(scale=noise_level)
    return y0_with_noise

L = 1.0
m = 1.0
g = 9.81
theta_0 = np.pi / 2
theta_dot_0 = 0
y0 = [theta_0, theta_dot_0]
Kp = 100.0
Kd = 20.0

t_0 = 0.0
t_f = 10.0
n = 100
dt = 0.01
sigma = 0.1

w_random = np.random.normal(0, sigma, size=(n, int((t_f - t_0) / dt)))

x_trace = []

for i in range(n):
    y0_with_noise = add_noise(y0, noise_level=sigma)
    t_span = (t_0, t_f)
    t_eval = np.linspace(t_0, t_f, int((t_f - t_0) / dt) + 1)
    sol = solve_ivp(single_pendulum, t_span, y0_with_noise, args=(L, m, g, Kp, Kd), t_eval=t_eval)
    theta = sol.y[0]
    x = L * np.sin(theta)
    x_trace.append(x)

fig, ax = plt.subplots()
for i in range(n):
    ax.plot(t_0 + dt * np.arange(len(x_trace[i])), x_trace[i], alpha=0.5)
ax.set_xlabel('Tiempo (s)')
ax.set_xlim(0, 0.5)
ax.set_ylabel('Posición x')
ax.set_title('Simulaciones del péndulo con control PD y ruido')
ax.grid(True)
plt.show()
