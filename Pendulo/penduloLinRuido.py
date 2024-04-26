import numpy as np
import matplotlib.pyplot as plt

def solve_pendulum(theta_0, omega_0, t, L, g):
    dt = t[1] - t[0]
    theta = np.zeros_like(t)
    omega = np.zeros_like(t)
    theta[0] = theta_0
    omega[0] = omega_0
    for i in range(1, len(t)):
        omega[i] = omega[i-1] - (g/L) * np.sin(theta[i-1]) * dt
        theta[i] = theta[i-1] + omega[i] * dt
    return theta, omega

g = 9.81
L = 1.0
n = 100
theta_0_min = np.pi/6
theta_0_max = np.pi/3
omega_0_min = 0.0
omega_0_max = 2.0

t = np.linspace(0, 10, 1000)
theta = np.zeros((n, len(t)))
omega = np.zeros((n, len(t)))
x = np.zeros((n, len(t)))

theta_0 = np.random.uniform(theta_0_min, theta_0_max, size=n)
omega_0 = np.random.uniform(omega_0_min, omega_0_max, size=n)

for i in range(n):
    theta_sim, omega_sim = solve_pendulum(theta_0[i], omega_0[i], t, L, g)
    x_sim = L * np.sin(theta_sim)
    theta[i] = theta_sim
    omega[i] = omega_sim
    x[i] = x_sim

fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
axs[0].plot(t, theta.T, alpha=0.5)
axs[0].set_ylabel('Ángulo (radianes)')
axs[0].grid()
axs[1].plot(t, x.T, alpha=0.5)
axs[1].set_xlabel('Tiempo (s)')
axs[1].set_ylabel('Posición (m)')
axs[1].grid()
plt.suptitle('Trayectorias del péndulo simple (Montecarlo)')
plt.show()
