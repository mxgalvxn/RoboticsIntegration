import numpy as np
import matplotlib.pyplot as plt

wheelbase = 0.5
dt = 0.01  # Reducimos el paso de tiempo para hacerlo compatible con el otro código
v_max = 1.0
w_max = np.pi / 4

t_0 = 0.0
t_f = 5.0
n = 100
sigma = 0.1

w_random = np.random.normal(0, sigma, size=(n, int((t_f - t_0) / dt)))

x = np.zeros((n, int((t_f - t_0) / dt) + 1))

# Aproximación lineal de la función de control
def control(t):
    v = 0.5 * v_max * t
    w = 0.8 - 0.8 * t
    return v, w

for i in range(n):
    x[i, 0] = 0.0
    for t_idx in range(1, len(x[i])):
        v, w = control(t_idx * dt)
        v_left = v - w * wheelbase / 2
        v_right = v + w * wheelbase / 2
        x[i, t_idx] = x[i, t_idx-1] + (v_left + v_right) / 2 * np.cos(x[i, t_idx-1]) * dt + w_random[i, t_idx-1] * dt

plt.figure()
for i in range(n):
    plt.plot(t_0 + dt * np.arange(len(x[i])), x[i])
plt.xlabel('Tiempo (s)')
plt.ylabel('Variable de estado (x)')
plt.title('Simulaciones del modelo con ruido')
plt.grid(True)
plt.show()
