import numpy as np
import matplotlib.pyplot as plt

t_0 = 0.0
t_f = 10.0
n = 100
dt = 0.01
sigma = 0.1

w_random = np.random.normal(0, sigma, size=(n, int((t_f - t_0) / dt)))

x = np.zeros((n, int((t_f - t_0) / dt) + 1))

for i in range(n):
    x[i, 0] = 0.0
    for t_idx in range(1, len(x[i])):
        x[i, t_idx] = x[i, t_idx-1] + w_random[i, t_idx-1] * dt

plt.figure()
for i in range(n):
    plt.plot(t_0 + dt * np.arange(len(x[i])), x[i])
plt.xlabel('Tiempo (s)')
plt.ylabel('Variable de estado (x)')
plt.title('Simulaciones del modelo con ruido')
plt.grid(True)
plt.show()
