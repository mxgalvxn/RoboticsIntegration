import numpy as np
import matplotlib.pyplot as plt

wheelbase = 0.5
dt = 0.01
v_max = 1.0
target_x = 2.0
target_y = 0.0

def control(t, x, y, theta):
    error_x = target_x - x
    error_y = target_y - y
    theta_target = np.arctan2(error_y, error_x)
    error_theta = theta_target - theta
    if error_theta > np.pi:
        error_theta -= 2 * np.pi
    elif error_theta < -np.pi:
        error_theta += 2 * np.pi
    Kp = 0.5
    w = Kp * error_theta
    v = v_max
    return v, w

def simulate():
    t_0 = 0.0
    t_f = 5.0
    n = 100
    sigma = 0.1

    w_random = np.random.normal(0, sigma, size=(n, int((t_f - t_0) / dt)))

    x = np.zeros((n, int((t_f - t_0) / dt) + 1))

    x[0, 0] = 0.0
    theta = 0.0
    for i in range(n):
        for t_idx in range(1, len(x[0])):
            v, w = control(t_idx * dt, x[0, t_idx-1], 0, theta)
            v_left = v - w * wheelbase / 2
            v_right = v + w * wheelbase / 2
            x[i, t_idx] = x[i, t_idx-1] + (v_left + v_right) / 2 * np.cos(x[i, t_idx-1]) * dt + w_random[i, t_idx-1] * dt


    plt.figure()
    for i in range(n):
        plt.plot(t_0 + dt * np.arange(len(x[i])), x[i])
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Variable de estado (x)')
    plt.title('Simulación del modelo con ruido y control')
    plt.grid(True)
    plt.show()

simulate()
