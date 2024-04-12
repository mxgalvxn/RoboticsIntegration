import numpy as np
import matplotlib.pyplot as plt
import math 

wheelbase = 0.5
dt = 0.1
v_max = 1.0
w_max = np.pi / 4

def control(t):
    v = v_max * np.sin(0.5 * t)
    w = 0.5 * np.cos(t)
    return v, w

def simulate():
    x = 0.0
    y = 0.0
    theta = 0.0
    theta_pos = []
    path = [(x, y)]

    for t in np.arange(0, 5, dt):
        v, w = control(t)
        #v_left = v - w * wheelbase / 2
        v_left = .4
        #v_right = v + w * wheelbase / 2
        v_right = .5
        x += (v_left + v_right) / 2 * np.cos(theta) * dt
        y += (v_left + v_right) / 2 * np.sin(theta) * dt
        theta += (v_right - v_left) / wheelbase * dt
        theta_pos.append(theta)
        print(theta)
        path.append((x, y))
        plt.clf()
        plot_path(path)
        plt.pause(dt)

def plot_path(path):
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    plt.plot(x_values, y_values, '-o', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y ')
    plt.title('carrito')
    plt.grid(True)
    plt.axis('equal')

simulate()
plt.show()
