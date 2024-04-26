import numpy as np
import matplotlib.pyplot as plt

wheelbase = 0.5
dt = 0.1
v_max = 1.0
w_max = np.pi / 4

def control(t):
    v = v_max * np.sin(0.5 * t)
    w = 0.8 * np.cos(t)
    return v, w

def linearized_control(t):
    # En este punto, la derivada de sin(t/2) es cos(t/2) * 0.5
    # y la derivada de cos(t) es -sin(t).
    v = v_max * (0.5 * np.cos(0.5 * t))
    w = -0.8 * np.sin(t)
    return v, w

def simulate():
    x = 0.0
    y = 0.0
    theta = 0.0
    theta_pos = [0]
    time_values = [0]
    path = [(x, y)]

    for t in np.arange(0, 5, dt):
        v, w = control(t)
        v_lin, w_lin = linearized_control(t)

        v_left = v - w * wheelbase / 2
        v_right = v + w * wheelbase / 2

        v_left_lin = v_lin - w_lin * wheelbase / 2
        v_right_lin = v_lin + w_lin * wheelbase / 2

        x += (v_left + v_right) / 2 * np.cos(theta) * dt
        y += (v_left + v_right) / 2 * np.sin(theta) * dt
        theta += (v_right - v_left) / wheelbase * dt

        print(theta)
        path.append((x, y))
        plt.clf()
        plot_path(path)
        #plot_orientation(theta)
        plot_linearized_orientation(t, linearized_control(t)[1])
        plt.pause(dt)

def plot_path(path):
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    plt.plot(x_values, y_values, '-o', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y ')
    plt.title('Carrito')
    plt.grid(True)
    plt.axis('equal')

def plot_orientation(theta_values):
    plt.plot(theta_values, '-')
    plt.xlabel('Tiempo (segundos)')
    plt.ylabel('Theta (radianes)')
    plt.title('Orientación del Carrito')
    plt.grid(True)

def plot_linearized_orientation(t_value, theta_lin_value):
    plt.plot(t_value, theta_lin_value * t_value, '--', color='red')
    plt.legend(['Real', 'Linearizada'])
    plt.xlabel('Tiempo (segundos)')
    plt.ylabel('Theta (radianes)')
    plt.title('Comparación de Orientación del Carrito (Linealizada vs. Real)')
    plt.grid(True)


simulate()
plt.show()
