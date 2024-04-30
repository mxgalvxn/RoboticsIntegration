import numpy as np
import matplotlib.pyplot as plt

wheelbase = 0.5
dt = 0.1
v_max = 1.0
w_max = np.pi / 4
target_x = 0.0
target_y = 2.0

# Aproximación lineal de la función de control
def control(t, x, y, theta):
    # Calcula el error de posición respecto al objetivo
    error_x = target_x - x
    error_y = target_y - y
    
    # Calcula el ángulo hacia el objetivo
    theta_target = np.arctan2(error_y, error_x)
    
    # Calcula el error de orientación respecto al objetivo
    error_theta = theta_target - theta
    
    # Limita el error de orientación en el rango [-pi, pi]
    if error_theta > np.pi:
        error_theta -= 2 * np.pi
    elif error_theta < -np.pi:
        error_theta += 2 * np.pi
    
    # Define una ganancia proporcional
    Kp = 0.5
    
    # Calcula la velocidad angular de acuerdo al error de orientación
    w = Kp * error_theta
    
    # Fija la velocidad lineal a la máxima
    v = v_max
    
    return v, w

def simulate():
    x = 0.0
    y = 0.0
    theta = 0.0
    path = [(x, y)]

    for t in np.arange(0, 5, dt):
        v, w = control(t, x, y, theta)
        v_left = v - w * wheelbase / 2
        v_right = v + w * wheelbase / 2
        x += (v_left + v_right) / 2 * np.cos(theta) * dt
        y += (v_left + v_right) / 2 * np.sin(theta) * dt
        theta += (v_right - v_left) / wheelbase * dt
        path.append((x, y))
        plt.clf()
        plot_path(path)
        plot_orientation(theta)
        plt.pause(dt)

def plot_path(path):
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    plt.plot(x_values, y_values, '-o', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Trayectoria del Vehículo')
    plt.grid(True)
    plt.axis('equal')

def plot_orientation(theta_values):
    plt.plot(theta_values, '-')
    plt.xlabel('Tiempo (segundos)')
    plt.ylabel('Theta (radianes)')
    plt.title('Orientación del Vehículo')
    plt.grid(True)

simulate()
plt.show()
