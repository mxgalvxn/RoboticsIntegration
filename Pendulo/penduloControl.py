import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parámetros del péndulo
L = 1  # longitud de la cuerda
g = 9.81  # aceleración gravitatoria
theta0 = np.pi / 4  # ángulo inicial
omega = np.sqrt(g / L)  # frecuencia angular

# Condiciones iniciales
theta = theta0  # ángulo inicial
dtheta_dt = 0  # velocidad angular inicial

# Controlador proporcional (P)
Kp = .005  # Ganancia proporcional

# Tiempo de simulación
t_final = 10  # segundos
dt = 0.01  # paso de tiempo
t = np.arange(0, t_final, dt)  # vector de tiempo

# Función para actualizar la animación
def update(frame):
    global theta, dtheta_dt

    # Posición deseada
    desired_position = -1

    # Error de posición
    error = desired_position - (-L * np.cos(theta))

    # Control proporcional
    torque = Kp * error

    # Aceleración angular
    ddtheta_dt2 = (-omega**2 * theta + torque) / L

    # Actualización de la velocidad angular y posición angular
    dtheta_dt += dt * ddtheta_dt2
    theta += dt * dtheta_dt

    # Mantener el ángulo en el rango [-pi, pi]
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    # Actualizar la posición del péndulo
    pendulum.set_data([0, L * np.sin(theta)], [0, -L * np.cos(theta)])

    return pendulum,

# Crear la figura y el eje
fig, ax = plt.subplots()
ax.set_xlim(-L - 0.5, L + 0.5)
ax.set_ylim(-L - 0.5, L + 0.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('Simulación del péndulo controlado')

# Dibujar el péndulo
pendulum, = ax.plot([], [], lw=2)

# Crear la animación
ani = FuncAnimation(fig, update, frames=len(t), interval=dt*1000, blit=True)

# Mostrar la animación
plt.show()
