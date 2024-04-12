import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Definición de las ecuaciones diferenciales del doble péndulo
def double_pendulum(theta1, theta1_dot, theta2, theta2_dot, L1, L2, m1, m2, g, dt):
    # Derivadas de las variables
    #theta1_ddot = (-g*(2*m1 + m2)*np.sin(theta1) - m2*g*np.sin(theta1 - 2*theta2) - 2*np.sin(theta1 - theta2)*m2*(theta2_dot**2*L2 + theta1_dot**2*L1*np.cos(theta1 - theta2))) / (L1*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2)))
    #theta2_ddot = (2*np.sin(theta1 - theta2)*(theta1_dot**2*L1*(m1 + m2) + g*(m1 + m2)*np.cos(theta1) + theta2_dot**2*L2*m2*np.cos(theta1 - theta2))) / (L2*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2)))
    #theta1_ddot = ((m1+m2)*L1*theta1_dot)+(m2*L2*theta2_dot*np.cos(theta1-theta2))+(m2*L2*)
    #theta2_ddot = ()

    theta1_ddot = -((m1+m2)/m1)*theta1 + -((g*m2)/(L1*m1))*theta2
    theta2_ddot = (((m1+m2)*g)/(L2*m1))*theta1 - (((m1+m2)*g)/(L2*m1))*theta2    

    # Método de Euler para la integración numérica
    theta1_new = theta1 + theta1_dot * dt
    theta1_dot_new = theta1_dot + theta1_ddot * dt
    theta2_new = theta2 + theta2_dot * dt
    theta2_dot_new = theta2_dot + theta2_ddot * dt
    
    return theta1_new, theta1_dot_new, theta2_new, theta2_dot_new

# Parámetros del doble péndulo
L1 = 1.0  # Longitud del primer péndulo
L2 = 1.0  # Longitud del segundo péndulo
m1 = 1.0  # Masa del primer péndulo
m2 = 1.0  # Masa del segundo péndulo
g = 9.81  # Aceleración debida a la gravedad
dt = 0.01  # Paso de tiempo para el método de Euler

# Condiciones iniciales
theta1 = np.pi / 2
theta2 = np.pi / 2
theta1_dot = 0
theta2_dot = 0

# Listas para almacenar las posiciones de los nodos en cada fotograma
x1_trace, y1_trace = [], []
x2_trace, y2_trace = [], []

# Función para la animación
def animate(i):
    global theta1, theta1_dot, theta2, theta2_dot
    
    # Calcular nuevas posiciones utilizando el método de Euler
    theta1, theta1_dot, theta2, theta2_dot = double_pendulum(theta1, theta1_dot, theta2, theta2_dot, L1, L2, m1, m2, g, dt)
    
    # Calcular posiciones cartesianas
    x1 = L1 * np.sin(theta1)
    y1 = -L1 * np.cos(theta1)
    x2 = x1 + L2 * np.sin(theta2)
    y2 = y1 - L2 * np.cos(theta2)
    
    # Agregar las posiciones de los nodos a las listas de rastro
    x1_trace.append(x1)
    y1_trace.append(y1)
    x2_trace.append(x2)
    y2_trace.append(y2)
    
    # Actualizar la línea y los puntos de rastro en la animación
    line.set_data([0, x1, x2], [0, y1, y2])
    trace1.set_data(x1_trace, y1_trace)
    trace2.set_data(x2_trace, y2_trace)
    return line, trace1, trace2

# Configuración de la figura
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
line, = ax.plot([], [], lw=2)
trace1, = ax.plot([], [], 'o-', lw=1, color='green')  # Rastro del primer nodo (verde)
trace2, = ax.plot([], [], 'o-', lw=1, color='blue')   # Rastro del segundo nodo (azul)

# Creación de la animación
ani = FuncAnimation(fig, animate, frames=range(1, 1000), interval=10, blit=True)

plt.title('Doble Péndulo')
plt.xlabel('Posición x')
plt.ylabel('Posición y')
plt.show()
