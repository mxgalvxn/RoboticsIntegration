from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import matplotlib.pyplot as plt

# Definir manualmente la matriz de covarianza del ruido de proceso Q
dim_Q = 6
dt = 0.1
variance = 0.13

Q = np.zeros((dim_Q, dim_Q))
Q[0:3, 0:3] = Q_discrete_white_noise(dim=3, dt=dt, var=variance)
Q[3:6, 3:6] = np.eye(3) * variance

# Inicializar el filtro de Kalman para un robot en tres dimensiones
f = KalmanFilter(dim_x=dim_Q, dim_z=3)  # 6 dimensiones en el estado (posición x, posición y, orientación theta, velocidad en x, velocidad en y, velocidad angular) y 3 dimensiones en la medida (posición x, posición y, orientación)

# Configurar el estado inicial del robot
f.x = np.array([[2.],    # posición inicial en x
                [1.],    # posición inicial en y
                [0.],    # orientación inicial
                [0.],    # velocidad inicial en x
                [0.],    # velocidad inicial en y
                [0.]])   # velocidad angular inicial

# Definir las matrices de transición de estado y de medición
dt = 0.1  # intervalo de tiempo
f.F = np.array([[1., 0., 0., dt, 0., 0.],
                [0., 1., 0., 0., dt, 0.],
                [0., 0., 1., 0., 0., dt],
                [0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 1.]])  

f.H = np.array([[1., 0., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0., 0.],
                [0., 0., 1., 0., 0., 0.]])  

# Configurar las matrices de covarianza
f.P = np.eye(dim_Q) * 1000.0  # Covarianza del error de estimación
f.R = np.eye(3) * 5.0      # Covarianza del error de medición
#f.Q = Q_discrete_white_noise(dim=dim_Q, dt=dt, var=0.13)  # Covarianza del ruido de proceso
f.Q = Q



# Realizar la estimación del estado del robot
iterator = 2
positions_x = []  # Lista para almacenar las posiciones estimadas en x
positions_y = []  # Lista para almacenar las posiciones estimadas en y
orientations = []  # Lista para almacenar las orientaciones estimadas
measurements_x = []  # Lista para almacenar las medidas simuladas en x
measurements_y = []  # Lista para almacenar las medidas simuladas en y
measurements_theta = []  # Lista para almacenar las medidas simuladas de orientación

while iterator < 10:
    # Generar medidas simuladas
    true_measurement = np.random.normal(loc=iterator, scale=0.1, size=(3, 1))
    measurements_x.append(true_measurement[0])
    measurements_y.append(true_measurement[1])
    measurements_theta.append(true_measurement[2])
    
    # Estimar el estado del robot con el filtro de Kalman
    f.predict()
    f.update(true_measurement)
    positions_x.append(f.x[0, 0])  # Guardar la posición estimada en x
    positions_y.append(f.x[1, 0])  # Guardar la posición estimada en y
    orientations.append(f.x[2, 0])  # Guardar la orientación estimada
    iterator += 1

# Graficar las posiciones estimadas y las medidas simuladas con respecto al tiempo
plt.figure(figsize=(10, 6))
plt.plot(range(2, 10), positions_x, marker='o', label='Posición estimada en x')
plt.plot(range(2, 10), positions_y, marker='o', label='Posición estimada en y')
plt.plot(range(2, 10), orientations, marker='o', label='Orientación estimada')
plt.plot(range(2, 10), measurements_x, marker='x', linestyle='--', label='Medida simulada en x')
plt.plot(range(2, 10), measurements_y, marker='x', linestyle='--', label='Medida simulada en y')
plt.plot(range(2, 10), measurements_theta, marker='x', linestyle='--', label='Medida simulada de orientación')
plt.xlabel('Iteración')
plt.ylabel('Posición / Orientación')
plt.title('Posición y orientación estimadas vs. Medidas simuladas vs. Iteración')
plt.grid(True)
plt.legend()
plt.show()
