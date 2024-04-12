#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division  # Importar división de punto flotante para compatibilidad con Python 2.x
import rospy
import numpy as np
from std_msgs.msg import Float32

global theta1, theta1_dot, theta2, theta2_dot

# Definición de las ecuaciones diferenciales del doble péndulo
def double_pendulum(theta1, theta1_dot, theta2, theta2_dot, L1, L2, m1, m2, g, dt):
    #Derivadas de las variables
    theta1_ddot = (-g*(2*m1 + m2)*np.sin(theta1) - m2*g*np.sin(theta1 - 2*theta2) - 2*np.sin(theta1 - theta2)*m2*(theta2_dot**2*L2 + theta1_dot**2*L1*np.cos(theta1 - theta2))) / (L1*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2)))
    theta2_ddot = (2*np.sin(theta1 - theta2)*(theta1_dot**2*L1*(m1 + m2) + g*(m1 + m2)*np.cos(theta1) + theta2_dot**2*L2*m2*np.cos(theta1 - theta2))) / (L2*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2)))

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
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rospy.init_node('Pendulo_Transmitter')
            pubX = rospy.Publisher('/PosX', Float32, queue_size=10)
            pubY = rospy.Publisher('/PosY', Float32, queue_size=10)
            pubTheta = rospy.Publisher('/PosTheta', Float32, queue_size=10)
            pubX1 = rospy.Publisher('/PosX1', Float32, queue_size=10)
            pubY1 = rospy.Publisher('/PosY1', Float32, queue_size=10)
            pubTheta1 = rospy.Publisher('/PosTheta1', Float32, queue_size=10)
            rospy.loginfo('El nodo transmisor se ha iniciado')
            rate = rospy.Rate(10)  # Frecuencia de publicacin de 10 Hz

    
    
    # Calcular nuevas posiciones utilizando el método de Euler
            theta1, theta1_dot, theta2, theta2_dot = double_pendulum(theta1, theta1_dot, theta2, theta2_dot, L1, L2, m1, m2, g, dt)
            
            # Calcular posiciones cartesianas
            x1 = L1 * np.sin(theta1)
            y1 = -L1 * np.cos(theta1)
            x2 = x1 + L2 * np.sin(theta2)
            y2 = y1 - L2 * np.cos(theta2)
            
            # Agregar las posiciones de los nodos a las listas de rastro
            pubX.publish(x1)
            pubY.publish(y1)
            pubTheta.publish(theta1)
            pubX1.publish(x2)
            pubY1.publish(y2)
            pubTheta1.publish(theta2)
            rate.sleep()
                
        rospy.loginfo('El nodo se ha detenido')        
    except rospy.ROSInterruptException:
        pass

