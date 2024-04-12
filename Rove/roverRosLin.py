#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from std_msgs.msg import Float32
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
    v_eq = (v_max + 0.4 * np.pi * wheelbase) / 2
    w_eq = (v_max - 0.4 * np.pi * wheelbase) / wheelbase
    return v_eq, w_eq

def plot_path(path):
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    plt.plot(x_values, y_values, '-o', markersize=3)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Carrito')
    plt.grid(True)
    plt.axis('equal')

def plot_orientation(theta_values):
    plt.plot(theta_values, '-')
    plt.xlabel('Tiempo (segundos)')
    plt.ylabel('Theta (radianes)')
    plt.title('Orientacion del Carrito Pioneer')
    plt.grid(True)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            x = 0.0
            y = 0.0
            theta = 0.0
            path = [(x, y)]

            rospy.init_node('transmitter')
            pubX = rospy.Publisher('/PosX', Float32, queue_size=10)
            pubY = rospy.Publisher('/PosY', Float32, queue_size=10)
            pubTheta = rospy.Publisher('/PosTheta', Float32, queue_size=10)
            rospy.loginfo('El nodo transmisor se ha iniciado')

            rate = rospy.Rate(10)  # Frecuencia de publicacin de 10 Hz

            for t in np.arange(0, 100, dt):
                v, w = control(t)
                v_eq, w_eq = linearized_control(t)

                v_left = v_eq - w_eq * wheelbase / 2
                v_right = v_eq + w_eq * wheelbase / 2
                x += (v_left + v_right) / 2 * np.cos(theta) * dt
                y += (v_left + v_right) / 2 * np.sin(theta) * dt
                theta += w_eq * dt

                path.append((x, y))
                pubX.publish(x)
                pubY.publish(y)
                pubTheta.publish(theta)

                #plt.clf()
                #plot_path(path)
                #plot_orientation(theta)
                plt.pause(dt)

                rate.sleep()

            rospy.loginfo('El nodo se ha detenido')
    except rospy.ROSInterruptException:
        pass
