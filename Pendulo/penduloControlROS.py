import rospy
from std_msgs.msg import Float64
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def single_pendulum(t, y, L, m, g, Kp, Kd):
    theta, theta_dot = y
    theta_ddot = (-g*np.sin(theta)) / L
    theta_eq = 0.0
    theta_dot_eq = 0.0
    u = Kp * (theta_eq - theta) + Kd * (theta_dot_eq - theta_dot)
    theta_ddot += u
    return [theta_dot, theta_ddot]

L = 1.0
m = 1.0
g = 9.81

theta_0 = np.pi / 2
theta_dot_0 = 0
y0 = [theta_0, theta_dot_0]

Kp = 100.0
Kd = 20.0

x_trace, y_trace = [], []

def animate(i):
    t_span = (0, i / 10)
    t_eval = np.linspace(0, i / 10, 100)
    sol = solve_ivp(single_pendulum, t_span, y0, args=(L, m, g, Kp, Kd), t_eval=t_eval)
    theta = sol.y[0]
    x = L * np.sin(theta)
    y = -L * np.cos(theta)
    x_trace.append(x[-1])
    y_trace.append(y[-1])
    line.set_data([0, x[-1]], [0, y[-1]])
    trace.set_data(x_trace, y_trace)
    return line, trace

def pd_control(theta, theta_dot):
    theta_eq = 0.0
    theta_dot_eq = 0.0
    u = Kp * (theta_eq - theta) + Kd * (theta_dot_eq - theta_dot)
    return u

def theta_callback(data):
    global theta_0
    theta_0 = data.data

def theta_dot_callback(data):
    global theta_dot_0
    theta_dot_0 = data.data

def listener():
    rospy.init_node('pendulum_controller', anonymous=True)
    rospy.Subscriber("theta", Float64, theta_callback)
    rospy.Subscriber("theta_dot", Float64, theta_dot_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    line, = ax.plot([], [], lw=2)
    trace, = ax.plot([], [], 'o-', lw=1, color='green')

    ani = FuncAnimation(fig, animate, frames=range(1, 1000), interval=10, blit=True)

    plt.title('Péndulo con Control PD (ROS)')
    plt.xlabel('Posición x')
    plt.ylabel('Posición y')
    plt.show()
