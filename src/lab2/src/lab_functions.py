# ===============================================================
#	Curso   :   Legged robots
# 	Alumno  :   Jhon Charaja
# 	Info	:	useful functions for laboratory
# ===============================================================

# =============
#   libraries
# =============
import os
import numpy as np
import pinocchio as pin
from copy import copy

# =============
#   functions
# =============
def sinusoidal_reference_generator(q0, a, f, t):
    """
    Info: generates a sine signal.

    Inputs: 
    ------
        - q0: initial joint position
        - a: amplitude [rad]
        - f: frecuency [hz]
        - t: simulation time [sec]
    Outputs:
    -------
        - sinusoidal signal
    """
    w = 2*np.pi*f               # [rad/s]
    q = q0 + a*np.sin(w*t)           # [rad]
    dq = a*w*np.cos(w*t)        # [rad/s]
    ddq = -a*w*w*np.sin(w*t)    # [rad/s^2]

    return q, dq, ddq

def step_reference_generator(q0, a):
    """
    Info: generate a constant reference.

    Inputs:
    ------
        - q0: initial joint position
        - a: constant reference
    Outputs:
    -------
        - constant signal 
    """
    q = q0 + a  # [rad]
    dq = 0      # [rad/s]
    ddq = 0     # [rad/s^2]
    return q, dq, ddq


def tl(array):
    """
    Info: add element elment of list
    """
    return array.tolist()    

class Robot(object):
    """
    Info: Class to load the .urdf of a robot. For thism Pinocchio library is used
    """    
    def __init__(self, q0, dq0, dt, urdf_path):
        self.robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_path)
        self.ndof = self.robot.model.nq
        self.q = copy(q0)                
        self.dq = copy(dq0)               
        self.ddq = np.zeros(self.ndof)
        self.M = np.zeros([self.ndof, self.ndof])
        self.b = np.zeros(self.ndof)
        self.g = np.zeros(self.ndof)
        self.z = np.zeros(self.ndof)
        self.dt = copy(dt)        

    def send_control_command(self, u):
        tau = np.squeeze(np.asarray(u))
        self.M = pin.crba(self.robot.model, self.robot.data, self.q)
        self.b = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, self.z)
        self.g = pin.rnea(self.robot.model, self.robot.data, self.q, self.z, self.z)

        self.ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.dq = self.dq + self.dt*self.ddq
        self.q = self.q + self.dt*self.dq + 0.5*self.dt*self.dt*self.ddq

    def read_joint_position_velocity_acceleration(self):
        return self.q, self.dq, self.ddq

    def get_M(self):
        return self.M

    def get_b(self):
        return self.b
    
    def get_g(self):
        return self.g


def dh(d, theta, a, alpha):
    """
    Info: Computes homogeneous transformation matrix for Denavit-Hartenverg parameters of UR5 robot.

    Inputs:
    ------
        - theta: [rad]
        - alpha: [rad]
        - d: [m]
        - a: [m]
    Output:
    ------
        - T: homogeneous transformation matrix
    """
    T = np.array(
        [[np.cos(theta),    -np.cos(alpha)*np.sin(theta),   +np.sin(alpha)*np.sin(theta),   a*np.cos(theta)],
         [np.sin(theta),    +np.cos(alpha)*np.cos(theta),   -np.sin(alpha)*np.cos(theta),   a*np.sin(theta)],
         [      0      ,            +np.sin(alpha)      ,           +np.cos(alpha)      ,           d      ],
         [      0      ,                    0           ,                   0           ,           1      ]])

    return T

def fkine_ur5(q):
    """
    Info: Computes forward kinematics of UR5 robot.

    Inputs:
    -----
        - q: joint configuration [6x1 rad]
    Outputs:
    -------
        - T: homogenoeus transformation matrix that relates end-effector with base.
    """
    #           d               th              a               alpha
    T01 = dh(0.08916,          +q[0],            0.0,            np.pi/2)
    T12 = dh(    0.0,          +q[1],         -0.425,                0.0)
    T23 = dh(    0.0,          +q[2],         -0.392,                0.0)
    T34 = dh(0.10915,          +q[3],            0.0,            np.pi/2)    
    T45 = dh(0.09465,     np.pi+q[4],            0.0,            np.pi/2)
    T56 = dh( 0.0823,          +q[5],            0.0,                0.0)
    # relate end effector with base link
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    
    #print("T01: ",T01[0:3,3])
    #print("T02: ",T02[0:3,3])
    #print("T03: ",T03[0:3,3])
    #print("T04: ",T04[0:3,3])
    #print("T05: ",T05[0:3,3])
    #print("T06: ",T06[0:3,3])
    return T06


def jacobian_xyz_ur5(q, delta=0.0001):
    """
    Info: Analytic jacobian for cartesian position
    
    Inputs:
    ------
        - q: joint configuration [6x1 rad]
    
    Outputs:
    -------
        - J: analytic jacobian [3x6]
    """
    J = np.zeros((3,6))
    # Initial homogeneous transformation (using q)
    T = fkine_ur5(q)
    for i in range(6):
        dq      = copy(q)
        dq[i]   = dq[i] + delta
        dT      = fkine_ur5(dq)
        J[:,i]  = (dT[0:3,3] - T[0:3,3])/delta
    return J        