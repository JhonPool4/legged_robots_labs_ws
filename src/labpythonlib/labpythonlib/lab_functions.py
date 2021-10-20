# ===============================================
#	Course  :   legged robots
# 	Alumno  :   jhon charaja
# 	Info	:	useful functions for laboratory
# ===============================================

# ======================
#   required libraries
# ======================
import os
import numpy as np
import pinocchio as pin
from copy import copy

# =============
#   functions
# =============
def sinusoidal_reference_generator(q0, a, f, t_change, t):
    """
    Info: generates a sine signal.

    Inputs: 
    ------
        - q0: initial joint/cartesian position
        - a: amplitude
        - f: frecuency [hz]
        - t_change: change from sinusoidal to constant reference [sec]
        - t: simulation time [sec]
    Outputs:
    -------
        - q, dq, ddq: joint/carteisan position, velocity and acceleration
    """
    w = 2*np.pi*f               # [rad/s]
    if t<=t_change:
        q = q0 + a*np.sin(w*t)      # [rad]
        dq = a*w*np.cos(w*t)        # [rad/s]
        ddq = -a*w*w*np.sin(w*t)    # [rad/s^2]
    else:
        q = q0 + a*np.sin(w*t_change)   # [rad]
        dq = 0                          # [rad/s]
        ddq = 0                         # [rad/s^2]
    return q, dq, ddq

def step_reference_generator(q0, a, t_step, t):
    """
    Info: generate a constant reference.

    Inputs:
    ------
        - q0: initial joint/cartesian position
        - a: constant reference
        - t_step: start step [sec]
        - t: simulation time [sec]
    Outputs:
    -------
        - q, dq, ddq: joint/carteisan position, velocity and acceleration
    """
    if t>=t_step:
        q = q0 + a  # [rad]
        dq = 0      # [rad/s]
        ddq = 0     # [rad/s^2]
    else:
        q = copy(q0)    # [rad]
        dq = 0          # [rad/s]
        ddq = 0         # [rad/s^2]            
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
        # robot object
        self.robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_path)
        # degrees of freedom
        self.ndof = self.robot.model.nq
        # joint configuration: position, velocity and acceleration
        self.q = copy(q0)                
        self.dq = copy(dq0)               
        self.ddq = np.zeros(self.ndof)
        # inertia matrix
        self.M = np.zeros([self.ndof, self.ndof])
        # nonlinear effects vector
        self.b = np.zeros(self.ndof)
        # gravivty effects vector
        self.g = np.zeros(self.ndof)
        # vector of zeros
        self.z = np.zeros(self.ndof)
        # sampling time
        self.dt = copy(dt)     
        # frame id: end-effector
        self.frame_ee = self.robot.model.getFrameId('ee_link') 
        # end-effector: position, velocity and acceleration
        self.p = np.zeros(3)
        self.dp = np.zeros(3)
        self.ddp = np.zeros(3)
        # end-effector: orientation
        self.R = np.zeros([3,3])
        # initial configuration: position (p) and orientation (R)
        self.p, self.R = self.forward_kinematics(self.q)
        # initial configuration: dynamic model
        self.M = pin.crba(self.robot.model, self.robot.data, self.q)
        self.b = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, self.z)
        self.g = pin.rnea(self.robot.model, self.robot.data, self.q, self.z, self.z)        
  

    def forward_kinematics(self, q0):
        """
        Info: computes the position (xyz) and rotation (R) of the end-effector.

        Inputs:
        -----
            - q0: joint configuration (rad)
        
        Outputs:
        -------
            - p: position of the end-effector (m).
            - R: rotation matrix of the end-effector (rad).
        """      
        # commpute forward kinematics
        pin.forwardKinematics(self.robot.model, self.robot.data, q0) 
        # get position and orientation       
        p = pin.updateFramePlacement(self.robot.model, self.robot.data, self.frame_ee).translation
        R = pin.updateFramePlacement(self.robot.model, self.robot.data, self.frame_ee).rotation
        return p, R

    def jacobian(self, q0):
        """
        Info: computes jacobian matrix of the end-effector.

        Inputs:
        ------
            - q0: joint configuration (rad)
        Outputs:
        -------
            - J: jacobian matrix            
        """
        # compute jacobian matrix (end-effector frame)
        pin.computeJointJacobians(self.robot.model, self.robot.data, q0)
        J = pin.getFrameJacobian(self.robot.model, self.robot.data, self.frame_ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J
    
    def jacobian_time_derivative(self, q0, dq0):
        """
        Info: computes time derivative of jacobian matrix of the end-effector.

        Inputs:
        ------
            - q0: joint position/configuration (rad)
            - dq0: joint velocity (rad/s)
        Outputs:
        -------
            - dJ: time derivative of jacobian matrix            
        """        
        # compute time-derivative of jacobian matrix (end-effector frame)
        pin.computeJointJacobiansTimeVariation(self.robot.model, self.robot.data, q0, dq0)
        dJ = pin.getFrameJacobianTimeVariation(self.robot.model, self.robot.data, self.frame_ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return dJ

    def jacobian_damped_pinv(self, J, lambda_=0.0000001):
        """
        Info: computes inverse jabobian using damped pseudo-inverse method

        Inputs:
        ------
            - J: position jacobian [3 x ndof]
            - lambda_ : damping term (optional)
        Outputs:
        -------
            - J_damped_inv: inverse of jacobian matrix            
        """
        J_damped_inv =  np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + lambda_*np.eye(3)))
        return J_damped_inv

    def send_control_command(self, u):
        """
        Info: uses the control signal (u) to compute forward dynamics (ddq). 
              Then update joint configuration (q) and end-effector pose (p, R)
        """
        tau = np.squeeze(np.asarray(u))
        # compute dynamics model
        self.M = pin.crba(self.robot.model, self.robot.data, self.q)
        self.b = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, self.z)
        self.g = pin.rnea(self.robot.model, self.robot.data, self.q, self.z, self.z)
        # forward dynamics
        self.ddq = np.linalg.inv(self.M).dot(tau-self.b)
        # update joint position/configuration
        self.dq = self.dq + self.dt*self.ddq
        self.q = self.q + self.dt*self.dq + 0.5*self.dt*self.dt*self.ddq
        # compute new jacobians
        J = self.jacobian(self.q)[0:3, 0:self.ndof] # position xyz
        dJ = self.jacobian_time_derivative(self.q, self.dq)[0:3, 0:self.ndof] # position xyz
        # update end-effector: position, velocity, acceleration and orientation
        self.p, self.R = self.forward_kinematics(self.q)
        self.dp = np.dot(J, self.dq)
        self.ddp = np.dot(J, self.ddq) + np.dot(dJ, self.dq)

    def inverse_kinematics_position(self, x_des, q0):
        """
        @info: computes inverse kinematics with the method of damped pseudo-inverse.

        @inputs:
        -------
            - xdes  :   desired position vector
            - q0    :   initial joint configuration (it's very important)
        @outputs:
        --------        
            - q_best  : joint position
        """         
        best_norm_e     = 1e-6 
        max_iter        = 10
        delta           = 1
        lambda_         = 0.0000001
        q               = copy(q0)

        for i in range(max_iter):
            p, _ = self.forward_kinematics(q) # current position
            e   = x_des - p      # position error
            J   = self.jacobian(q)[0:3, 0:self.ndof] # position jacobian [3x6]
            J_damped_inv =  self.jacobian_damped_pinv(J, lambda_) # inverse jacobian [6x3]
            dq  = np.dot(J_damped_inv, e)
            q   = q + delta*dq
                       
            # evaluate convergence criterion
            if (np.linalg.norm(e)<best_norm_e):
                best_norm_e = np.linalg.norm(e)
                q_best = copy(q) 
        return q_best 

    def read_joint_position_velocity_acceleration(self):
        return self.q, self.dq, self.ddq

    def read_cartesian_position_velocity_acceleration(self):
        return self.p, self.dp, self.ddp

    def get_ee_position(self):
        return self.p
    
    def get_ee_orientation(self):
        return self.R

    def get_M(self):
        return self.M

    def get_b(self):
        return self.b
    
    def get_g(self):
        return self.g















# currenty not neccesary
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
    Info: Computes forward kinematics of UR5 robot. With respect to base frame

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

