import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

class kinematicModel:
    def __init__ (self, wheel_radius = 0, lx = 0, ly = 0):
        
        self.wheel_radius = wheel_radius
        self.lx = lx
        self.ly = ly
        self.scalar = 1/self.wheel_radius
        self.sum = self.lx + self.ly
        self.forward_kinematics_matrix = np.array([[1, -1, -self.sum],
                                                   [1,  1,  self.sum],
                                                   [1,  1, -self.sum],
                                                   [1, -1,  self.sum]])
    
    def mecanum_4_vel_forward(self, Vx, Vy, W):

        
        input_array = np.array([Vx, Vy, W])

        outside_matrix = np.matmul(self.scalar * self.forward_kinematics_matrix, input_array)

        return outside_matrix




