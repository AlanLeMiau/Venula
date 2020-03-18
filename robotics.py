from mpl_toolkits.mplot3d import axes3d

import matplotlib.pyplot as plt
import numpy as np

class Robot(object):
    def __init__(self,l0,l1,l2,l3=0):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def getPosition(self, q1, q2, q3=0):
        # Get position for first two angles (RR robot)
        g  = q2 + q1
        h  = g + q3

        x0 = 0
        y0 = 0
        z0 = self.l0;

        x1 = x0 + self.l1*np.cos(q1 * (np.pi/180))
        y1 = y0 + self.l1*np.sin(q1 * (np.pi/180))
        z1 = z0;
        x2 = x1 + self.l2*np.cos(g * (np.pi/180))
        y2 = y1 + self.l2*np.sin(g * (np.pi/180))
        z2 = z1
        x3 = x2 + self.l3*np.cos(h * (np.pi/180))
        y3 = y2 + self.l3*np.sin(h * (np.pi/180))
        z3 = z2
        x  = [0, x0, x1, x2, x3]
        y  = [0, y0, y1, y2, y3]
        z  = [0, z0 ,z1, z2, z3]

        return (x, y, z)
        

    def draw2D(self, x, y, z=0):
        # Plot the robot at plane
        fig2d = plt.figure()
        axs2d = fig2d.add_subplot(111)
        axs2d.plot([0,1] , [0,0], c='r') # Eje X
        axs2d.plot([0,0] , [0,1], c='g') # Eje Y
        axs2d.plot(x , y , c='k')             # Robot
        plt.show()
        return

    def draw3D(self, x, y, z):
        # Move the robot at space
        fig3d = plt.figure()
        axs3d = fig3d.add_subplot(111, projection='3d')
        axs3d.plot([0,1] , [0,0] , [0,0] , c='r') # Eje X
        axs3d.plot([0,0] , [0,1] , [0,0] , c='g') # Eje Y
        axs3d.plot([0,0] , [0,0] , [0,1] , c='b') # Eje Z
        axs3d.plot(x , y , z , c='k')             # Robot
        plt.show()
        return

    def inverseKinematic(self, x, y, z=0):
        # Inverse Kinematics from RR robot
        h = np.sqrt(y**2 + x**2)
        # print("h:")
        # print(h)
        if (h <= self.l1 + self.l2):
            aa = (self.l1**2 - self.l2**2 + h**2)/(2*self.l1*h)
            # print(aa)
            aa = np.arccos(aa)
            bb = np.arctan2(y,x)
            cc = (self.l1**2 + self.l2**2 - h**2)/(2*self.l1*self.l2)
            # print(cc)
            cc = np.arccos(cc)
            q1 = (bb - aa) * (180/np.pi)
            q2 = (np.pi - cc) * (180/np.pi) 
            # q1 = q1*(180/np.pi)
            # q2 = q2*(180/np.pi)
        else:
            q1 = 0
            q2 = 0

        return (q1,q2)

    def setCam(self, x, y):
        # Move the robot to set the camera in position (first two DOF)
        return self.l0

    def setLaser(self, x, y):
        # Move the robot to set the laser in position (three DOF)
        return self.l0

    def getPose(self, q1, q2, q3=0):
        # Get position for the three angles (RRR robot)
        return self.l0 #return (x,y,z)

    # def __str__(self):
    #     return '<' + str(self.getX()) + ',' + str(self.getY()) + '>'
    # 
    # def __eq__(self, other):
    #     if ((self.x == other.x) and (self.y == other.y)):
    #         return True
    #     else:
    #         return False
    # def __repr__(self):
    #     return 'Coordinate(' + str(self.getX()) + ', ' + str(self.getY()) + ')'

if __name__ == '__main__':

    Scara = Robot(30, 27, 27, 3)

    (q1, q2) = Scara.inverseKinematic(20, 40.5)
    print("q1:")
    print(q1)
    print("q2:")
    print(q2)

    (x, y, z) = Scara.getPosition(q1, q2)
    print("x:")
    print(x)
    print("y:")
    print(y)
    print("z:")
    print(z)

    Scara.draw2D(x, y)

    Scara.draw3D(x, y, z)

