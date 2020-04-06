from mpl_toolkits.mplot3d import axes3d

import matplotlib.pyplot as plt
import numpy as np

class Robot(object):
    def __init__(self,l0,l1,l2,l3=0):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        # self.q1 = 0
        # self.q2 = 0
        # self.q3 = 0

        # self.x = l1+l2+l3
        # self.y = 0
        # self.z = l0

    def getPosition(self, q1, q2, q3=0):
        # Get position for first two angles (RR robot)
        g  = q2 + q1
        h  = g + q3

        x0 = 0
        y0 = 0
        z0 = self.l0;
        x1 = x0 + self.l1*np.cos(q1 * (np.pi/180))
        y1 = y0 + self.l1*np.sin(q1 * (np.pi/180))
        z1 = z0
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

    def draw2D(self, mode, a, b, c):
        fig2d = plt.figure()
        axs2d = fig2d.add_subplot(111)
        axs2d.plot([0,1] , [0,0], c='r') # Eje X
        axs2d.plot([0,0] , [0,1], c='g') # Eje Y
        
        if(mode == "position"):
            (x, y, z) = (a, b, c)
            (aa, bb, cc) = (a[len(a)-1], b[len(b)-1], c[len(c)-1]) 
            (q1, q2, q3) = self.inverseKinematic(aa, bb, cc)
            error = 0
        elif(mode == "angles"):
            (q1, q2, q3) = (a, b, c)
            (x, y, z) = self.getPosition(a, b, c)
            error = 0
        else: 
            error = 1

        if error:
            return error
        else:
            # Plot the robot at plane
            axs2d.plot(x , y , c='k')        # Robot
            plt.text(0, y[len(y)-1],'(x: %s , y: %s )\n q1: %s \n q2: %s \n q3: %s'%(x[len(x)-1],y[len(y)-1], q1, q2, q3))
            plt.show()
            return error

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

    def inverseKinematic(self, xf, yf, zf=0):
        # Inverse Kinematics from RR robot
        hf = np.sqrt(yf**2 + xf**2)

        if (hf <= self.l1 + self.l2 + self.l3):
            ee = np.arctan2(yf,xf)

            x = xf - self.l3*np.cos(ee)
            y = yf - self.l3*np.sin(ee)
            h = np.sqrt(y**2 + x**2)

            aa = (self.l1**2 - self.l2**2 + h**2)/(2*self.l1*h)
            aa = np.arccos(aa)
            bb = np.arctan2(y,x)
            cc = (self.l1**2 + self.l2**2 - h**2)/(2*self.l1*self.l2)
            cc = np.arccos(cc)

            q1 = (bb - aa) * (180/np.pi)
            q2 = (np.pi - cc) * (180/np.pi) 
            q3 = ee* (180/np.pi) - (q1 + q2)

        else:
            q1 = 0
            q2 = 0
            q3 = 0

        return (q1, q2, q3)

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

def main():
    Scara = Robot(30, 27, 27, 3)

    (q1, q2, q3) = Scara.inverseKinematic(30, 10)
    print("q1:")
    print(q1)
    print("q2:")
    print(q2)
    print("q3:")
    print(q3)

    (x, y, z) = Scara.getPosition(q1, q2, q3)
    print("x:")
    print(x)
    print("y:")
    print(y)
    print("z:")
    print(z)

    # Scara.draw2D("position", x, y, z)
    Scara.draw2D("angles", q1, q2, q3)
    # Scara.draw3D(x, y, z)

if __name__ == '__main__':
    main()
