from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import cv2          
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

    def info(self):
        print(self.l0)
        print(self.l1)
        print(self.l2)
        print(self.l3)

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
            plt.text(0, y[len(y)-1],'(x: %.6s , y: %.6s )\n q1: %.6s \n q2: %.6s \n q3: %.6s'%(x[len(x)-1],y[len(y)-1], q1, q2, q3))
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

def nothing(x):
    # Scara.info()
    global screen
    screen[:] = 255
    (h, w, l) = screen.shape
    (h, w) = (int(h/2), int(w/2))
    # Draw X Axis
    screen = cv2.line(screen,(w-w,h),(w+w,h),(0,0,255),1)
    # Draw Y Axis
    screen = cv2.line(screen,(w,h-h),(w,h+h),(0,255,0),1)
    
def main():
    # Create a white image, and a window
    global screen
    screen = np.zeros((600,800,3), np.uint8)
    # White screen
    screen[:] = 255
    (h, w, l) = screen.shape
    (h, w) = (int(h/2), int(w/2))

    cv2.namedWindow('Robot User Interface')

    # create trackbars for angle change
    TrackbarLimit = 360
    cv2.createTrackbar('Q1','Robot User Interface',0,TrackbarLimit,nothing)
    cv2.createTrackbar('Q2','Robot User Interface',0,TrackbarLimit,nothing)
    cv2.createTrackbar('Q3','Robot User Interface',0,TrackbarLimit,nothing)

    q1 = cv2.setTrackbarPos('Q1','Robot User Interface',int(TrackbarLimit/2))
    q2 = cv2.setTrackbarPos('Q2','Robot User Interface',int(TrackbarLimit/2))
    q3 = cv2.setTrackbarPos('Q3','Robot User Interface',int(TrackbarLimit/2))

    # Create a object Robot
    Scara = Robot(30, 27, 27, 3)

    while(1):
        # get current positions of trackbars
        q1 = cv2.getTrackbarPos('Q1','Robot User Interface') - TrackbarLimit/2
        q2 = cv2.getTrackbarPos('Q2','Robot User Interface') - TrackbarLimit/2
        q3 = cv2.getTrackbarPos('Q3','Robot User Interface') - TrackbarLimit/2

        # get current positions for Robot
        (x, y, z) = Scara.getPosition(q1, q2, q3)

        # Draw a diagonal blue line with thickness of 5 px
        for i in range(0,len(z)-1):
            f = 5 # Scale factor for Robot dimensions and screen
            x1 = w + int(f*x[i])
            y1 = h - int(f*y[i])
            x2 = w + int(f*x[i+1])
            y2 = h - int(f*y[i+1])
            screen = cv2.line(screen,(x1, y1),(x2, y2),(255,0,0),2)

        # Write the info about the Robot configuration
        font = cv2.FONT_HERSHEY_PLAIN
        # Robot position
        label = '(x: %.6s , y: %.6s )'%(x[len(x)-1],y[len(y)-1])
        cv2.putText(screen, label,(50,70), font, 1,(0,0,0),1,cv2.LINE_8)
        # Robot configuration (angles)
        label = 'q1: %.6s'%(q1)
        cv2.putText(screen, label,(50,90), font, 1,(0,0,0),1,cv2.LINE_8)
        label = 'q2: %.6s'%(q2)
        cv2.putText(screen, label,(50,110), font, 1,(0,0,0),1,cv2.LINE_8)
        label = 'q3: %.6s'%(q3)
        cv2.putText(screen, label,(50,130), font, 1,(0,0,0),1,cv2.LINE_8)

        cv2.imshow('Robot User Interface',screen)
        if (cv2.waitKey(1) == 27):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
