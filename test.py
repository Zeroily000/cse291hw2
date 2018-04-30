##import numpy as np
##import cv2
##from pyzbar.pyzbar import decode
##
##cap = cv2.VideoCapture(0)
##
##while(True):
##    # Capture frame-by-frame
##    ret, frame = cap.read()
##
##    # Display the resulting frame
##    cv2.imshow('frame', frame)
##    print decode(frame)
##    if cv2.waitKey(1) & 0xFF == ord('q'):
##        break
##
### When everything done, release the capture
##cap.release()
##cv2.destroyAllWindows()

import picar
import time
import numpy as np

picar.setup()

L = 1
dt = 0.1
FORWARD = 92.23
class Robot:
    def __init__(self):
        self.fw = picar.front_wheels.Front_Wheels()
        self.bw = picar.back_wheels.Back_Wheels()
        self.fw.ready()
        self.bw.ready()
        
        self.fw.turn(FORWARD)
        self.bw.speed = 0
        
        self.dir = 0
        
##    def turnStraight(self):
##        self.fw.turn(93)
        
    def toPolar(self, X):
        x, y, theta = X
        
        rho = np.linalg.norm(X[:2])
        
        if self.dir == 0:
            beta = -np.arctan2(-y, -x)
            alpha = -theta - beta
            
            if alpha > np.pi/2 or alpha < -np.pi/2:
                self.dir = -1
            else:
                self.dir = 1
                
        elif self.dir == -1:
            beta = -np.arctan2(y, x)
            alpha = -theta - beta
        
        else:
            beta = -np.arctan2(-y, -x)
            alpha = -theta - beta
        
        alpha = np.clip(alpha, -np.pi/2, np.pi/2)
        return rho, alpha, beta
    
    def move(self, Xc, v, gamma, theta):
        v = np.clip(v, 20./60., 100./60.)
        gamma = np.clip(gamma, -30*np.pi/180, 30*np.pi/180)
        
        Xc += np.array([np.cos(theta), np.sin(theta), np.tan(gamma)/L]) * self.dir * v * dt
##        print X
        self.fw.turn(FORWARD - gamma*180/np.pi)
        self.bw.speed = int(v*60)
        if self.dir == -1:
            self.bw.forward()
        else:
            self.bw.backward()
        time.sleep(dt*5/12)
    
    def stop(self):
        self.fw.turn(FORWARD)
        self.bw.stop()
        
    def drivePose(self, X0, Xg, krho = 1, kalpha = 5, kbeta = -2):
        Xc = X0 * 1.
        Xd = np.array([Xg[0], Xg[1], 0.])
        
        rho, alpha, beta = self.toPolar(Xc - Xd)
        
        while rho > 0.08:
            
            v = krho * rho
##            print v
            omega = kalpha*alpha + kbeta*beta
            gamma = np.arctan(omega * L / v)
            
            
            self.move(Xc, v, gamma, Xc[2])
            
##            print Xc
            
            rho, alpha, beta = self.toPolar(Xc - Xd)
            beta += Xg[2]
            print np.clip(gamma, -30*np.pi/180, 30*np.pi/180)*180/np.pi, rho, alpha, beta
            
        self.stop()
            
            
        
        
        
    


# bw.speed(50) x time.sleep(3) = 1m
##def main():
##    fw.turn(93)
##    bw.speed = 50
##    time.sleep(3)
##    destroy()
##    car.drivePose(np.array([0, 0, 0]), np.array([1, 0, 0]))
    
def destroy():
    fw.turn(93)
    bw.stop()
    
if __name__ == '__main__':
    car = Robot()
    try:
##        car.drivePose(np.array([0, 0, 0]), np.array([5, 0, 0]))
##        car.drivePose(np.array([5, 0, 0]), np.array([10, 10, np.pi]))
        car.drivePose(np.array([10, 10, np.pi]), np.array([0, 0, 0]))
    except :
        car.stop()
 