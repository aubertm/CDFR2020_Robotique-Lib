from robotic_basics import *
import math


class VirtualRobot :
    def __init__(self, a_id): # Constructeur
        self.id = a_id
        self.reg = Polar_regulator()
        self.target = State(0,0,0)
        self.state  = State(0,0,0)
        self.dist = 0
        self.simuCmd = False
        self.width = 23
        self.lenght = 13
        self.sensors = {'front':0,'left':0,'right':0}

    def __str__(self): # Built-in print
        return self.id

    def setID(self, name):
        self.id = name

    def getID(self):
        return self.id

    def getWidth(self):
        return self.width
        
    def getLength(self):
        return self.width

    def setPosition(self, x , y):
        self.state.setXY(x,y)

    def setAngle(self, teta):
        self.state.setAngle(teta)

    def setStaticState(self, x , y, teta):
        self.state.setXYT(x,y,teta)

    def setTarget(self, x, y, teta, priority,arg):
        self.target.setXYT(x,y,teta)
        self.reg.setPriority(priority,arg)

    def isArrived(self):
        dist_Error = self.getTargetDistance()
        heading_Error = self.getAngle() - self.target.getAngle()
        if dist_Error < 3 and np.cos(heading_Error) > 0.90 :
            boul = True
        else :
            boul = False
        return boul

    def getTarget(self):
        return self.target

    def getTargetDistance(self):
        return self.target.distanceFrom(self.state)

    def getPosition(self):
        return self.state

    def getAngle(self):
        return self.state.angle

    def setSensors(self, mesures):
        self.sensors = mesures.copy()

    def getSensors(self):
        return self.sensors.copy()

    def simulate(self,timeStep_ms):
        self.simuCmd = True

        #l_t = 0
        #l_dt = 100/1000
        l_dt = timeStep_ms/1000
        #while l_t < timeStep_ms:
        l_bigX = np.array([self.state.getX(),self.state.getY(), self.state.getAngle(), self.dist])
        l_dotBigX = self.functionF(self.state,self.reg.getOutput(self.state,self.target))
        l_bigX = l_bigX + l_dotBigX*l_dt

        self.state.setXYT(l_bigX[0],l_bigX[1],l_bigX[2])
        self.dist = l_bigX[3]

        #l_t = l_t + l_dt
        return 0 

    def functionF(self,state, command):
        l_dotX =  command[1]*math.cos(state.angle) # Vitesse sur X
        l_dotY =  command[1]*math.sin(state.angle) # Vitesse sur Y
        l_dotT =  command[0] # Vitesse angulaire
        l_dotD =  command[1] # Vitesse linéique
        l_dotBigX = np.array([l_dotX,l_dotY,l_dotT,l_dotD],float)
        return l_dotBigX

class Polar_regulator :
    def __init__(self): # Constructeur
        self.Kp_angle = 5
        self.Kp_dist = 1

        self.dist2wheels = 22         # Cm
        self.linMax = 80              # Cm  / s
        self.rotMax = np.deg2rad(90)  # rad / s
        
        self.priorities = ('delay','orientation','speedSign')
        self.priority = self.priorities[0]
        self.priority_arg = 1

    def __str__(self): # Built-in print
        return "Robot regulator - Angle and Linear Speed"

    def setPriority(self, priority, arg):
        if priority in self.priorities :
            self.priority = priority
            self.priority_arg = arg
        else :
            self.priority = self.priorities[0]
            print('SetPriority : Unknowed priority')

    def fitToRobot(self,rotS,linS):
        l_rot = rotS / self.rotMax
        l_lin = linS / self.linMax

        polarVect = np.sqrt((l_lin**2) + (l_rot**2)),np.arctan2(l_lin,l_rot)

        if polarVect[0] > np.sqrt(1/2) :
            polarVect = np.sqrt(1/2),polarVect[1]
        else :
            pass

        rotS = polarVect[0]*np.cos(polarVect[1])*self.rotMax
        linS = polarVect[0]*np.sin(polarVect[1])*self.linMax

        return rotS,linS

    def getOutput(self,a_currentState, a_target):
        cmd_vector = np.array([0,0],float)

        errorDistance = a_target.distanceFrom(a_currentState)

        if errorDistance < 2.0 :
            errorAngle = moduloPI(a_target.angle - a_currentState.angle)
            errorDistance = 0
        else :
            errorAngle = moduloPI(a_currentState.capTo(a_target) - a_currentState.angle)

        if self.priority == 'orientation' :
            if np.cos(errorAngle) > 0.95 :
                l_rotS = self.Kp_angle * errorAngle
                l_linS = self.Kp_dist  * errorDistance
            else :
                if np.sign(errorAngle) == np.sign(self.priority_arg):
                    l_rotS = self.Kp_angle * errorAngle
                else :
                    l_rotS = self.Kp_angle * (np.abs(errorAngle)+self.priority_arg*2*np.pi)
                l_linS = 0
        elif self.priority == 'speedSign' :
            if errorDistance < 2.0/100 :
                l_rotS = self.Kp_angle * errorAngle
            else :
                l_rotS = self.Kp_angle * moduloPI(errorAngle + np.pi*((1-self.priority_arg)/2))
            l_linS = self.Kp_dist  * errorDistance * self.priority_arg
        else : # Shorter way
            if np.abs(errorAngle) > np.pi/2 and errorDistance > 2/100:
                # marche arrière
                l_rotS = self.Kp_angle * moduloPI(errorAngle + np.pi)
                l_linS = self.Kp_dist  * errorDistance * -1
            else:
                l_rotS = self.Kp_angle * errorAngle
                l_linS = self.Kp_dist  * errorDistance
        
        cmd_vector[0], cmd_vector[1] = self.fitToRobot(l_rotS,l_linS) # Commande rot Speed, lin Speed

        return cmd_vector