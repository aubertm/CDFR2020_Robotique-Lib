import numpy as np

class Robot_static :
    def __init__(self, a_id): # Constructeur
        self.id = a_id
        self.state  = State(0,0,0)
        self.target = State(0,0,0)
        self.reg = Polar_regulator()
        self.dist = 0
        self.width = 23
        self.lenght = 13
        self.sensors = {'front':300,'left':300,'right':300}

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

class Point :
    def __init__(self, x, y): # Constructeur
        self.x = x
        self.y = y
        self.id = 'point'

    def __str__(self): # Built-in print
        return "X: "+repr(round(self.x))+" Y: "+repr(round(self.y))+ '\tID: '+self.id

    def setID(self,str):
        self.id = str

    def getID(self):
        return self.id

    def setXY(self,x,y):
        self.x=x
        self.y=y

    def setPosition(self,x,y):
        self.setXY(x,y)

    def getPosition(self):
        return Point(self.x,self.y)

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def distanceFrom(self, pointB):
        try :
            l_distance = (pointB.getX() - self.x)**2 + (pointB.getY() - self.y)**2
            l_distance = np.sqrt(l_distance)
        except :
            print('Erreur distanceFrom()')
            l_distance = 0
        return l_distance

    def capTo(self,pointB):
        l_cap = np.arctan2(pointB.getY() - self.y, pointB.getX() - self.x)
        return l_cap

class State(Point) :
    def __init__(self,x, y, teta):
        Point.__init__(self,x,y)
        self.angle = teta 
        self.id = 'state'

    def __str__(self):
        return 'Teta : '+repr(round(np.rad2deg(self.angle)))+' '+ Point.__str__(self)

    def setAngle(self,teta):
        self.angle = teta

    def getAngle(self):
        return self.angle

    def setXYT(self,x,y,teta):
        self.setXY(x,y)
        self.angle = teta

class Circle(Point):
    def __init__(self,x, y, r):
        Point.__init__(self,x,y)
        self.r = r 
        self.id = 'circle'

    def __str__(self):
        return 'Rayon : '+repr(self.r)+' '+Point.__str__(self)

    def getRayon(self):
        return self.r

    def contains(self,point):
        return self.distanceFrom(point) < self.r

class Rect(Point):
    def __init__(self,x, y, width, lenght):
        Point.__init__(self,x,y)
        self.id = 'rect'
        self.w = width
        self.l = lenght

    def __str__(self):
        str = 'Width : ' + repr(self.w)+' Lenght : '+repr(self.l)
        str = str + Point.__str__(self)
        return str

    def getWidth(self):
        return self.w

    def getLength(self):
        return self.l

    def contains(self,point):
        upXbound = self.getX()+(self.getWidth()/2)
        dwXbound = self.getX()-(self.getWidth()/2)
        upYbound = self.getY()+(self.getLength()/2)
        dwYbound = self.getY()-(self.getLength()/2)
        inX = (point.getX() < upXbound) and (point.getX() > dwXbound)
        inY = (point.getY() < upYbound) and (point.getY() > dwYbound)

        return inX and inY

def moduloPI(a_angle):
    return 2*np.arctan(np.tan(a_angle/2))