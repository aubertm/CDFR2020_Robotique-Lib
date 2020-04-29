import numpy as np
import math
from PySide2.QtCore import QRect

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

class Strategy:
    def __init__(self):
        self.elements = dict()
        self.actions = dict()
        self.actionsMade = list()

        self.actionsEnCours = {'MarioBot':'InitMPos','GuiguiBot':'InitGPos'}
        self.gameArea = Rect(0,0,200,300)
        self.teamColor = 'blue'

        self.buildGameStaticMap()

    def __str__(self): # Built-in print
        return 'Strat'

    def buildGameStaticMap(self):
        self.actIM = Action('InitMPos',90,25,np.deg2rad(0))
        self.actIG = Action('InitGPos',70,25,np.deg2rad(180))
        self.act1  = Action('Code Bar',125,150,0)
        self.act2  = Action('Zone Dep',100,277,np.pi/4)
        self.act3  = Action('Manche R',185,236,0)
        self.act4  = Action('Phare',10,23,np.pi/2)

        self.addActionList((self.actIM,self.actIG,self.act1,self.act2,self.act3,self.act4))

    def buildGameContext(self):
        # Ligne de chenal
        self.actIM = Action('InitMPos',90,25,np.deg2rad(0))
        self.actIG = Action('InitGPos',70,25,np.deg2rad(180))
        self.act1  = Action('TAG ArUco',125,150,0)
        self.act2  = Action('Port',100,277,np.pi/4)
        self.act3  = Action('Manche R1',185,236,0)
        self.act3  = Action('Manche R2',185,236,0)
        self.act4  = Action('Phare',10,23,np.pi/2)
        self.act4  = Action('Girouette',10,23,np.pi/2)
        self.act4  = Action('Ecueils S',10,23,np.pi/2)
        self.act4  = Action('Ecueils NO',10,23,np.pi/2)
        self.act4  = Action('Ecueils NE',10,23,np.pi/2)
        self.act4  = Action('Mouillage Sud',10,23,np.pi/2)
        self.act4  = Action('Mouillage Nord',10,23,np.pi/2)

        self.addActionList(())

    def addElement(self, key, value):
        self.elements[key]=value

    def removeElement(self, key):
        self.elements.pop(key)

    def modifyElement(self, key, value):
        self.elements[key]=value

    def containsElement(self, key):
        return key in self.elements.keys()

    def getElementsKeys(self):
        return self.elements.keys()

    def getElement(self, key):
        if key in self.elements.keys():
            return self.elements[key]
        else :
            return 0

    def addAction(self, key, value):
        self.actions[key]=value

    def addActionList(self, actions):
        for l_a in actions :
            self.actions[l_a.getID()]=l_a

    def removeAction(self, key):
        self.actions.pop(key)

    def modifyAction(self, key, value):
        self.actions[key]=value

    def containsAction(self, key):
        return key in self.actions.keys()

    def getActionKeys(self):
        return self.actions.keys()

    def getAction(self, key):
        if key in self.actions.keys():
            return self.actions[key]
        else :
            return 0

    def assignAction(self, action_k, rbt_ID):
        if action_k in self.actions.keys():
            self.actionsEnCours[rbt_ID] = action_k
        else :
            print(f'assignAction : Unknowed Action Key {action_k}')

    def giveMeAnAction(self,rbt_ID):
        l_toDo = [k for k in list(self.actions.keys()) if not(('Init' in k) or ('Manuel' in k) or (k in self.actionsEnCours.values()) )]
        l_key = ''
        if len(l_toDo)>0 :
            l_key = l_toDo[0]
        else :
            l_key = 'Init'+rbt_ID[0]+'Pos'
        return l_key

    def setTargetPoint(self, robot):
        stopNeeded = robot.getSensors()['front'] < 30 
        stopNeeded |= robot.getSensors()['left'] < 30 
        stopNeeded |= robot.getSensors()['right']< 30

        l_currentActionID = self.actionsEnCours[robot.getID()]
        l_toDo = [k for k in list(self.actions.keys()) if not('Init' in k)]
        l_nbToDo = len(l_toDo)

        if stopNeeded :
            a=robot.getPosition()
        else :
            if robot.isArrived() and l_nbToDo > 0 and not('Init' in l_currentActionID):
                print(f"{robot.getID()}\t a terminé l'action {l_currentActionID}.")
                self.actionsMade.append(l_currentActionID)
                self.removeAction(l_currentActionID)
                l_a = self.giveMeAnAction(robot.getID())
                self.assignAction(l_a,robot.getID())
                a = self.actions[self.actionsEnCours[robot.getID()]]
            else :
                a=self.actions[self.actionsEnCours[robot.getID()]]
        robot.setTarget(a.getX(),a.getY(),a.getAngle(),'delay',1)
        return Point(a.getX(),a.getY())

    def findPath(self,init,target):
        l_trajectory = Point(50,50),Point(100,150),Point(50,250)
        return l_trajectory



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
        l_cap = np.arctan2(pointB.getY() - self.y, pointB.getX() - self.x);
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

class PID_regulator :
    def __init__(self, a_kp , a_ki , a_kd): # Constructeur
        self.kp = a_kp
        self.ki = a_ki
        self.kd = a_kd
        self.error = 0
        self.errorSum = 0
        self.errorDt = 0
        self.lastError = 0

    def __str__(self): # Built-in print
        l_config = "kp : "+ repr(self.kp) + " ki : "+ repr(self.ki) + " kd : "+ repr(self.kd)
        return l_config

    def getOutput(self, signal, target):
        self.lastError = self.error
        
        self.error = target - signal
        self.errorSum = self.errorSum + self.error
        self.errorDt = self.lastError - self.error

        l_output = ( self.kp * self.error ) + ( self.ki * self.errorSum ) + ( self.kd * self.errorDt )
        return l_output

def moduloPI(a_angle):
    return 2*np.arctan(np.tan(a_angle/2))

class Action(State): 
    def __init__(self,ID,x,y,teta):
        State.__init__(self,x,y,teta)
        self.id = ID
        self.father = 0
        self.child = list()
        self.done = bool()
        self.pain = int()
        self.gain = int()

    def __str__(self):
        return 'Action : '+self.id

    def isDone(self):
        return self.done

    def toPlan(self, boul):
        self.done = boul

    def getPain(self):
        return self.pain

    def setPain(self, pain):
        self.pain = pain

    def getGain(self):
        return self.gain

    def setGain(self, gain):
        self.price = gain