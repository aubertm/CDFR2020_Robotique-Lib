import numpy as np
from robotic_basics import Point, Rect, State

class Strategy:
    def __init__(self):
        # Element de jeux
        self.gameArea = Rect(0,0,200,300)
        self.elements = dict()
        self.actions = dict()
        self.actionsMade = list()

        self.robots = list()

        self.actionsEnCours = {'MarioBot':'InitMPos','GuiguiBot':'InitGPos'}
        self.teamColor = 'blue'

        self.buildGameStaticMap()

    def __str__(self): # Built-in print
        return 'Strat'

    def addRobot(self, rbt):
        self.robots.append(rbt)

    def buildGameStaticMap(self):
        l_a1 = Action('InitMPos',90,25,np.deg2rad(0))
        l_a2 = Action('InitGPos',70,25,np.deg2rad(180))
        l_a3 = Action('Code Bar',125,150,0)
        l_a4 = Action('Zone Dep',100,277,np.pi/4)
        l_a5 = Action('Manche R',185,236,0)
        l_a6 = Action('Phare',10,23,np.pi/2)

        self.addActionList((l_a1,l_a2,l_a3,l_a4,l_a5,l_a6))

    # def buildGameContext(self):
    #     # Ligne de chenal à ne pas oublier
    #     l_a1  = Action('InitMPos',90,25,np.deg2rad(0))
    #     l_a2  = Action('InitGPos',70,25,np.deg2rad(180))
    #     l_a3  = Action('TAG ArUco',125,150,0)
    #     l_a4  = Action('Port',100,277,np.pi/4)
    #     l_a5  = Action('Manche R1',185,236,0)
    #     l_a6  = Action('Manche R2',185,236,0)
    #     l_a7  = Action('Phare',10,23,np.pi/2)
    #     l_a8  = Action('Girouette',10,23,np.pi/2)
    #     l  = Action('Ecueils S',10,23,np.pi/2)
    #     self.act4  = Action('Ecueils NO',10,23,np.pi/2)
    #     self.act4  = Action('Ecueils NE',10,23,np.pi/2)
    #     self.act4  = Action('Mouillage Sud',10,23,np.pi/2)
    #     self.act4  = Action('Mouillage Nord',10,23,np.pi/2)

    #     self.addActionList(())

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

    def getElements(self):
        return self.elements

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
        l_toDo = [k for k in list(self.actions.keys()) if not(('Init' in k) or ('Manuel' in k) or 
                                                            (k in self.actionsEnCours.values()))] 
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
        robot.setTarget(a.getX(),a.getY(),a.getAngle(),'delay',1)

    def findPath(self,init,target):
        l_trajectory = Point(50,50),Point(100,150),Point(50,250)
        return l_trajectory

    def beat(self):
        for rsc in self.robots:
            self.setTargetPoint(rsc)

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