import numpy as np

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