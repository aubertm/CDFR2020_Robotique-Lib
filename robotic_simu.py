import math
import numpy as np
from robotic_basics import Robot_static, Point, moduloPI



class VirtualRobot(Robot_static) :
    def __init__(self, a_id): # Constructeur
        Robot_static.__init__(self,a_id)

    def updateState(self,timeStep_ms):
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

    def updateSensors(self, elements):
        ### Construction de la matrice de transformation
        tx = -self.getPosition().getX() # Translation sur axe X
        ty = -self.getPosition().getY() # Translation sur axe Y
        Mtr = np.array([ [1,0,tx],
                            [0,1,ty],
                            [0,0,1] ])

        ### Initialisation des données senseurs
        data = {'front':300,'left':300,'right':300}

        ### Calcul des valeurs mesurées
        """ 
        On calcul pour chaque element son influence sur la valeur mesuree
        on conserve l'influence de l'element qui coupe le faisseau en premier.
        """
        for elt in elements:
            ### Placer l'elt dans le référentiel du robot
            Vori = np.array([elt.getX(),elt.getY(),1])
            Vrob = np.dot(Mtr,np.transpose(Vori))
            l_elt_pos = Point(Vrob[0],Vrob[1])

            ### Les rectangles sont approximés à des cercles de rayon r
            if 'rect' in elt.getID():
                r = np.max((elt.getWidth(),elt.getLength()))/2
            elif 'circle' in elt.getID():
                r = elt.getRayon()
            else :
                r = 0

            ### Distance centre à centre
            dist_p2p = self.getPosition().distanceFrom(elt.getPosition())

            ### Prise en compe de l'orientation du robot
            l_e =  moduloPI( self.getAngle() - Point(0,0).capTo(l_elt_pos) )

            a_f = l_e
            a_l = l_e + np.deg2rad(20)
            a_r = l_e - np.deg2rad(20)

            ### Distance entre l'element et la droite vue par le capteur
            dist_p2l_f = np.abs(dist_p2p*np.tan(a_f))
            dist_p2l_l = np.abs(dist_p2p*np.tan(a_l))
            dist_p2l_r = np.abs(dist_p2p*np.tan(a_r))

            ### On calcul si l'element est dans le champ du capteur
            d_f = d_l = d_r = 300
            if dist_p2l_f <  r and ( np.abs(a_f) < np.deg2rad(20) ):
                d_f = np.sqrt(dist_p2p**2 - dist_p2l_f**2)
            if dist_p2l_l <  r and ( np.abs(a_l) < np.deg2rad(20) ):
                d_l = np.sqrt(dist_p2p**2 - dist_p2l_l**2)
            if dist_p2l_r <  r and ( np.abs(a_r) < np.deg2rad(20) ):
                d_r = np.sqrt(dist_p2p**2 - dist_p2l_r**2)

            # Si l'element courant est devant les autres elements mesures
            # alors on conserve que l'element le plus proche 
            data['front']   = np.min((data['front'],d_f))
            data['left']    = np.min((data['left'],d_l))
            data['right']   = np.min((data['right'],d_r))

        self.setSensors(data.copy())