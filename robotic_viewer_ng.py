# This Python file uses the following encoding: utf-8
import sys
import numpy as np
from PySide2.QtWidgets  import QMainWindow, QApplication, QGridLayout
from PySide2.QtWidgets  import QLabel, QPushButton, QLineEdit, QWidget, QSlider
from PySide2.QtWidgets  import QGroupBox, QVBoxLayout, QComboBox
from PySide2.QtGui      import QPixmap, QImage, QIcon, QPainter, QBrush,QColor
from PySide2.QtGui      import QPen, QTransform, QPolygon
from PySide2.QtCore     import QObject,QRect, Signal, Slot, QPoint, QTimer, Qt
from robotique_lib      import *

class Robotic_Viewer_NG(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setWindowTitle('A Wonderfull Name')
        self.resize(1080,720)
        self.setWindowIcon(QIcon(QPixmap('smallLogo.png')))
        self.setStyleSheet("QMainWindow {background: '#f1fcb3';}");
        self.setMinimumWidth(960)
        self.setMinimumHeight(720)

        ## Création des widgets
        self.settingsWidget = QWidget(self)
        ## Configuration des signals/slots
        self.timeSimu = 90
        self.time = 0
        self.rythmSim = QTimer()
        self.rythmSim.timeout.connect(self.simulationStep)
        self.buildGameElements()
        ## Configuration des widgets
        self.mapPx = QImage('TableCDFR2020.png').scaledToWidth(self.width()*2/3,aspectmode='KeepAspectRatio')
        self.scaleFactor = 300/self.mapPx.width()
        self.buildSettingsGroupBox(self.settingsWidget)
        self.settingsWidget.setStyleSheet("background: '#94e4f7';")
        self.fitToWindow()

    def buildGameElements(self):
        self.robotM = VirtualRobot('MarioBot')
        self.robotG = VirtualRobot('GuiguiBot')
        self.robots = [self.robotM,self.robotG]

        self.robotM_traget = Circle(0,0,5)
        self.robotG_traget = Circle(0,0,5)

        self.cercA = Circle(80,-30,30)
        self.cercB = Circle(30,330,30)
        self.cercC = Circle(80,330,30)
        self.cercD = Circle(30,-30,40)

        self.staticElts = [self.cercA,self.cercB,self.cercC,self.cercD]

        i = 0
        for elt in self.staticElts :
            elt.setID(elt.getID()+' '+repr(i))
            print(elt)
            i = i +1

        self.brain = Strategy()

        self.act1 = Action('ManuelM',90,25,np.deg2rad(0))
        self.act2 = Action('ManuelG',70,25,np.deg2rad(180))

        self.brain.addAction(self.act1.getID(),self.act1)
        self.brain.addAction(self.act2.getID(),self.act2)

        self.rstElmtPos()

    def rstElmtPos(self):
        self.robotM.setStaticState(90,25,np.deg2rad(0))
        self.robotG.setStaticState(70,25,np.deg2rad(180))

        self.repaint()

    def rstElmtStat(self):
        #self.staticElts = [self.cercA,self.cercB,self.cercC,self.cercD]

        self.repaint()

    def paintEvent(self, event):
        mapP = QPainter()
        mapP.begin(self)
        mapP.drawImage(self.centralGP.x(),self.centralGP.y(),self.mapPx)
        mapP.end()

        self.drawRobot(self.robotM,'blue')
        self.drawRobot(self.robotG,'purple')
        self.drawElts(self.staticElts,'black','pink')
        self.drawElts([self.robotM_traget,self.robotG_traget],'red','white')

    def drawElts(self,elt_list,colorElt,colotTxt):
        for elt in elt_list :
            l_pp = self.getRealFromUiCoord(elt.getX(),elt.getY())
            eltP = QPainter()
            eltP.begin(self)
            eltP.setBrush(QBrush(QColor(colorElt)))

            myPen = QPen()
            myPen.setBrush(QBrush(QColor(colotTxt)))

            if 'rect' in elt.getID() :
                l_pw = elt.getWidth()/self.scaleFactor
                l_pl = elt.getLength()/self.scaleFactor
                eltP.drawRect(l_pp.getX()-(l_pw/2),l_pp.getY()-(l_pl/2),l_pw,l_pl)

                eltP.setPen(myPen)
                eltP.drawText(l_pp.getX()-(l_pw/4),l_pp.getY(),l_pw,l_pl,1,elt.getID())
            elif 'circle' in elt.getID() :
                l_pr = elt.getRayon()/self.scaleFactor
                eltP.drawEllipse(l_pp.getX()-(l_pr/2),l_pp.getY()-(l_pr/2),l_pr,l_pr)

                eltP.setPen(myPen)
                eltP.drawText(l_pp.getX()-l_pr/3,l_pp.getY(),l_pr,l_pr,1,elt.getID())
            else :
                pass
            eltP.end()

    def drawRobot(self, robot, color):
        # Position computing
        relativePoint = self.getRealFromUiCoord(robot.getPosition().x,robot.getPosition().y)
        w_tr = robot.getWidth()/(2*self.scaleFactor)
        l_tr = robot.getLength()/(4*self.scaleFactor)
        x_center = relativePoint.x # w_tr
        y_center = relativePoint.y #- l_tr

        pts_list = QPoint(-w_tr,-l_tr), QPoint(-w_tr,l_tr), QPoint(0,l_tr*1.2), QPoint(w_tr,l_tr),QPoint(w_tr,-l_tr),QPoint(-w_tr,-l_tr)

        polyst = self.rotatePolygon(pts_list,robot.getAngle())
        poly = QPolygon(polyst)
        poly.translate(x_center,y_center)

        # Draw Item
        myPen = QPen()
        myPen.setBrush(QBrush(QColor(color)))
        myPen.setWidth(5)

        robotP = QPainter()
        robotP.begin(self)
        robotP.setPen(myPen)
        robotP.drawPolyline(poly)
        robotP.end()

    def rotatePolygon(self,polygon,teta):
        rotatedPolygon = []
        for corner in polygon :
            Vi = np.array([corner.x(),corner.y(),1])
            Mrot = np.array([ [np.cos(teta), np.sin(teta),0],
                              [-np.sin(teta), np.cos(teta),0],
                              [ 0,0,0] ])
            Vo = np.dot(Mrot,np.transpose(Vi))

            rotatedPolygon.append(QPoint(Vo[0],Vo[1]))
        return rotatedPolygon

    def buildSettingsGroupBox(self, a_widget) :
        self.settingsGroupBox = QGroupBox('Settings')
        l_gridLayout = QGridLayout()

        self.labelRegTitle = QLabel('Position')           # 0
        self.lineEditReg = QLineEdit()                      # 1
        self.lineEditReg.setText("X : 0 - Y : 0 - T : 0")   # 1
        self.pushBtnRegRst = QPushButton('Restart')         # 2
        self.pushBtnRegRst.clicked.connect(self.rstElmtPos) # 2
        self.labelRegTarget = QLabel('Target : not defined')  # 3
        self.selectedRobot = 0

        self.labelSimuTitle = QLabel('Simulation')      # 0
        self.pushBtnStartSimu = QPushButton('Start')    # 1
        self.pushBtnStartSimu.clicked.connect(self.startSimu)
        self.pushBtnStopSimu = QPushButton('Stop')      # 2
        self.pushBtnStopSimu.clicked.connect(self.stopSimu)
        self.labelSimuInfos = QLabel('Durée de simulation : '+repr(self.timeSimu)+' sec')   # 4
        self.hSlider= QSlider()                         # 3
        self.hSlider.setOrientation(Qt.Horizontal)      # 3
        self.hSlider.valueChanged.connect(self.sliderEvolution)
        self.hSlider.setValue(self.timeSimu)
        self.hSlider.setRange(0,120)

        self.labelPfTitle = QLabel('Path Finding')      # 0
        self.comboBoxAlgoPF = QComboBox()               # 1
        self.comboBoxAlgoPF.addItem('A*')               # 1
        self.comboBoxAlgoPF.addItem('Dijtra')           # 1
        self.comboBoxLenghtPF = QComboBox()             # 2
        self.comboBoxLenghtPF.addItem('Manathan')       # 2
        self.comboBoxLenghtPF.addItem('Euclidian')      # 2
        self.pushBtnPF = QPushButton('NOT USED')         # 3
        self.pushBtnPF.clicked.connect(self.rstElmtStat)
        self.labelPfElt = QLabel('')                    # 4

        self.labelStratTitle = QLabel('Stratégie')      # 0
        self.labelStratRbtG = QLabel('GuiGuiBot')       # 0
        self.comboBoxStratRbtG = QComboBox()            # 1
        for l_act in self.brain.getActionKeys() :
            self.comboBoxStratRbtG.addItem(l_act)
        self.comboBoxStratRbtG.currentIndexChanged.connect(self.updateStrategies)
        self.labelStratRbtM = QLabel('MarioBot')        # 2
        self.comboBoxStratRbtM = QComboBox()            # 3
        for l_act in self.brain.getActionKeys() :
            self.comboBoxStratRbtM.addItem(l_act)
        self.comboBoxStratRbtM.currentIndexChanged.connect(self.updateStrategies)

        l_gridLayout.addWidget(self.labelRegTitle,0,0)
        l_gridLayout.addWidget(self.lineEditReg,1,0)
        l_gridLayout.addWidget(self.pushBtnRegRst,2,0)
        l_gridLayout.addWidget(self.labelRegTarget,3,0)

        l_gridLayout.addWidget(self.labelSimuTitle,0,1)
        l_gridLayout.addWidget(self.pushBtnStartSimu,1,1)
        l_gridLayout.addWidget(self.pushBtnStopSimu,2,1)
        l_gridLayout.addWidget(self.hSlider,3,1)
        l_gridLayout.addWidget(self.labelSimuInfos,4,1)

        l_gridLayout.addWidget(self.labelStratTitle,0,2)
        l_gridLayout.addWidget(self.labelStratRbtG,1,2)
        l_gridLayout.addWidget(self.comboBoxStratRbtG,2,2)
        l_gridLayout.addWidget(self.labelStratRbtM,3,2)
        l_gridLayout.addWidget(self.comboBoxStratRbtM,4,2)

        l_gridLayout.addWidget(self.labelPfTitle,0,3)
        l_gridLayout.addWidget(self.comboBoxAlgoPF,1,3)
        l_gridLayout.addWidget(self.comboBoxLenghtPF,2,3)
        l_gridLayout.addWidget(self.pushBtnPF,3,3)
        l_gridLayout.addWidget(self.labelPfElt,4,3)

        self.settingsGroupBox.setLayout(l_gridLayout)
        vbox = QVBoxLayout()
        vbox.addWidget(self.settingsGroupBox)
        a_widget.setLayout(vbox)

    def startSimu(self):
        self.time = 0
        self.rythmSim.start(100)

    def stopSimu(self):
        self.rythmSim.stop()

    def updateStrategies(self):
        self.brain.assignAction(self.comboBoxStratRbtG.currentText(),self.robotG.getID())
        self.brain.assignAction(self.comboBoxStratRbtM.currentText(),self.robotM.getID())

    def updateRbtSensors(self):
        for rbt in self.robots :
            tx = -rbt.getPosition().getX()
            ty = -rbt.getPosition().getY()
            Mtr = np.array([ [1,0,tx],
                             [0,1,ty],
                             [0,0,1] ])

            data = {'front':300,'left':300,'right':300}
            for k in self.brain.getElementsKeys() :
                elt = self.brain.getElement(k)

                # Placer l'elt dans le référentiel du robot
                Vori = np.array([elt.getX(),elt.getY(),1])
                Vrob = np.dot(Mtr,np.transpose(Vori))
                l_elt_pos = Point(Vrob[0],Vrob[1])

                r = 0
                if 'rect' in elt.getID():
                    r = np.max((elt.getWidth(),elt.getLength()))/2
                elif 'circle' in elt.getID():
                    r = elt.getRayon()
                else :
                    pass

                dist_p2p = rbt.getPosition().distanceFrom(elt.getPosition())

                l_e =  rbt.getAngle()-Point(0,0).capTo(l_elt_pos)
                l_e = moduloPI(l_e)

                a_f = l_e
                a_l = l_e+np.deg2rad(20)
                a_r = l_e-np.deg2rad(20)

                dist_p2l_f = np.abs(dist_p2p*np.tan(a_f))
                dist_p2l_l = np.abs(dist_p2p*np.tan(a_l))
                dist_p2l_r = np.abs(dist_p2p*np.tan(a_r))

                # if 'M' in rbt.getID() :
                #     print(f'MarioBot : rayon de elt = {r}')
                #     print(f'MarioBot : Angle Err    = {np.rad2deg(l_e)}')
                #     print(f'MarioBot : Distance P2P = {dist_p2p}')
                #     print(f'MarioBot : Distance P2L = {dist_p2l_f}')

                d_f = 300
                d_l = 300
                d_r = 300
                if dist_p2l_f <  r and np.abs(a_f) < np.deg2rad(20):
                    d_f = np.sqrt(dist_p2p**2 - dist_p2l_f**2)
                if dist_p2l_l <  r and np.abs(a_l) < np.deg2rad(20):
                    d_l = np.sqrt(dist_p2p**2 - dist_p2l_l**2)
                if dist_p2l_r <  r and np.abs(a_r) < np.deg2rad(20):
                    d_r = np.sqrt(dist_p2p**2 - dist_p2l_r**2)

                data['front']   = np.min((data['front'],d_f))
                data['left']    = np.min((data['left'],d_l))
                data['right']   = np.min((data['right'],d_r))

            rbt.setSensors(data.copy())
            #print (rbt.getID()+'dist : '+repr(rbt.getSensors()))

    def simulationStep(self):
        if self.time < self.timeSimu :
            # Mise à jour de l'interface graphique
            self.time = self.time + (self.rythmSim.interval()/1000)
            self.labelSimuInfos.setText('Simulation : '+repr(round(self.time,2))+'\t/ '+repr(self.timeSimu)+' sec')
            # Mise à jour des donnés senseurs
            self.updateRbtSensors()

            # Appel de la stratégie
            self.brain.setTargetPoint(self.robotM)
            self.brain.setTargetPoint(self.robotG)

            # Simulation des robots
            self.robotM_traget.setXY(self.robotM.getTarget().getPosition().getX(),self.robotM.getTarget().getPosition().getY())
            self.robotG_traget.setXY(self.robotG.getTarget().getPosition().getX(),self.robotG.getTarget().getPosition().getY())

            for rbt in self.robots :
                rbt.simulate(self.rythmSim.interval())
            self.repaint()
        else :
            pass

    def fitToWindow(self):
        self.settingsWidget.setGeometry(0,0,self.width(),self.height()/4)
        self.centralGP = QPoint((self.width()-self.mapPx.width())/2, self.settingsWidget.height()+20)
        self.repaint()

    def getUiFromRealCoord(self,x,y):
        ty = -self.centralGP.x()
        tx = -self.centralGP.y()
        s = self.scaleFactor

        Vre = np.array([x,y,s])
        Mtr = np.array([ [ 0,s,tx],
                         [ s,0,ty],
                         [ 0,0,0] ])
        Vui = np.dot(Mtr,np.transpose(Vre))

        l_x = Vui[0]
        l_y = Vui[1]
        return Point(round(l_x),round(l_y))

    def getRealFromUiCoord(self,x,y):
        tx = (y/self.scaleFactor)
        ty = (x/self.scaleFactor)
        Vui = np.array([self.centralGP.x(),self.centralGP.y(),1])
        Mtr = np.array([ [ 1,0,tx],
                         [ 0,1,ty],
                         [ 0,0,1] ])
        Vre = np.dot(Mtr,np.transpose(Vui))

        l_x = Vre[0]
        l_y = Vre[1]
        return Point(round(l_x),round(l_y))

    def mousePressEvent(self, it):
        rel_pt = self.getUiFromRealCoord(it.x(),it.y())
        if it.button()== Qt.MouseButton.RightButton and self.selectedRobot != 0:
            self.labelRegTarget.setText('Manual target of '+self.selectedRobot.getID()+' X: '+repr(rel_pt.x)+' Y:'+repr(rel_pt.y))
            if 'M' in self.selectedRobot.getID():
                self.brain.modifyAction('ManuelM',State(rel_pt.x,rel_pt.y,np.pi))
            if 'G' in self.selectedRobot.getID():
                self.brain.modifyAction('ManuelG',State(rel_pt.x,rel_pt.y,np.pi))

        if it.button()== Qt.MouseButton.LeftButton :
            for rbt in self.robots :
                if rbt.getPosition().distanceFrom(rel_pt)<10 :
                    self.selectedRobot = rbt
                    self.labelRegTarget.setText(self.selectedRobot.getID()+ ' selected...')

    def refreshEltLabel(self):
        l_str = ''
        for i in self.brain.getElementsKeys():
            l_str = i+'\n'+l_str
        self.labelPfElt.setText(l_str)

    def sliderEvolution(self,it):
        self.timeSimu = self.hSlider.value()
        self.labelSimuInfos.setText('Durée de simulation :'+repr(self.timeSimu)+' sec')

    def wheelEvent(self,event):
        angle_delta = 10.0*(event.delta()/120)
        rel_pt = self.getUiFromRealCoord(event.x(),event.y())
        
        for rbt in self.robots :
            if rbt.getPosition().distanceFrom(rel_pt)<10 :
                rbt.setAngle(rbt.getAngle() + np.deg2rad(angle_delta))
                self.lineEditReg.setText(rbt.__str__()+' '+rbt.getPosition().__str__())
                self.repaint()

    def mouseMoveEvent(self,it):
        rel_pt = self.getUiFromRealCoord(it.x(),it.y())

        table = Rect(100,150,200,300)

        for rbt in self.robots:
            if rbt.getPosition().distanceFrom(rel_pt)<10 :
                rbt.setPosition(rel_pt.x,rel_pt.y)
                self.lineEditReg.setText(rbt.__str__()+' '+rbt.getPosition().__str__())
                self.repaint()

        for elt in self.staticElts:
            if elt.getPosition().distanceFrom(rel_pt)<10 :
                elt.setPosition(rel_pt.x,rel_pt.y)

                if table.contains(elt.getPosition()):
                    if not self.brain.containsElement(elt.getID()):
                        self.brain.addElement(elt.getID(),elt)
                        self.refreshEltLabel()
                    else :
                        pass
                else :
                    if self.brain.containsElement(elt.getID()):
                        self.brain.removeElement(elt.getID())
                        self.refreshEltLabel()
                    else:
                        pass
                self.lineEditReg.setText(elt.__str__())
            self.repaint()

    def resizeEvent(self, it):
        self.fitToWindow()

if __name__ == "__main__":
    app = QApplication([])
    window = Robotic_Viewer_NG()
    window.show()
    sys.exit(app.exec_())