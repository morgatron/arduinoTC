# -*- coding: utf-8 -*-
"""
GUI for an arduino temperature controller
"""



#import initExample ## Add path to library (just for examples; you do not need this)

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np
import scipy as sp
from collections import deque
from pyqtgraph.dockarea import *
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
import pdb
import re, time

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
area = DockArea()
win.setCentralWidget(area)
win.resize(1000,700)
win.setWindowTitle('Application Name!!')

## Create docks, place them into the window one at a time.
## Note that size arguments are only a suggestion; docks will still have to
## fill the entire dock area and obey the limits of their internal widgets.
d1 = Dock("Dock1- UI", size=(1, 1))     ## give this dock the minimum possible size
d1a = Dock("Params", size=(300, 500))     ## give this dock the minimum possible size
d2 = Dock("Dock2 - Console", size=(500,200))
d3 = Dock("wave-forms", size=(300,400))
d4 = Dock("Response", size=(500,400))
d5 = Dock("Impedance Graph", size=(500,400))
#d5 = Dock("Impedance Graph", size=(500,400))
area.addDock(d1, 'left')      ## place d1 at left edge of dock area (it will fill the whole space since there are no other docks yet)
area.addDock(d1a, 'above', d1)
area.addDock(d2, 'right')     ## place d2 at right edge of dock area
area.addDock(d3, 'bottom', d1)## place d3 at bottom edge of d1
area.addDock(d4, 'top', d2)   
area.addDock(d5, 'above', d4)   
#area.addDock(d5, 'above', d4)   

#area.moveDock(d4, 'top', d2)     ## move d4 to top edge of d2

## Add widgets into each dock

## first dock gets save/restore buttons
w1 = pg.LayoutWidget()
label = QtGui.QLabel("""Some buttons""")
getParamsBtn = QtGui.QPushButton('Get params')
sendParamsBtn = QtGui.QPushButton('Send params')
#sendParamsBtn.setEnabled(False)

w1.addWidget(label, row=0, col=0)
w1.addWidget(getParamsBtn, row=1, col=0)
w1.addWidget(sendParamsBtn, row=2, col=0)
d1.addWidget(w1)
#w2=pg.console.ConsoleWidget()
w2 = QtGui.QPlainTextEdit()
w2.setReadOnly(True)
w2.setMaximumBlockCount(500)

#w2 = QtGui.QScrollArea()
d2.addWidget(w2)

#d3.hideTitleBar()
w3 = pg.PlotWidget(title="Waveform")
w3.addLegend(offset=[-30,-30])
sens_curve=w3.plot([], pen=None, symbolPen=(255,0,0), symbol='o', name='sense')
#sens_curve=w3.plot([], pen=(255,0,0), symbol='o', name='sense')
heat_curve=w3.plot([], pen=None, symbolBrush=(0,255,0), symbol='o', name='output')
#heat_curve=w3.plot([], pen=(0,255,0), name='output')
d3.addWidget(w3)


w5 = pg.PlotWidget(title="Impedance")
w5.addLegend()
R_curve=w5.plot([], pen=(255,255,0), name='R')
C_curve=w5.plot([], pen=(255,0,255), name='C')

w4 = pg.PlotWidget(title="Response graph")
w4.addLegend()
read_curve0=w4.plot([], pen=(255,0,0), name='0')
read_curve90=w4.plot([], pen=(0,255,0), name='90')
read_curveR=w4.plot([], pen=(0,0,255), name='R')
output_curve=w4.plot([], pen=(0,255,255), name='Out')
d4.addWidget(w4)

Npts=2000
read_data0 = deque([],Npts)
read_data90 = deque([],Npts)
read_dataR = deque([],Npts)
output_data = deque([],Npts)
R_data = deque([],Npts)
C_data = deque([],Npts)
tax = deque([],Npts)

state = None

import time
from pylab import rand
class PretendArd(object):
    t_last=0;
    def flushInput(self):
        pass;
    def flushOutput(self):
        pass;
    def write(self, st):
        pass
    def inWaiting(self):
        t_now=time.time()
        if t_now>self.t_last+0.1:
            self.t_last=t_now
            return True;
        else:
            return False
        
    def readline(self):
        return "d: {0} {1} {2} {3}\n".format(time.time()*1000, int(rand()*1000), int(rand()*1000), 0)

import serial
if 0:
    ardPort=serial.Serial("/dev/ttyACM0", 115200, timeout=0.2, writeTimeout=0.2)
    ardPort.flushInput()
    ardPort.flushOutput()
else:
    ardPort=PretendArd()

contParams=[
    dict(
        name='read_lag',
        type='int',
        suffix='pts',
        limits=(0,100),
        value=0,
        tip='Expected delay for the read waveform (in points)'),
    dict(
        name='sens_phase',
        type='int',
        suffix='pts',
        limits=(0,100),
        value=0,
        tip='Phase of the sense waveform (in points)'
        ),
    dict(
        name='sens_amp',
        type='int',
        limits=(0,4095),
        value=300,
        tip='Size of the sens-waveform (0-4095)'
        ),
    dict(
        name='set_point',
        type='float',
        limits=(0,1e5),
        ),
    dict(
        name='kp',
        type='float',
        value=0.001,
        ),
    dict(
        name='ki',
        type='float',
        value=0.001,
        ),
    dict(
        name='kd',
        type='float',
        value=0.000,
        ),
    dict(
        name='sens_wfm?',
        type='action',
        ),
    dict(
        name='heat_wfm?',
        type='action',
        ),
    dict(
        name='reset_integral',
        type='action',
        ),
]

p = Parameter.create(name='contParams', type='group', children=contParams)

def sendCommand(cmdName, *args):
    global w2 # Reference to the scroll area so we can update it
    #print("cmdName is: {}".format(cmdName))
    cmdString=cmdName + ' '+' '.join(args)
    print("sending: {}".format(cmdString))
    w2.appendPlainText("send -> {}".format(cmdString))
    if ardPort:
        ardPort.write(cmdString+'\n')

def change(param, changes):
    print("tree changes:")
    for param, changeType, newVal in changes:
        #path = p.childPath(param)
        #if path is not None:
            #paramName = '.'.join(path)
        #else:
        paramName = param.name()
        print('  parameter: %s'% paramName)
        print('  changeType:    %s'% changeType)
        print('  data:      %s'% str(newVal))
        print('  ----------')
    if changeType=='value':
        sendCommand(paramName, str(newVal)) 
    if changeType=='activated':
        sendCommand(paramName)#, str(newVal)) 

p.sigTreeStateChanged.connect(change)
paramTree = ParameterTree()
paramTree.setParameters(p, showTop=False)
d1a.addWidget(paramTree)

r=re.compile('[: ,]+'); # Regex to match any sequence of colons, 
                #spaces, or commas as seperators in communications from the arduino.

def readAndProcessArd():
    global w2; #Scroll area widget
    lines=[]
    #while ardPort.inWaiting():
        
    while ardPort.inWaiting():
        line=ardPort.readline()
        line=line.strip().strip(',')
        line=line.strip()
        lines.append(line)
    for line in lines:
        splts=r.split(line)#.split()
        if len(splts)==0:
            print('Problem with recieved line: "{0}"'.format(line))
            return

        cmdName=splts[0]
        if cmdName != 'd':
            #print('ard-> {}'.format(line))
            w2.appendPlainText("rec -> {}".format(line))
        args=splts[1:]
        if cmdName=='d':
            updateResponsePlots(*[float(a) for a in args])

        if 1:
            if cmdName=='sens_wfm':
                updateSensWfm(np.array(args, dtype='i4'))
            if cmdName=='heat_wfm':
                updateHeatWfm(np.array(args, dtype='i4'))
            if cmdName in p.names.keys():
                p.names[cmdName].setValue(args[0])
                if len(args)>0:
                    print('Too many args ({}) to assign to parameter {}. Will just do the first'.format(args, cmdName))

#k=0
def update():
    global w2#,k
    #w2.appendPlainText(str(k))
    #k+=1
    #updateResponsePlots(0,0,0,0)
    readAndProcessArd()

def updateResponsePlots(sampleTime, read0, read90, output):
    global read_data0, read_data90, output_data, read_curve0, read_curve90, read_curveR, output_curve;

    #read0=sp.rand()
    #read90=sp.rand()
    #output=sp.rand()/2+0.4

    read_data0.append(read0)
    read_data90.append(read90)
    read_dataR.append(sp.sqrt(read0**2+read90**2))
    output_data.append(output)

    #tlast+=0.5+sp.rand()/5 
    tax.append(sampleTime)
    #tax.append(time.time()-tstart)

    read_curve0.setData(tax, read_data0)
    read_curve90.setData(tax, read_data90)
    read_curveR.setData(tax, read_dataR)
    output_curve.setData(tax, output_data)

def updateImpedancePlot(sampleTime, read0, read90, output):
    global R_data, C_data,  R_curve, C_curve
    #read0=sp.rand()
    #read90=sp.rand()
    #output=sp.rand()/2+0.4
    read_data0.append(read0)
    read_data90.append(read90)
    read_dataR.append(sp.sqrt(read0**2+read90**2))
    output_data.append(output)

    #tlast+=0.5+sp.rand()/5 
    tax.append(sampleTime)
    #tax.append(time.time()-tstart)

    read_curve0.setData(tax, read_data0)
    read_curve90.setData(tax, read_data90)
    read_curveR.setData(tax, read_dataR)
    output_curve.setData(tax, output_data)

def updateSensWfm(dat):
    sens_curve.setData(dat)
def updateHeatWfm(dat):
    heat_curve.setData(dat)

def sendParams():
    print("Sending params to arudino") 
    print("Not implemented yet")
    #Somehow loop through a list of parameters and send them all
def getParams():
    print("Get params from arduino")
    print("Not implemented yet")


getParamsBtn.clicked.connect(getParams)
sendParamsBtn.clicked.connect(sendParams)

timer = QtCore.QTimer()
timer.timeout.connect(update)



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    timer.start(50)
    win.show()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
