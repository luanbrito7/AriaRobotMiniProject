from AriaPy import *
import sys
import numpy
import math
import heapq

from util import get_robot_home, get_map_min_pos, get_map_max_pos

# variaveis do mapa
map_path = '/usr/local/Aria/maps/office.map'
robot_home = get_robot_home(map_path)
mapMinPos = get_map_min_pos(map_path)
mapMaxPos = get_map_max_pos(map_path)

blocoSize = 510 # Aprox tamanho do robo
tempoSleep = 100
xSize = int(45 * 1000 / blocoSize) # Mapa real tem aprox 18.4m no eixo X
ySize = int(45 * 1000 / blocoSize) # Mapa real tem aprox 13.8m no eixo Y
offset = xSize / 2
mapa = numpy.zeros(shape=(xSize, ySize))

Aria_init()
parser = ArArgumentParser(sys.argv)
parser.loadDefaultArguments()
robot = ArRobot()
conn = ArRobotConnector(parser, robot)

if not conn.connectRobot():
    print "Could not connect to robot, exiting"
    Aria_exit(1)

sonar = ArSonarDevice()
robot.addRangeDevice(sonar)
robot.runAsync(1)
sonarMaxRange = sonar.getMaxRange()


# initial position = [1000, 1500] (1m e 1.5m)
robot.moveTo(ArPose(robot_home['x'], robot_home['y']))

# funcoes de conversao
def getRealCoords(x, y):
    return (x * blocoSize + offset, y * blocoSize + offset)

def getArrayCoords(x, y):
    return (int(round((x - offset) / blocoSize)), int(round((y - offset) / blocoSize)))

recover = ArActionStallRecover()
robot.addAction(recover, 100)

gotoPoseAction = ArActionGoto("goto")
robot.addAction(gotoPoseAction, 50)

stopAction = ArActionStop("stop")
robot.addAction(stopAction, 40)

robot.enableMotors()

def length(x, y):
  return math.sqrt(x*x+y*y)

pos_x, pos_y = getArrayCoords(9000, 9000)
#gotoPoseAction.setGoal(ArPose(pos_x, pos_y))

def getNeighbors(x, y): #pega os 6 vizinhos do elemento onde estou
    neighbors = []
    if(x + 1 < xSize):
        neighbors.append((x+1, y))
    if(y + 1 < ySize):
        neighbors.append((x, y+1))
    if(x > 0):
        neighbors.append((x-1, y))
    if(y > 0):
        neighbors.append((x, y-1))
    return neighbors

def heuristic((x, y)): #distancia minima ate o destino
    position = getRealCoords(x, y)
    dx = pos_x - position[0]    
    dy = pos_y - position[1]
    return length(dx, dy)

def chegueiEm(x, y):
    realX, realY = getArrayCoords(robot.getX(), robot.getY())
    return (realX == x and realY == y)

def moveTo(ar, x, y):
    robot.lock()
    gotoPoseAction.setGoal(ar)
    robot.unlock()
    ArUtil.sleep(50)
    while(not chegueiEm(x, y)):
        h1, h2 = robot.getX(), robot.getY()
        log = "Andando e tou na ({}, {})".format(h1, h2)
        print(log)
        ArUtil.sleep(tempoSleep)

def DFS(x, y):
    robot.lock()
    print (mapa)
    mapa[x][y] = 1
    if(x == pos_x and y == pos_y):
        return 1
    for i in range(0, robot.getNumSonar()):
      sr = robot.getSonarReading(i)
      if sr.getRange() < sonarMaxRange: # parede
        # insere na matriz de obstaculos
        globalCoords = {'x': sr.getX(), 'y': sr.getY()}
        mapCoords = getArrayCoords(globalCoords['x'], globalCoords['y'])
        
        log = "globalCoords = ({}, {})    mapCoords = ({}, {})".format(globalCoords['x'], globalCoords['y'], mapCoords[0], mapCoords[1])
        print log
        mapa.itemset(mapCoords, 1)
        # print mapa
    robot.unlock()
    neighbors = getNeighbors(x, y)
    h1, h2 = robot.getX(), robot.getY()
    h3, h4 = getRealCoords(x, y)
    log = "Debugando isso aqui, tou na posicao ({}, {}) e acho que tou na ({}, {})".format(h1, h2, h3, h4)
    print(log)
    for i,j in neighbors:
        if(mapa[i][j] == 0):
            realI, realJ = getRealCoords(i, j)
            log = "Tentando ir para ({}, {})".format(realI, realJ)
            print(log)
            moveTo(ArPose(realI, realJ), i, j)
            h1, h2 = robot.getX(), robot.getY()
            log = "Debugando isso aqui, cheguei na posicao ({}, {})".format(h1, h2)
            print(log)
            if(DFS(i, j)):
                return 1
            realX, realY = getRealCoords(x, y)
            moveTo(ArPose(realX, realY), x, y)
            h1, h2 = robot.getX(), robot.getY()
            log = "Debugando isso aqui, voltei pra posicao ({}, {})".format(h1, h2)
            print(log)
    return 0 


posX = robot.getX()
posY = robot.getY()
x, y = getArrayCoords(posX, posY)
if(DFS(x, y)):
    print("Deu bom")
else:
    print("Deu ruim")

while Aria.getRunning:

    # pos_x *= 2
    # pos_y *= 2
    ArUtil.sleep(5000)
    # robot.unlock()
