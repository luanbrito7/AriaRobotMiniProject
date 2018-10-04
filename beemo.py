from AriaPy import *
import sys
import numpy
import math

from util import get_robot_home, get_map_min_pos, get_map_max_pos

# variaveis do mapa
map_path = '/usr/local/Aria/maps/office.map'
robot_home = get_robot_home(map_path)
mapMinPos = get_map_min_pos(map_path)
mapMaxPos = get_map_max_pos(map_path)

blocoSize = 500 # Aprox tamanho do robo
xSize =int((mapMaxPos['x']-mapMinPos['x']) / blocoSize) # Mapa real tem aprox 18.4m no eixo X
ySize = int((mapMaxPos['y']-mapMinPos['y']) / blocoSize) # Mapa real tem aprox 13.8m no eixo Y
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
    return (x * blocoSize, y * blocoSize)

def getArrayCoords(x, y):
    return (int(round(x / blocoSize)), int(round(y / blocoSize)))

recover = ArActionStallRecover()
robot.addAction(recover, 100)

gotoPoseAction = ArActionGoto("goto")
robot.addAction(gotoPoseAction, 50)

stopAction = ArActionStop("stop")
robot.addAction(stopAction, 40)

robot.enableMotors()

def length(x, y):
  return math.sqrt(x*x+y*y)

pos_x = robot.getX()+40000
pos_y = robot.getY()+5000
gotoPoseAction.setGoal(ArPose(pos_x, pos_y))

def getNeighbors(x, y): #pega os 6 vizinhos do elemento onde estou
    neighbors = []
    if(x < xSize):
        neighbors.append((x+1, y))
        if(y < ySize):
            neighbors.append((x+1, y+1))
            neighbors.append((x, y+1))
    if(x > 0):
        neighbors.append((x-1, y))
        if(y > 0):
            neighbors.append((x-1, y-1))
            neighbors.append((x, y-1))
    if(x > 0 and y < ySize):
        neighbors.append((x-1, y+1))
    if(x < xSize and y > 0):
        neighbors.append((x+1, y-1))
    return neighbors

def heuristic(x, y): #distancia minima ate o destino
    position = getRealCoords(x, y)
    dx = pos_x - position[0]
    dy = pos_y - position[1]
    return length(dx, dy)


def greedy(x, y):
    position = getArrayCoords(x, y)
    neighbors = getNeighbors(position[0], position[1])
    


while Aria.getRunning:
    
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



    print robot.getPose()
    # pos_x *= 2
    # pos_y *= 2
    ArUtil.sleep(5000)
    # robot.unlock()
