from AriaPy import *
import sys
import numpy
from util import get_robot_home, get_map_min_pos, get_map_max_pos

# variaveis do mapa
map_path = '/usr/local/Aria/maps/office.map'
robot_home = get_robot_home(map_path)
mapMinPos = get_map_min_pos(map_path)
mapMaxPos = get_map_max_pos(map_path)

blocoSize = 500 # Aprox tamanho do robo
xSize =int(mapMaxPos['x']-mapMinPos['x'] / blocoSize) # Mapa real tem aprox 18.4m no eixo X
ySize = int(mapMaxPos['y']-mapMinPos['y'] / blocoSize) # Mapa real tem aprox 13.8m no eixo Y
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

pos_x = 1
pos_y = 1
while Aria.getRunning:
    




    print robot.getPose()
    gotoPoseAction.setGoal(ArPose(pos_x, pos_y))
    pos_x *= 2
    pos_y *= 2
    ArUtil.sleep(20000)
    # robot.unlock()
