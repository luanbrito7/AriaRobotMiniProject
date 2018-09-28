"""
#include "Aria.h"
int main(int argc, char** argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  // Try connecting to the robot.
  if(!robotConnector.connectRobot(&robot))
  {
    // Error!
    ArLog::log(ArLog::Terse, "Error, could not connect to robot.\n");
    robotConnector.logOptions();
    Aria::exit(1);
  }
  // Run the ArRobot processing/task cycle thread.
  robot.runAsync(true);
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  // Parse command line arguments (there may be arguments specifying 
  // what lasers to try to connect to)
  if(!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(2);
  }
  // Try connecting to all lasers specified in the robot's parameter file
  // and in command line arguments
  if(!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Terse, "Error, could not connect to lasers.\n");
    Aria::logOptions();
    Aria::exit(3);
  }
  // Now we're connected, and the robot and laser objects are running in 
  // background threads reading and processing data. (You can get access
  // to the ArLaser objects using ArRobot::findLaser() or
  // ArRobot::getLaserMap()
"""

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
    # robot.lock()
    # poses = sonar.getCurrentBufferAsVector()
    print robot.getPose()
    gotoPoseAction.setGoal(ArPose(pos_x, pos_y))
    pos_x *= 2
    pos_y *= 2
    ArUtil.sleep(2000)
    # robot.unlock()
