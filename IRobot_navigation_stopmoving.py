from marvelmind import MarvelmindHedge
from time import sleep
import sys
import math
from breezycreate2 import Robot
import time

bot = Robot()
hedge = MarvelmindHedge(tty="/dev/ttyACM0", debug=False)  # create MarvelmindHedge thread
hedge.start()  # start thread
X_final = 210
Y_final = 145
SleepPeriod = 0.2
sleep(1)

ErrorY = 100
ErrorX = 100
StepHeading = 1
StepApproach = 2
Threshold = (StepApproach + StepHeading) * 10
while (ErrorX > Threshold or ErrorY > Threshold):
    positionX = list()
    positionY = list()
    start_time = time.time()
    while (time.time() - start_time) < 5 * SleepPeriod:
        sleep(SleepPeriod)
        current_psition = hedge.position()
        positionX.append(current_psition[1])
        positionY.append(current_psition[2])
        print
        'current_psition=', current_psition[1], ',', current_psition[2]
    X1 = sum(positionX) / len(positionX)
    Y1 = sum(positionY) / len(positionY)
    print
    X1, '', Y1

    bot.setForwardSpeed(100)
    sleep(StepHeading)
    bot.setForwardSpeed(0)

    positionX = list()
    positionY = list()
    start_time = time.time()
    while (time.time() - start_time) < 5 * SleepPeriod:
        sleep(SleepPeriod)
        current_psition = hedge.position()
        positionX.append(current_psition[1])
        positionY.append(current_psition[2])
        print
        'current_psition=', current_psition[1], ',', current_psition[2]
    X2 = sum(positionX) / len(positionX)
    Y2 = sum(positionY) / len(positionY)
    print
    X2, '', Y2

    Teta_heading = math.atan2(Y2 - Y1, X2 - X1) * 180 / math.pi
    if Teta_heading < 0:
        Teta_heading = Teta_heading + 360
    print
    'Theta Heading=', Teta_heading

    Teta_aproach = math.atan2(Y_final - Y2, X_final - X2) * 180 / math.pi
    if Teta_aproach < 0:
        Teta_aproach = Teta_aproach + 360
    print
    'Theta Approach=', Teta_aproach

    Teta_Robot = Teta_aproach - Teta_heading;
    while Teta_Robot < 0:
        Teta_Robot = Teta_Robot + 360
    while Teta_Robot > 360:
        Teta_Robot = Teta_Robot + 360

    if (Teta_Robot <= 180):
        bot.setTurnSpeed(-400)
    else:
        Teta_Robot = 360 - Teta_Robot
        bot.setTurnSpeed(+400)

    sleep(Teta_Robot / 180)
    bot.setTurnSpeed(0)

    bot.setForwardSpeed(100)
    sleep(StepApproach)
    bot.setForwardSpeed(0)
    ErrorX = math.fabs(X_final - X2)
    ErrorY = math.fabs(Y_final - Y2)

sys.exit()