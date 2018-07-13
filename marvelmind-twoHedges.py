from marvelmind import MarvelmindHedge
from time import sleep
import sys
import time

hedge1 = MarvelmindHedge(tty = "/dev/ttyACM0",adr=2, debug=False)
hedge2 = MarvelmindHedge(tty = "/dev/ttyACM1",adr=8, debug=False)
hedge1.start()
hedge2.start()
sleep(1)
positionX=list()
positionY=list()
start_time = time.time()
while (time.time() - start_time) < 5:
	sleep(1)
	current_psition=hedge1.position()
	if current_psition[1]== 0 or current_psition[2]== 0:
		print 'error in GPS positioning'
		continue
	positionX.append(current_psition[1])
	positionY.append(current_psition[2])
	print 'current_psition=',current_psition[1],',',current_psition[2]
#hedge1.stop()
X1=100*(sum(positionX)/len(positionX))
Y1=100*(sum(positionY)/len(positionY))
print 'hedge1',X1,'',Y1

positionX=list()
positionY=list()
start_time = time.time()
while (time.time() - start_time) < 5:
	sleep(1)
	current_psition=hedge2.position()
	if current_psition[1]== 0 or current_psition[2]== 0:
		print 'error in GPS positioning'
		continue
	positionX.append(current_psition[1])
	positionY.append(current_psition[2])
	print 'current_psition=',current_psition[1],',',current_psition[2]
hedge1.stop()
hedge2.stop()
X2=100*(sum(positionX)/len(positionX))
Y2=100*(sum(positionY)/len(positionY))
print 'hedge2',X2,'',Y2