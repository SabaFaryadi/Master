from marvelmind import MarvelmindHedge
from time import sleep
import sys
import math
import time

hedge = MarvelmindHedge(tty = "/dev/ttyACM0", debug=False) # create MarvelmindHedge thread
hedge.start() # start thread
SleepPeriod=0.2
DataCollectionDuration=10

positionX=list()
positionY=list()
start_time = time.time()
while (time.time() - start_time) < DataCollectionDuration:
	sleep(SleepPeriod)
	current_psition=hedge.position()
	if current_psition[1]== 0 or current_psition[2]== 0:
		print 'error in GPS positioning'
		continue
	positionX.append(current_psition[1])
	positionY.append(current_psition[2])
	print 'current_psition=',current_psition[1],',',current_psition[2]
X1=sum(positionX)/len(positionX)
Y1=sum(positionY)/len(positionY)

'''STD=0
for element in positionX:
	STD=STD+math.pow((X1-element),2)
STD=math.sqrt(STD)/(len(positionX)-1)
XNew=list()
for element in positionX:
	if((element<(X1+STD)) and (element>(X1-STD))):
		XNew.append(element)
print XNew
X1=sum(XNew)/len(XNew)

STD=0
for counter in positionY:
	STD=STD+math.pow((Y1-counter),2)
STD=math.sqrt(STD)/(len(positionY)-1)
YNew=list()
for element in positionY:
	if((element<(Y1+STD)) and (element>(Y1-STD))):
		YNew.append(element)
print YNew
Y1=sum(YNew)/len(YNew)'''

print X1,'',Y1
hedge.stop()