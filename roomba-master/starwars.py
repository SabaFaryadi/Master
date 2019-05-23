# Taken from
# https://gist.github.com/thenoviceoof/5465084
# but modified to work on my Roomba 770

import time
import create

ROOMBA_PORT = "/dev/tty.usbserial-DA017V6X"

# define silence
r = 30

# map note names in the lilypad notation to irobot commands
c4 = 60
cis4 = des4 = 61
d4 = 62
dis4 = ees4 = 63
e4 = 64
f4 = 65
fis4 = ges4 = 66
g4 = 67
gis4 = aes4 = 68
a4 = 69
ais4 = bes4 = 70
b4 = 71
c5 = 72
cis5 = des5 = 73
d5 = 74
dis5 = ees5 = 75
e5 = 76
f5 = 77
fis5 = ges5 = 78
g5 = 79
gis5 = aes5 = 80
a5 = 81
ais5 = bes5 = 82
b5 = 83
c6 = 84
cis6 = des6 = 85
d6 = 86
dis6 = ees6 = 87
e6 = 88
f6 = 89
fis6 = ges6 = 90

# define some note lengths
# change the top MEASURE (4/4 time) to get faster/slower speeds
MEASURE = 160
HALF = MEASURE/2
Q = MEASURE/4
E = MEASURE/8
Ed = MEASURE*3/16
S = MEASURE/16

MEASURE_TIME = MEASURE/64.



def play_starwars(robot):
  starwars1 = [(a4,Q), (a4,Q), (a4,Q), (f4,Ed), (c5,S), (a4,Q), (f4,Ed), (c5,S), (a4,HALF)]
  starwars2 = [(e5,Q), (e5,Q), (e5,Q), (f5,Ed), (c5,S),(aes4,Q), (f4,Ed), (c5,S), (a4,HALF)]
  starwars3 = [(a5,Q), (a4,Ed), (a4,S), (a5,Q), (aes5,E), (g5,E),(ges5,S), (f5,S), (ges5,S)]
  starwars4 = [(r,E), (bes4,E), (ees5,Q), (d5,E), (des5,E),(c5,S), (b4,S), (c5,E), (c5,E)]
  starwars5 = [(r,E), (f4,E), (aes4,Q), (f4,Ed), (aes4,S),(c5,Q), (a4,Ed), (c5,S), (e5,HALF)]
  starwars6 = [(r,E), (f4,E), (aes4,Q), (f4,Ed), (c5,S),(a4,Q), (f4,Ed), (c5,S), (a4,HALF)]
  print("uploading songs")
  robot.setSong( 1, starwars1 )
  robot.setSong( 2, starwars2 )
  robot.setSong( 3, starwars3 )
  time.sleep(2.0)
  print("playing part 1")
  robot.playSongNumber(1)
  time.sleep(MEASURE_TIME*2.01)
  print("playing part 2")
  robot.playSongNumber(2)
  time.sleep(MEASURE_TIME*2.01)
  print("playing part 3")
  robot.playSongNumber(3)
  robot.setSong( 1, starwars4 )
  time.sleep(MEASURE_TIME*1.26)
  print("playing part 4")
  robot.playSongNumber(1)
  robot.setSong( 2, starwars5 )
  time.sleep(MEASURE_TIME*1.15)
  print("playing part 5")
  robot.playSongNumber(2)
  robot.setSong( 3, starwars3 )
  time.sleep(MEASURE_TIME*1.76)
  print("playing part 3 again")
  robot.playSongNumber(3)
  robot.setSong( 2, starwars6 )
  time.sleep(MEASURE_TIME*1.26)
  print("playing part 4 again")
  robot.playSongNumber(1)
  time.sleep(MEASURE_TIME*1.15)
  print("playing part 6")
  robot.playSongNumber(2)
  time.sleep(MEASURE_TIME*1.76)
  print("done")

robot = create.Create(ROOMBA_PORT)
robot.toSafeMode()
play_starwars(robot)
robot.close()
