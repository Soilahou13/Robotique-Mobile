import math
import numpy as np

from controller import Robot, DistanceSensor, Motor, Camera, Keyboard



# Création de l'instance  Robot .
robot = Robot()
keyboard=Keyboard()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# Definition de la vitesse maximum  
MAX_SPEED = 6.28


braitenberg_coefficients =[[ 0.942, -0.22], [0.63, -0.1], [0.5, -0.06],[-0.06, -0.06],[-0.06, -0.06],[-0.06, 0.5], [-0.19, 0.63], [-0.13, 0.942]]
sensors_value=[1,1,1,1,1,1,1,1]  
speed=[0,0]

# initialisation de l'appareil
print('---Starting initialisation device-------------')

print('---------Initialisation Leds------------------')
leds=[]
ledsNames = ['led0','led1','led2','led3','led4','led5','led6','led7','led8','led9']
for i in range(10):
    leds.append(robot.getLED(ledsNames[i]))


print('---Initialisation Proximity Sensors ps--------')
ps = []
psNames = ['ps0', 'ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(timestep)


print('----Initialisation Light Sensors ls-----------')
ls = []
lsNames = ['ls0', 'ls1', 'ls2','ls3','ls4','ls5','ls6','ls7']
for i in range(8):
    ls.append(robot.getLightSensor(lsNames[i]))
    ls[i].enable(timestep)

print('----Initialisation Motors---------------------')
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

print('----Initialisation Position Sensor---------------------')
#'left wheel sensor' and 'right wheel sensor'
leftcodeur=robot.getPositionSensor('left wheel sensor')
rightcodeur=robot.getPositionSensor('right wheel sensor')
leftcodeur.enable(timestep)
rightcodeur.enable(timestep)
print(leftcodeur.getType())
print(rightcodeur.getType())
right_codeur=rightcodeur.getValue()
left_codeur=leftcodeur.getValue()
distance_parcourue=0

print('----Initialisation Cameras---------------------')
camera=robot.getCamera('camera')
camera.enable(timestep)

print('----Initialisation Keyboard---------------------')
keyboard.enable(timestep)
   
print('--------- Initialisation finished-------------')

print('Vous êtes en mode manuel')



useManual = False
useMode="Manual"
thetaMemo=0
theta=0

while robot.step(timestep) != -1:
    # read sensors ps
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.0 * MAX_SPEED
    rightSpeed = 0.0 * MAX_SPEED
    
    # read keyboard and control
    currentKey = keyboard.getKey()  
    
    if currentKey == ord('m') or currentKey == ord('M'):
        useMode="Manual"
        print('Passage en mode manuel')
    if currentKey == ord('a') or currentKey == ord('A'):
        useMode="Auto"  
        print('Passage en mode automatique') 
    if currentKey == ord('b') or currentKey == ord('B'):
        useMode="Braitenberg"         
        print('Passage en mode Braitenberg') 
         
    if useMode=="Manual":
        if currentKey == keyboard.UP:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        elif currentKey == keyboard.DOWN:
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
        elif currentKey == keyboard.LEFT:
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        elif currentKey == keyboard.RIGHT:
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED     
   
    if useMode=="Braitenberg":
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
       
    if useMode=="Auto": 
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
        if left_obstacle:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 *  MAX_SPEED 
       
          
        elif right_obstacle:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED  
            
    
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    
   
# Début traitement du but (trouver la balle)  
# Récupération de l'image
image = camera.getImageArray()

largeurImage = int(camera.getWitdth())
hauteurImage = int(camera.getHeight())

red = np.zeros((largeurImage,hauteurImage))
green = np.zeros((largeurImage,hauteurImage))
blue = np.zeros((largeurImage,hauteurImage))

#
for x in range (largeurImage):
    for y in range (hauteurImage):
        red[x][y] = image [x][y][0]
        green[x][y] = image [x][y][1]
        blue[x][y] = image [x][y][2]
        if  x==25 and y==15:
            print ("red=", red[x][y])
            print ("green=", green[x][y])
            print ("blue=",[x][y])
            print ("--------------------")
            
    for k in range (largeurImage):
        pixelVerts=0
        for y in range (hauteurImage):
            l=np.sqrt(red[k][y]*red[k][y]+green[k][y]*green[k][y]+blue[k][y]*blue[k][y])
            teta=np.arccos(blue[k][y]/l)
            phi = np.arctan(green[k][y]/red[k][y])
            if k==25 and y ==15:
                print ("l=",l)
                print ("teta=",teta)
                print ("phie=",phi)
                print ("--------------------------------------")
            if( (phi>1.0) and (teta>1.0)):
                pixelVerts +=1
        histogrammePixelVerts = pixelVerts
        
        