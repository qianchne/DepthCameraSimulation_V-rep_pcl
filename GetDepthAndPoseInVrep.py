### chan 2019/8/2
### get depth data in vrep  
import vrep,time,sys
import matplotlib.pyplot as plt
from PIL import Image as I
import array 
import numpy as np
import cv2 


def getVisionSensor(visionSensorName, robotBase, clientID):
    ## Get the handle of the vision sensor
    _,visionSensorHandle = vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
    if robotBase != -1 :
        _,robotBaseHandle = vrep.simxGetObjectHandle(clientID,robotBase,vrep.simx_opmode_oneshot_wait)
    else:
        robotBaseHandle = -1
    time.sleep(1)
    
    ## first import, activate API imw
    _,resolution,image=vrep.simxGetVisionSensorDepthBuffer(clientID,visionSensorHandle,vrep.simx_opmode_streaming)
    _,position = vrep.simxGetObjectPosition(clientID, visionSensorHandle, robotBaseHandle, vrep.simx_opmode_streaming)
    _,quarternion = vrep.simxGetObjectQuaternion(clientID, visionSensorHandle, robotBaseHandle, vrep.simx_opmode_streaming)
    time.sleep(0.5)


    ## capture depth data and pose information !!!
    number = 0
    while (vrep.simxGetConnectionId(clientID)!=-1): 

        ## get depth information
        _,resolution,image=vrep.simxGetVisionSensorDepthBuffer(clientID,visionSensorHandle,vrep.simx_opmode_streaming)
        ## get position and quarternion of the vision sensor
        _,position = vrep.simxGetObjectPosition(clientID, visionSensorHandle, robotBaseHandle, vrep.simx_opmode_buffer)
        _,quarternion = vrep.simxGetObjectQuaternion(clientID, visionSensorHandle, robotBaseHandle, vrep.simx_opmode_buffer)

        ## conbine position and quarternion
        position.extend(quarternion)

        ## record the time get information above
        imageAquirsitionTime = vrep.simxGetLastCmdTime(clientID)
        print(imageAquirsitionTime)

        ## control the number of image
        number = number + 1
        if number == 1:
            break
        
    return image, position

def saveDate(depthData, poseData):
    ## save posedata to a txt
    file = open('pose.txt', 'w')
    content = str(poseData[0])
    for i in range(1,7):
        content = content + ' ' + str(poseData[i])
    file.write(content)
    file.close()

    ## save depth data as an image
    for i in range(len(depthData)):
        depthData[i] = round(depthData[i]*7300 + 700)
    depthData = np.array(depthData, dtype='uint16')
    image = np.reshape(depthData, [552, 672])
    cv2.imwrite('1.pgm',image)



if __name__ == '__main__':
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.2',19999,True,True,5000,5)
    if clientID!=-1:
        print('Connected to remote API server')
        depthData, poseData = getVisionSensor('Vision_sensor2', -1, clientID)
        saveDate(depthData, poseData)
        print('cheers!!')
    else:
        print('Connection non successful')
        sys.exit('Could not connect')