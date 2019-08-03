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

def saveDate(depthData1, poseData1, depthData2, poseData2, depthData3, poseData3):
    ## save posedata to a txt
    file = open('pose.txt', 'w')
    content1 = str(poseData1[0])
    for i in range(1,7):
        content1 = content1 + ' ' + str(poseData1[i])
    
    content2 = str(poseData2[0])
    for i in range(1,7):
        content2 = content2 + ' ' + str(poseData2[i])
    
    content3 = str(poseData3[0])
    for i in range(1,7):
        content3 = content3 + ' ' + str(poseData3[i])
    file.write(content1)
    file.write('\n')
    file.write(content2)
    file.write('\n')
    file.write(content3)
    file.close()

    ## save depth data as an image

    ## initial an image to save data
    depthData1 = np.array(depthData1, dtype='uint16')
    image = np.reshape(depthData1, [552, 672])
    
    ## vision sensor 1
    for i in range(len(depthData1)):
        depthData1[i] = round(depthData1[i]*7300 + 700)
    ## mirror image vertically
    depthData1 = np.array(depthData1, dtype='uint16')
    depthData1 = np.reshape(depthData1, [552, 672])
    for i in range(552):
        for j in range(672):
            image[i][j] = depthData1[i][671-j]
    cv2.imwrite('1.pgm',image)

    ## vision sensor 2
    for i in range(len(depthData2)):
        depthData2[i] = round(depthData2[i]*7300 + 700)
    ## mirror image vertically
    depthData2 = np.array(depthData2, dtype='uint16')
    depthData2 = np.reshape(depthData2, [552, 672])
    for i in range(552):
        for j in range(672):
            image[i][j] = depthData2[i][671-j]
    cv2.imwrite('2.pgm',image)


    ## vision sensor 3
    for i in range(len(depthData3)):
        depthData3[i] = round(depthData3[i]*7300 + 700)
    ## mirror image vertically
    depthData3 = np.array(depthData3, dtype='uint16')
    depthData3 = np.reshape(depthData3, [552, 672])
    for i in range(552):
        for j in range(672):
            image[i][j] = depthData3[i][671-j]
    cv2.imwrite('3.pgm',image)



if __name__ == '__main__':
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.2',19999,True,True,5000,5)
    if clientID!=-1:
        print('Connected to remote API server')
        depthData1, poseData1 = getVisionSensor('Vision_sensor1', -1, clientID)
        depthData2, poseData2 = getVisionSensor('Vision_sensor2', -1, clientID)
        depthData3, poseData3 = getVisionSensor('Vision_sensor3', -1, clientID)
        saveDate(depthData1, poseData1, depthData2, poseData2, depthData3, poseData3)
        print('cheers!!')
    else:
        print('Connection non successful')
        sys.exit('Could not connect')