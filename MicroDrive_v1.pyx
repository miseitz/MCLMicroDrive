# -*- coding: utf-8 -*-
"""
Class to control the Mad City Labs MicroDrive with 3 axes (26.1mm range) and encoders (50 nm resolution).

Functions include:
    - getPosition()
        Reads the encoders.
        return (x,y,z) encoder position
    - move(x,y,z,velocity = 3)
        Moves to the (x,y,z) encoder position with given velocity (mm/s).
        return (x,y,z) from encoders
    - moveControlled(self, x, y, z, velocity = 3, errorMove = 0.001)
        Moves to the (x,y,z) encoder position with given velocity (mm/s).
        Movement is finished once the encoder read is within +/- of the given 
        errorMove (mm)
        return (x,y,z) from encoders
    - moveAxis(axis, value, velocity = 3)
        Moves single axis to the given value. x = 1, y = 2, z = 3.
        Moves to the (x or y or z) encoder position with given velocity (mm/s).
        return (x,y,z) from encoders
    - moveAxisControlled(axis, value, velocity = 3, errorMove = 0.001)
        Moves single axis to the given value. x = 1, y = 2, z = 3.  
        Moves to the (x,y,z) encoder position with given velocity (mm/s).
        Movement is finished once the encoder read is within +/- of the given 
        errorMove (mm)
        return (x,y,z) from encoders
    - moveRelative(self, dx, dy, dz, velocity = 3)
        Moves by (dx,dy,dz) relative to the current position with given velocity (mm/s).
        return (x,y,z) from encoders
    - moveRelativeAxis(self, axis, distance, velocity = 3)
        Moves by distance relative to the current position of a single axis 
        with given velocity (mm/s).
        return (x,y,z) from encoders
    - isMoving()
        Checks if a motor is moving.
        return motor response
    - stopMoving()
        Stops the motors from moving.
        return None
    - centerHomePosition()
        Centers the encoders to the total range window.
        To find the center it moves into the reverse limit of each axis and 
        determines the center positions by knowing the total range is 26.1 mm. 
        Motors are moved to the center position (0,0,0).
        return None
    - centerHomePositionLong()
        Same as centerHomePosition(), but it does not assume a total range of 26.1mm.
        Rather it moves into all limits (forward and reverse) and calculates the
        center from there.
        return None
    - Home
        Moves to the (0,0,0) position of the encoders.
        return None
    - EncodersReset
        Sets the current position as the new center position (0,0,0)
        return None
    - getStatus()
        Checks if an axis is out of bounds and returns a list of axes which axes 
        are out of bounds:
        return ListOfAxesOutOfBounds
    - solveLimit(distanceFromLimit = 1)
        Checks if at least one axis is in forward or reverse limit and moves 
        the axis out of the limit by distanceFromLimit millimiters.
        return None
    - getInfo()
        Returns information about the MicroStage:
            encoderResolution = 0.05
            stepSize =  9.525e-05
            maxVelocity = 4
            maxVelocityTwoAxis = 3.75
            maxVelocityThreeAxis = 3
            minVelocity = 0.01905
    - closeConnection()
        Releases the handle.
        return None
    
 "Private" functions:
    - wait
        Waits for the previous function to finish.
        return None
     - _move(x,y,z,velocity = 3)
        Moves to x, y, z position according to encoders with velocity. Waits
        until the movement is complete.
        return errorCode of movement
     - _moveRelative(self, dx, dy, dz, velocity = 3)
        Moves motors by dx, dy, dz relative to the current position. Waits
        until the movement is complete.
        return errorCode of movement
    - _moveRelativeAxis(self, axis, distance, velocity = 3)
        Moves axis by distance relative to the current position. Waits
        until the movement is complete.
        return errorCode of movement

Additional remarks:
    - Wait should be called after the movement to guarantee that the motors are 
    not moving anymore. Trying to read the encoders or move the motors, while
    they are still movingn, might affect the internal timing pulses.
    However, wait should not  be called multiple times, as it is slowed if it
    is called more then once.


author: Michael Seitz
"""
import ctypes
import numpy as np
#import time
#import matplotlib.pyplot as plt

class MicroDrive(object):
    def __init__(self, mcl_lib = "C:/Program Files/Mad City Labs/MicroDrive/MicroDrive"):
        """
        Here, we initialize the object
        - Loading the DLL
        - Connecting to the stage.
        """
    
        self.errorDictionary = {0: 'MCL_SUCCESS',
                                -1: 'MCL_GENERAL_ERROR',
                                -2: 'MCL_DEV_ERROR',
                                -3: 'MCL_DEV_NOT_ATTACHED',
                                -4: 'MCL_USAGE_ERROR',
                                -5: 'MCL_DEV_NOT_READY',
                                -6: 'MCL_ARGUMENT_ERROR',
                                -7: 'MCL_INVALID_AXIS',
                                -8: 'MCL_INVALID_HANDLE'}
        
        #Dictionary to know the axis limit returrns. Dicitionary saves [axis, forward (1) or backward (-1), description]        
        self.motorLimits = [[1,-1,'Axis 1 reverse limit'],   #126 <-> '1111110' <-> position 0
                            [1,1,'Axis 1 forward limit'],    #125 <-> '1111101' <-> position 1
                            [2,-1,'Axis 2 reverse limit'],   #123 <-> '1111011' <-> position 2
                            [2,1,'Axis 2 forward limit'],    #119 <-> '1110111' <-> position 3
                            [3,-1,'Axis 3 reverse limit'],   #111 <-> '1101111' <-> position 4
                            [3,1,'Axis 3 forward limit']]    #095 <-> '1011111' <-> position 5
                                
        # Load the DLL
        self.mcl = ctypes.cdll.LoadLibrary(mcl_lib)
        #Release existing handles
        self.mcl.MCL_ReleaseAllHandles()
        #Connect to the instrument and creat a handle
        self.handle = self.mcl.MCL_InitHandle() #Handle number is assigned, which is a positive integer
        #Check if connection was successful
        if self.handle > 0:
            print('Connected to SN: '+ str(self.mcl.MCL_GetSerialNumber(self.handle)) + '\nWith handle: ' + str(self.handle))
        else:
            print('Connection failed. Maybe the device is turned off?')
        
        #Save Product information
          # Create pointers for query
        encoderResolution_temp = ctypes.pointer(ctypes.c_double())
        stepSize_temp = ctypes.pointer(ctypes.c_double())
        maxVelocity_temp = ctypes.pointer(ctypes.c_double())
        maxVelocityTwoAxis_temp = ctypes.pointer(ctypes.c_double())
        maxVelocityThreeAxis_temp = ctypes.pointer(ctypes.c_double())
        minVelocity_temp = ctypes.pointer(ctypes.c_double())
          #Make the query
        self.mcl.MCL_MDInformation(encoderResolution_temp, stepSize_temp, maxVelocity_temp, maxVelocityTwoAxis_temp, maxVelocityThreeAxis_temp, minVelocity_temp, self.handle)
          #Save the information
        self.encoderResolution = encoderResolution_temp.contents.value
        self.stepSize = stepSize_temp.contents.value
        self.maxVelocity = maxVelocity_temp.contents.value
        self.maxVelocityTwoAxis = maxVelocityTwoAxis_temp.contents.value
        self.maxVelocityThreeAxis = maxVelocityThreeAxis_temp.contents.value
        self.minVelocity = minVelocity_temp.contents.value
          #Delete pointers just to be save
        del encoderResolution_temp
        del stepSize_temp
        del maxVelocity_temp
        del maxVelocityTwoAxis_temp
        del maxVelocityThreeAxis_temp
        del minVelocity_temp
        
          #Set standard minimum and maximum velocity
        self.velocityMin = self.minVelocity # mm/s (normally 0.01905 mm/s)
        self.velocityMax = self.maxVelocity # mm/s (normally 3 mm/s)
        self.totalScanRange = 26.1 # mm

    def __enter__(self):
        return self

    def getPosition(self):
        """
        This function takes approximately 10ms.
        """
        
        e1 = ctypes.pointer(ctypes.c_double())
        e2 = ctypes.pointer(ctypes.c_double())
        e3 = ctypes.pointer(ctypes.c_double())
        e4 = ctypes.pointer(ctypes.c_double())
        errorNumber = self.mcl.MCL_MDReadEncoders(e1, e2, e3, e4, self.handle)
        if errorNumber != 0:
            print('Error reading the encoders: ' + self.errorDictionary[errorNumber])
        print('Encoders: ' + str(np.round(e1.contents.value,4)) + ', ' + str(np.round(e2.contents.value,5)) + ', ' + str(np.round(e3.contents.value,5)))
        position_temp = [e1.contents.value, e2.contents.value, e3.contents.value]
        del e1
        del e2
        del e3
        del e4
        return position_temp

    def _getStatus(self): #Internal function to get the error number
        status_temp = ctypes.pointer(ctypes.c_ushort())
        self.mcl.MCL_MDStatus(status_temp,self.handle)
        result_temp = status_temp.contents.value
        del status_temp
        return result_temp
    
#    def getStatus(self): #External status
#        # Takes 0.3 ms
#        return self.motorLimitCodes[self._getStatus()]
    
    def getStatus(self):
        """
        Returns a list of motors that are out of bounds (reverse of forward limit)
        [axis, forward (1) / reverse (-1), description]
        [1,-1,'Axis 1 reverse limit']
        [1, 1,'Axis 1 forward limit']
        [2,-1,'Axis 2 reverse limit']
        [2, 1,'Axis 2 forward limit']
        [3,-1,'Axis 3 reverse limit']
        [3, 1,'Axis 3 forward limit']
        """
        status = self._getStatus()
        
        errorsLimit = []
        for i, b in enumerate(bin(status)[:1:-1]):
            if b == '0':
                errorsLimit.append(self.motorLimits[i])
        if errorsLimit == []: #If no limit is detected, we add the All ok line
            errorsLimit.append([0,0,'All ok'])
        return errorsLimit
    
    def solveLimit(self, distanceFromLimit = 1):
        """
        Function to get motors out of reverse or forward limit. 
        distanceFromLimit is the distance to move fromom the limit
        """
            
        # Check status of motors
        status = self.getStatus()
        if status[0][0] == 0:
            print('No limit was encountered. No movement done.')
        else:
            # Check if the given distance is bigger than the actual range
            if distanceFromLimit > 25:
                print('Given distance is too big and was set to the standard of 1 mm.')
                distanceFromLimit = 1
            for status_temp in status: #Go through all limits
                # Move the axis out of the limit
                self.moveRelativeAxis(status_temp[0],-status_temp[1]*distanceFromLimit)

    def wait(self):
        """
        This function takes approximately 10ms if the motors are not moving.
        """
        errorNumber = self.mcl.MCL_MicroDriveWait(self.handle)
        if errorNumber != 0:
            print('Error while waiting: ' + self.errorDictionary[errorNumber])
            
#    def wait(self):
#        """
#        Optimized wait function that only waits as long as really needed. Only good for smaller steps
#        If steps are missed. This time delay needs to be increased by probably 1-2 ms: time.sleep((waitTime-1)/1.1 + 0.002)
#        """
#        waitTime = ctypes.pointer(ctypes.c_int())
#        errorNumber = self.mcl.MCL_MicroDriveGetWaitTime(waitTime, self.handle)
#        if errorNumber != 0:
#            print('Error while waiting: ' + self.errorDictionary[errorNumber])
#        time.sleep((waitTime.contents.value-1)/1.1 + 0.002) #  close approximate real time as follows: (double)(wait - 1) / 1.10
    
    # Start: Internal move functions that have no error handling and should be used with caution and only if one is familiar with the motors
    def _move(self, x, y, z, velocity = 3): #Internal function without wait
        #Get the current position
        position = np.array(self.getPosition())
        dx = x - position[0]
        dy = y - position[1]
        dz = z - position[2]
        #Move to desired absolute position based on the encoder values
        errorCode = self.mcl.MCL_MDMoveThreeAxes(ctypes.c_uint(1),ctypes.c_double(velocity), ctypes.c_double(dx),
                                            ctypes.c_uint(2),ctypes.c_double(velocity), ctypes.c_double(dy),
                                            ctypes.c_uint(3),ctypes.c_double(velocity), ctypes.c_double(dz), self.handle)
        self.wait()
        return errorCode
    
    def _moveRelative(self, dx, dy, dz, velocity = 3): #Internal function without wait
        errorCode = self.mcl.MCL_MDMoveThreeAxes(ctypes.c_uint(1),ctypes.c_double(velocity), ctypes.c_double(dx),
                                            ctypes.c_uint(2),ctypes.c_double(velocity), ctypes.c_double(dy),
                                            ctypes.c_uint(3),ctypes.c_double(velocity), ctypes.c_double(dz), self.handle)
        self.wait()
        return errorCode
        
    def _moveRelativeAxis(self, axis, distance, velocity = 3):
        errorCode = self.mcl.MCL_MDMove(ctypes.c_uint(axis),ctypes.c_double(velocity), ctypes.c_double(distance), self.handle)
        self.wait()
        return errorCode
    # End: Internal move functions that have no error handling and should be used with caution and only if one is familiar with the motors
    
    def move(self, x, y, z, velocity = 3):
        """
        Moves until to x, y, z according to the encoders.
        Moving 1 mm with all axes takes roughly  s =  mm/s (velocity = 3)
        Moving 0.1 mm with all axes takes roughly 0.22 s = 0.45 mm/s (velocity = 2) or 0.18 s = 0.55 mm/s (velocity = 3)
        Moving 0.01 mm with all axes takes roughly 0.21 s = 0.047 mm/s (velocity = 3)
        """

        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too low. Velocity is set to minimum value.')
            velocity = self.velocityMin
        
        # Check if x, y , z values are out of bounds
        if abs(x) > self.totalScanRange/2:
            print('Value for x is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        elif abs(y) > self.totalScanRange/2:
            print('Value for y is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        elif abs(z) > self.totalScanRange/2:
            print('Value for z is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
            
        #Move
        errorNumber = self._move(x, y, z, velocity)
        if errorNumber != 0: #Check for error
            print('Error while moving: ' + self.errorDictionary[errorNumber])
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
        
        position = self.getPosition()
        return position
    
    def moveControlled(self, x, y, z, velocity = 3, errorMove = 0.001):
        """
        Moves until to x, y, z according to the encoders and keeps moving until
        it reaches the x, y, z position within the error of 1/- errorMove.
        Moving 1 mm with all axes takes roughly 
        Moving 0.1 mm with all axes takes roughly 0.36 s = 0.28 mm/s (velocity = 3)
        Moving 0.01 mm with all axes takes roughly 0.178 s = 0.056 mm/s (velocity = 3)
        """
        
        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too low. Velocity is set to minimum value.')
            velocity = self.velocityMin
            
        # Check if x, y , z values are out of bounds
        if abs(x) > self.totalScanRange:
            print('Value for x is out of bound +/- ' + str(self.totalScanRange) + ' mm. Movement is aborted.')
            return list(self.getPosition())
        elif abs(y) > self.totalScanRange:
            print('Value for y is out of bound +/- ' + str(self.totalScanRange) + ' mm. Movement is aborted.')
            return list(self.getPosition())
        elif abs(z) > self.totalScanRange:
            print('Value for z is out of bound +/- ' + str(self.totalScanRange) + ' mm. Movement is aborted.')
            return list(self.getPosition())
        
        # First move quickly to the position
        errorNumber = self._move(x, y, z, velocity) #Move the stage 
        if errorNumber != 0: #Check for error
            print('Error while moving: ' + self.errorDictionary[errorNumber])
        position = np.array(self.getPosition())
        
        while any(abs(position-np.array([x,y,z])) > errorMove):
            dx = x - position[0]
            dy = y - position[1]
            dz = z - position[2]
            errorNumber = self._moveRelative(dx, dy, dz, velocity) #Move the stage 
            if errorNumber != 0: #Check for error
                print('Error while moving: ' + self.errorDictionary[errorNumber])
            position = np.array(self.getPosition())
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
        
        return list(position)
    
    def moveControlledAxis(self, axis, value, velocity = 3, errorMove = 0.001):
        """
        Moves a single axis to value with velocity with a specified precision.
        """
        # Check if axis is valid
        if axis not in [1,2,3]:
            print('Invalid axis given. Axis needs to be 1, 2 or 3. Movement aborted.')
            return self.getPosition()
        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too low. Velocity is set to minimum value.')
            velocity = self.velocityMin
        # Check if the movement would go out of bounds
        position = self.getPosition()
        if abs(value) > self.totalScanRange/2:
            print('Value for ' + [0, 'x', 'y', 'z'][axis] + ' is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        # First move quickly to the position   
        errorNumber = self._moveRelativeAxis(axis, value - position[axis-1], velocity)
        #Check for error
        if errorNumber != 0:
            print('Error while moving: ' + self.errorDictionary[errorNumber])
        position = np.array(self.getPosition())
        
        while abs(position[axis-1]-value) > errorMove:
            errorNumber = self._moveRelativeAxis(axis, value - position[axis-1], velocity) #Move the stage 
            if errorNumber != 0: #Check for error
                print('Error while moving: ' + self.errorDictionary[errorNumber])
            position = np.array(self.getPosition())
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
            
        return self.getPosition()
    
    def moveAxis(self, axis, value, velocity = 3):
        """
        Moves a single axis to value with velocity anc 
        """
        # Check if axis is valid
        if axis not in [1,2,3]:
            print('Invalid axis given. Axis needs to be 1, 2 or 3. Movement aborted.')
            return self.getPosition()
        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too low. Velocity is set to minimum value.')
            velocity = self.velocityMin
        # Check if the movement would go out of bounds
        position = self.getPosition()
        if abs(value) > self.totalScanRange/2:
            print('Value for ' + [0, 'x', 'y', 'z'][axis] + ' is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        #Move the stage    
        errorNumber = self._moveRelativeAxis(axis, value - position[axis-1], velocity)
        #Check for error
        if errorNumber != 0:
            print('Error while moving axis ' + str(axis) + ': ' + self.errorDictionary[errorNumber])
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
            
        return self.getPosition()
    
    
    def moveRelative(self, dx, dy, dz, velocity = 3):
        """
        Moves all axes by (dx, dy, dz) relative to the current position.
        Moving 1 mm with all axes takes roughly 0.69 s = 1.45 mm/s (velocity = 2) or 0.47 s = 2.13 mm/s (velocity = 3)
        Moving 0.1 mm with all axes takes roughly 0.09 s = 1.11 mm/s (velocity = 2) or 0.08 s = 1.25 mm/s (velocity = 3)
        """
        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too high. Velocity is set to minimum value.')
            velocity = self.velocityMin
        # Check if the movement would go out of bounds
        position = self.getPosition()
        if abs(position[0]+dx) > self.totalScanRange/2:
            print('Value for x is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        elif abs(position[1]+dy) > self.totalScanRange/2:
            print('Value for y is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        elif abs(position[2]+dz) > self.totalScanRange/2:
            print('Value for z is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        #Move the stage
        errorNumber = self._moveRelative(dx, dy, dz, velocity)
        #Check for error
        if errorNumber != 0:
            print('Error while moving: ' + self.errorDictionary[errorNumber])
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
        
        return self.getPosition()
    
    def moveRelativeAxis(self, axis, distance, velocity = 3):
        """
        Moves a single axis by distance with velocity.
        """
        # Check if axis is valid
        if axis not in [1,2,3]:
            print('Invalid axis given. Axis needs to be 1, 2 or 3. Movement aborted.')
            return self.getPosition()
        # Check the given velocity
        if velocity > self.velocityMax:
            print('Given velocity is too high. Velocity is set to maximum value.')
            velocity = self.velocityMax
        elif velocity < self.velocityMin:
            print('Given velocity is too high. Velocity is set to minimum value.')
            velocity = self.velocityMin
        # Check if the movement would go out of bounds
        position = self.getPosition()
        if abs(position[0]+distance) > self.totalScanRange/2:
            print('Value for ' + [0, 'x', 'y', 'z'][axis] + ' is out of bound +/- ' + str(self.totalScanRange/2) + ' mm. Movement is aborted.')
            return self.getPosition()
        #Move the stage    
        errorNumber = self._moveRelativeAxis(axis, distance, velocity)
        #Check for error
        if errorNumber != 0:
            print('Error while moving axis ' + str(axis) + ': ' + self.errorDictionary[errorNumber])
        
        # Check if motors moved out of bounds
        status = self.getStatus()
        if status[0] != [0,0,'All ok']:
            print('Motor moved out of bounds: ' + str([temp[2] for temp in status]))
            
        return self.getPosition()
        
    def isMoving(self):
        """
        Checks if motors are moving.
        This function takes approximately 20ms.
        """
        isMoving = ctypes.pointer(ctypes.c_int())
        self.mcl.MCL_MicroDriveMoveStatus(isMoving, self.handle)
        result_temp = isMoving.contents.value
        del isMoving
        return result_temp
    
    def stopMoving(self):
        """
        Stops motors from moving.
        """
        status = ctypes.pointer(ctypes.c_ushort())
        errorNumber = self.mcl.MCL_MDStop(status, self.handle)
        del status
        if errorNumber != 0:
            print('Error while stopping device: ' + self.errorDictionary[errorNumber])
    
    def getPositionStepsTakenAxis(self, axis):
        microSteps = ctypes.pointer(ctypes.c_int())
        errorNumber = self.mcl.MCL_MDCurrentPositionM(ctypes.c_int(axis), microSteps, self.handle)
          #Check for error
        if errorNumber != 0:
            print('Error reading the position of axis' + str(axis) + ': ' + self.errorDictionary[errorNumber])
        position_temp = microSteps.contents.value*self.stepSize
        del microSteps
        return position_temp
    
    def getPositionStepsTaken(self):
        #Axis 1
        microSteps1 = ctypes.pointer(ctypes.c_int())
        errorNumber1 = self.mcl.MCL_MDCurrentPositionM(ctypes.c_int(1), microSteps1, self.handle)
          #Check for error
        if errorNumber1 != 0:
            print('Error reading the position of axis 1: ' + self.errorDictionary[errorNumber1])
        #Axis 2
        microSteps2 = ctypes.pointer(ctypes.c_int())
        errorNumber2 = self.mcl.MCL_MDCurrentPositionM(ctypes.c_int(2), microSteps2, self.handle)
          #Check for error
        if errorNumber2 != 0:
            print('Error reading the position of axis 2: ' + self.errorDictionary[errorNumber2])
        #Axis 3
        microSteps3 = ctypes.pointer(ctypes.c_int())
        errorNumber3 = self.mcl.MCL_MDCurrentPositionM(ctypes.c_int(3), microSteps3, self.handle)
          #Check for error
        if errorNumber3 != 0:
            print('Error reading the position of axis 3: ' + self.errorDictionary[errorNumber3])
        position_temp = [microSteps1.contents.value*self.stepSize,microSteps2.contents.value*self.stepSize,microSteps3.contents.value*self.stepSize]
        del microSteps1
        del microSteps2
        del microSteps3
        return position_temp

    def Home(self):
        """
        Moves to (0,0,0) position.
        """
        self.moveControlled(0,0,0)
        
    def centerHomePosition(self):
        """
        Centers the encoders in the center of the motor's range.
        Gets the center by moving into the reverse limit and moving by half the total range 26.1mm.
        """
        distanceFromLimit = 5
        distanceToLimit = self.totalScanRange + 4
        #Check if there is a limit and if there is one get out of it
        self.solveLimit(distanceFromLimit)
        
        edgeValues = [None, None, None]
        
        #Move into backward limit: axis 1 backwards
        self._moveRelativeAxis(1,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [1,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 1: ' + str([temp[2] for temp in status]))
            return
        edgeValues[0] = self.getPosition()[0] # save value
        self._moveRelativeAxis(1, distanceFromLimit) #Move out of limit
        
        #Move into backward limit: axis 2 backwards
        self._moveRelativeAxis(2,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [2,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 2: ' + str([temp[2] for temp in status]))
            return
        edgeValues[1] = self.getPosition()[1] # save value
        self._moveRelativeAxis(2, distanceFromLimit) #Move out of limit
        
        #Move into backward limit: axis 3 backwards
        self._moveRelativeAxis(3,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [3,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 3: ' + str([temp[2] for temp in status]))
            return
        edgeValues[2] = self.getPosition()[2] # save value
        self._moveRelativeAxis(3, distanceFromLimit) #Move out of limit
        
        print('Limits: ' + str(edgeValues))
        
        newHome = [edgeValues[i] + self.totalScanRange/2 for i in range(3)]
        self.move(*newHome)
        
        self.EncodersReset()
        
    def centerHomePositionLong(self):
        """
        Centers the encoders in the center of the motor's range.
        Gets the center by moving into both forward and reverse limit and moving 
        into the center of both.
        """
        distanceFromLimit = 5
        distanceToLimit = self.totalScanRange + 4
        #Check if there is a limit and if there is one get out of it
        self.solveLimit(distanceFromLimit)
        
        edgeValues = [[None, None],[None, None],[None, None]]
        
        #Move into backward limit: axis 1 backwards
        self.moveRelativeAxis(1,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [1,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 1: ' + str([temp[2] for temp in status]))
            return
        edgeValues[0][0] = self.getPosition()[0] # save value
        #Move into backward limit: axis 1 forward
        self.moveRelativeAxis(1,distanceToLimit) # move into forward limit
        status = self.getStatus()
        if not any([temp[:2] == [1,1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 1: ' + str([temp[2] for temp in status]))
            return
        edgeValues[0][1] = self.getPosition()[0]
        self.moveRelativeAxis(1,-distanceFromLimit) #Move out of limit
        
        #Move into backward limit: axis 2 backwards
        self.moveRelativeAxis(2,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [2,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 2: ' + str([temp[2] for temp in status]))
            return
        edgeValues[1][0] = self.getPosition()[1] # save value
        #Move into backward limit: axis 1 forward
        self.moveRelativeAxis(2,distanceToLimit) # move into forward limit
        status = self.getStatus()
        if not any([temp[:2] == [2,1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 2: ' + str([temp[2] for temp in status]))
            return
        edgeValues[1][1] = self.getPosition()[1]
        self.moveRelativeAxis(2,-distanceFromLimit) #Move out of limit
        
        #Move into backward limit: axis 3 backwards
        self.moveRelativeAxis(3,-distanceToLimit) # move into backwards limit
        status = self.getStatus()
        if not any([temp[:2] == [3,-1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 3: ' + str([temp[2] for temp in status]))
            return
        edgeValues[2][0] = self.getPosition()[2] # save value
        #Move into backward limit: axis 1 forward
        self.moveRelativeAxis(3,distanceToLimit) # move into forward limit
        status = self.getStatus()
        if not any([temp[:2] == [3,1] for temp in status]):
            print('Unexpected limit encountered while attempting forward limit of axis 3: ' + str([temp[2] for temp in status]))
            return
        edgeValues[2][1] = self.getPosition()[2]
        self.moveRelativeAxis(3,-distanceFromLimit) #Move out of limit
        
        print('Limits: ' + str(edgeValues))
        
        newHome = [(edgeValues[i][0]+edgeValues[i][1])/2 for i in range(3)]
        self.move(*newHome)
        
        self.EncodersReset()
        
    def EncodersReset(self):
        """
        Resets the encoders and sets the current position as the new (0,0,0) position.
        """
        status_temp = ctypes.pointer(ctypes.c_ushort())
        self.mcl.MCL_MDResetEncoders(status_temp,self.handle)
        self.wait()
        del status_temp

    def getInfo(self):
        """
        Returns info about the motors:
            encoderResolution = 0.05
            stepSize =  9.??e-5??
            maxVelocity = 4
            maxVelocityTwoAxis = ??
            maxVelocityThreeAxis = 3
            minVelocity = 0.019??
        """
        if self.handle > 0:
            #Device attached
            print('Device attached: ' + str(self.mcl.MCL_DeviceAttached(ctypes.c_uint(500), self.handle)))
            #Serial number
            print('SN: ' + str(self.mcl.MCL_GetSerialNumber(self.handle)))
            #Product ID:
            PID = ctypes.pointer(ctypes.c_ushort())
            self.mcl.MCL_GetProductID(PID, self.handle)
            print('PID: ' + str(PID.contents.value))
            #Encoder, StepSize and Velocities
            encoderResolution = ctypes.pointer(ctypes.c_double())
            stepSize = ctypes.pointer(ctypes.c_double())
            maxVelocity = ctypes.pointer(ctypes.c_double())
            maxVelocityTwoAxis = ctypes.pointer(ctypes.c_double())
            maxVelocityThreeAxis = ctypes.pointer(ctypes.c_double())
            minVelocity = ctypes.pointer(ctypes.c_double())
            self.mcl.MCL_MDInformation(encoderResolution, stepSize, maxVelocity, maxVelocityTwoAxis, maxVelocityThreeAxis, minVelocity, self.handle)
            print('encoderResolution: ' + str(encoderResolution.contents.value))
            print('stepSize: ' + str(stepSize.contents.value))
            print('maxVelocity: ' + str(maxVelocity.contents.value))
            print('maxVelocityTwoAxis: ' + str(maxVelocityTwoAxis.contents.value))
            print('maxVelocityThreeAxis: ' + str(maxVelocityThreeAxis.contents.value))
            print('minVelocity: ' + str(minVelocity.contents.value))
        else:
            print('Invalid handle. No device is connncted.')

    def closeConnection(self):
        """
        Closes the connection by releasing the handle.
        """
    #    self.mcl.MCL_ReleaseAllHandles()
        self.stopMoving()
        self.mcl.MCL_ReleaseHandle(self.handle)
        print('Handle released.')

    def __exit__(self, exception_type, exception_value, traceback):
        if not(self.released == True):
            if exception_value == None:
                print('Handle released')
            else:
                print('An unexpected error occured. Releasing handle now.')
                self.closeConnection()
    def __del__(self):
        self.mcl.MCL_ReleaseHandle(self.handle)
        print('Deconstructor was called.')


#
# The MIT License
#
# Copyright (c) 2009 Zhuang Lab, Harvard University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
