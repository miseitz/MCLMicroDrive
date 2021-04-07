# -*- coding: utf-8 -*-
"""
Example for the Mad City Labs MicroDrive with 3 axes (26.1mm range) and encoders (50 nm resolution).

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
import MicroDrive_Michael_3 as mcl

#%%
motor = MicroDrive() # Create the MicroDrive object
motor.centerHomePosition() # Bring center the encoders to the middle of the range
motor.getPosition() # get the position
motor.move(5,10,15) # move to (5,10,15)
motor.getPosition() # get the position (should be: (5,10,15))

#%%
motor.closeConnection()
del motor
#%%
motor = MicroDrive() # Create the MicroDrive object
#%%
# Here we will have the stages move in circles. You will have to stop the console to stop the movement.
t = 0
while True:
    t += 1
    x = 5*np.sin(2*np.pi*t/100)
    y = 5*np.sin(2*np.pi*t/100 + np.pi/2)
    z = 5*np.sin(2*2*np.pi*t/100)
    print(x,y,z)
    motor.move(x,y,z)

#%%
motor.closeConnection()
del motor