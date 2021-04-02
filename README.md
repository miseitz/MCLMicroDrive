# MCLMicroDrive
Python class to control a Mad City Labs MicroDrive with 3 axes (26.1mm range) and 3 encoders (50 nm resolution).

The class calls C-functions from the dll (provided by Mad City Labs), converts the python input into the C format, and returns the output from the MicroDrive.
The class was written for a new MicroDrive with 3 axes and 3 encoders, which was bought in the beginning of 2020 (in case there is a significant software update).

## How to get the MicroDrive class working

1. ...
2. ...

## Functions of the MicroDrive class
The list of the c-functions from Mad City Lab is provided in the word document in this folder. Functions in this python wrapper were derived from all of these c-functions. Here, is a brief description of the functions.

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
            stepSize =  9.525e-5
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


PS: Please let me know if you find any mistakes such that we can fix it for everyone! Thanks!
