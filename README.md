# MCLMicroDrive
Python class to control a Mad City Labs MicroDrive with 3 axes (26.1mm range) and 3 encoders (50 nm resolution).

The class calls C-functions from the dll (provided by Mad City Labs), converts the python input into the C format, and returns the output from the MicroDrive.
The class was written for a new MicroDrive with 3 axes and 3 encoders, which was bought in the beginning of 2020 (in case there is a significant software update).

## Files in this directory
- MicroDrive_2_0.doc: Description of C-functions provided by Mad City Labs
- MicroDrive_v1.pyx: Python module/wrapper
- setup.py: Setup file to build the .c file
- MCL_MicroDriveExample.py: Example file on how to create a MicroDrive object, home the encoders, move the stage, and readout the encoder position.

## How to get the MicroDrive class working

    1. Dowload the MicroDrive_v1.pyx and setup.py file to one folder.
    2. Copy the MicroDrive.dll file from Mad City Labs into the same folder (should come with your installation files from Mad City Labs)
    3. In the "MicroDrive_v1.pyx" file you will want to update the standard value for "mcl_lib" as one of the initialization arguments of the MicroDrive class. Replace the path with the folder where you saved the dll file to. If you do not do that, you will have to provide the path everytime you create a new MicroDrive object. 
    4. Now lets build the module: Open the terminal (if you use Anaconda, open the Anaconda terminal)
    5. Navigate to your folder.
    6. Run: "python setup.py build_ext --inplace"
    7. Done!

Now you can import the module in your scripts through: "import MicroDrive".
For this to work properly you have to have the your script in the same folder as the MicroDrive.c file. 
Or add the folder of the MicroDrive.c file to the path for modules in python through:

     import sys
     sys.path.insert(0, pathToFolderOfMicroDrive) # e.g. r'C:\Program Files\MCL MicroDrive'

## Functions of the MicroDrive class
The list of the C-functions from Mad City Lab is provided in the word document MicroDrive_2_0.doc. Functions in this python wrapper were derived from all of these C-functions. Here, is a brief description of the functions that were implemented.

Functions include (movements and velocities are in mm and mm/s, respectively):

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

Wait should be called after the movement to guarantee that the motors are not moving anymore. Trying to read the encoders or move the motors, while they are still movingn, might affect the internal timing pulses. However, wait should not  be called multiple times, as it is slowed if it is called more then once.


PS: Please let me know if you find any mistakes such that we can fix it for everyone! Thanks!
