# virtual-hrp-robot
Package that emulates a robot using the HID-Robot-Protocol (hrp)

## Install
```npm install virtual-hrp-robot```

## Use
var robot = require('virtual-hrp-robot')('robotName');

where _robotName_ is the name of the js file located in the folder ```robots/```.

The virtual robot will start listening for connections on the port 5555. You should be able to communicate using the HID-Robot-Protocol ([hrp](https://github.com/andresmanelli/hrp)).

## Notes
 - Rotation of end effector is not supported, this package only deals with displacements for the moment.
 - To create another robot, define:
  - QX for the joints IDs
  - X,Y,Z for the axis names
  - directK(joints):
    - _joints_ :
    ```js
    {
      ID1: val,
      ID2: val,
      ID3: val
    }
    ```
  - inverseK(position,joints):
    - _position_:
    ```js
    {
      X: val,
      Y: val,
      Z: val
    }
    ```
    - _joints_: same as before
    
- See ```robots/ScaraRobot.js``` for an example
