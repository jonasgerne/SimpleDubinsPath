# SimpleDubinsPath
This is a simple MATLAB script to create the shortest [Dubins path](https://en.wikipedia.org/wiki/Dubins_path) using simple and easy to understand geometric operations. You can find a more sophisticated approach [here](https://github.com/EwingKang/Dubins-Curve-For-MATLAB).

### Usage
The script takes two parameters as input, the start and the goal configuration. The configuration is given in x and y coordinates and orientation in degrees. For example:
```MATLAB
Dubin([-1; -2; 0], [0; 0; 65])
```
will plot the shortest Dubins path starting from `(x,y) = (-1, -2)` at an angle of 0° degrees to the endpoint at `(x,y) = (0,0)` and a final orientation of 65°.   
You can leave the final orientation undetermined by setting the goal configuration's orientation to -1.

### Bugs
Please feel free to point out to me any bugs and issues you might bump into.
