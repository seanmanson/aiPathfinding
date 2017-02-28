aiPathfinding
=============

Simple pathfinding algorithm for a robot with an arm chain, used for
UQ's 2015 COMP3702 artificial intelligence course.

Given a specified input file containing robot and obstacle information,
find the best path to take from the start to end point without colliding
with any obstacles.

Also included is the visualiser given to us by the lecturer.

Installation
------------

Use Ant on build.xml to build a .jar file, which can be used to run the algorithm.
Alternatively, import into eclipse using the build.xml file as a build script.

Usage
-----
```
jar a1-3702.jar inputFileName outputFileName
```

Input/Output formats
--------------------

A robot's configuration is given by the x/y position and the relative angles between each chain in its arm, each separated by spaces:
```
xposition yposition [jointangle1] [jointangle2] [...]
```
Each of these values have double precision. x and y positions are given between 0.0 and 1.0, while the angles are between -5pi/6 to 5pi/6 radians. The arms must not collide.

Obstacles are rectangles given simply by the x/y coordinates of their topleft and bottom right corners:
```
topleftx toplefty bottomrightx bottomrighty
```

From this, the input file format is given as follows, with newlines between each piece of information:
```
startconfiguration
endconfiguration
numberofobstacles
[obstacle1]
[obstacle2]
[...]
```

Similarly, the output file format is a path of valid configurations from the start to end:
```
startconfiguration
[intermediateconfiguration1]
[intermediateconfiguration2]
...
[intermediateconfiguration9999]
endconfiguration
```
The steps between each configuration in the output depend on some hardcoded epsilon values.
