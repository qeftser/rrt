
# RRT Demos

These are just some visualizations of RRT to show it off and help explain and understand it.

## Install

clone the repo:
```

$ git close https://github.com/qeftser/rrt.git

```
install sfml:
```
 /* you are on your own for windows xD */

$ sudo apt install libsfml-dev

```
cd into directory and build:
```
$ cd ./rrt 
$ mkdir build
$ cd ./build
$ cmake ..
$ make
```
run program (in build dir):
```
$ ./rrt_visual
```

## Controls

Movement can be done with the arrow keys or via vim motions. Zoom in and out is done via I and O. 
To start or pause/unpause the simualation use the space bar. The scroll wheel will increase or
decrease the speed new edges are generated at. If you have a scroll wheel button, that will reset
the rate to the default. The C key will clear all edges and restart. To add obstacles, hold down
the left mouse button and drag. Doing the same thing with the right mouse button will remove obstacles.
   
```
Up,    K = move up
Down,  J = move down
Left,  H = move left
Right, L = move right
       I = zoom in
       O = zoom out
       C = clear edges
       E = empty environment

Left  Mouse  = add obstacle
Right Mouse  = remove obstacle
Scroll Wheel = increase generation rate
Middle Mouse = reset generation rate
```
