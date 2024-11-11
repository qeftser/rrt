
# RRT Demos

Some visualizations of RRT to show it off and help explain it.

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
You can use the number keys to swap algorithms as well. The default is normal RRT.
   
```
Space Bar = Start/Pause simulation

Up,    K = move up
Down,  J = move down
Left,  H = move left
Right, L = move right
       I = zoom in
       O = zoom out
       C = clear edges
       E = empty environment
       X = display closest route to mouse position
       F = display only the closest route to mouse position
       S = change start point to mouse position and reset

Left  Mouse  = add obstacle
Right Mouse  = remove obstacle
Scroll Wheel = increase/decrease generation rate
Middle Mouse = reset generation rate

   Num 0 = Swap to RRT
   Num 1 = Swap to RRT*
   Num 2 = Swap to Q-RRT*
   Num 3 = Swap to RRTX
   Num 4 = Swap to SST
   Num 5 = Swap to SSTX
   Num 6 = Swap to RRT*FN
```
