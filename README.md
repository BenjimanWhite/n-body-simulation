N-Body-Simulation
=================

####A serial version of the N Body Simulation, written in C

*Miriam Robinson, Ben White, Chris Mitchell*

Lewis & Clark College - summer 2012

You'll need the x11 library in order to run this program.
  
Compile the program with:

```gcc n-body.c `pkg-config --cflags --libs x11` -lm```


run with:

 `./a.out`

To specify the number of bodies in the world, the program also
optionally accepts an integer as its second command line argument:
 
 `./a.out 10 // 10 bodies`

