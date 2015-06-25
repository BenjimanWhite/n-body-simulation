N-Body-Simulation
=================

####A serial version of the N Body Simulation, written in C

*Miriam Robinson, Ben White, Chris Mitchell*

Lewis & Clark College - summer 2012

#Run this on Mac OSX

You'll need the x11 library in order to run this program: http://xquartz.macosforge.org/landing/
  
Compile the program with:

`n-body.c -I/opt/X11/include -L/opt/X11/lib -lm -lX11`

run with:

 `./a.out`

To specify the number of bodies in the world, the program also
optionally accepts an integer as its second command line argument:
 
 `./a.out 10 // 10 bodies`

