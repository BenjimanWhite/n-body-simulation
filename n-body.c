/*
  N Body Simulation - Miriam Robinson, Ben White, Chris Mitchell
  Lewis & Clark College - summer 2012
  
  Compile with:
  gcc n-body.c `pkg-config --cflags --libs x11` -lm

  run with:
  ./a.out

  To specify the number of bodies in the world, the program also
  optionally accepts an integer as its second command line argument:
  ./a.out 10 // 10 bodies
*/

#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <X11/Xlib.h>

#define WIDTH 900
#define HEIGHT 500

// default number of bodies
#define DEF_NUM_BODIES 200
// gravitational constant
#define GRAV 10.0
// initial velocities are scaled by this value
#define V_SCALAR 20.0
// initial masses are scaled by this value
#define M_SCALAR 5.0
// radius scalar
#define R_SCALAR 3
// coefficient of restitution determines the elasticity of a collision: C_REST = [0,1]
//  if C_REST = 0 -> perfectly inelastic (particles stick together)
//  if C_REST = 1 -> perfectly elastic (no loss of speed)
#define C_REST 0.9

struct body {
    double x, y; // position
    double vx, vy; // velocity
    double m; // mass
    double r; // radius of the particle
};

struct world {
    struct body *bodies;
    int num_bodies;
     /* The prev_force array stores the x and y components of the total force acting
     * on each body from the previous time step. The forces are index like this:
     *     F on body i in the x dir = F[2*i]
     *     F on body i in the y dir = F[2*i+1] */
    double *prev_force;
};


/* This function initializes each particle's mass, velocity and position */
struct world* create_world(int num_bodies) {
    struct world *world = malloc(sizeof(struct world));

    world->num_bodies = num_bodies;
    world->bodies = malloc(sizeof(struct body)*num_bodies);
    world->prev_force = malloc(2 * sizeof(struct body) * num_bodies);
    world->prev_force = memset(world->prev_force, 0, 2 * sizeof(struct body) * num_bodies);

    int i = 0;
    double x;
    double y;
    double rc;

    int min_dim = (WIDTH < HEIGHT) ? WIDTH : HEIGHT;

    while (i<num_bodies) {
        x = drand48() * WIDTH;
        y = drand48() * HEIGHT;
        rc = sqrt((WIDTH/2-x)*(WIDTH/2-x) + (y-HEIGHT/2)*(y-HEIGHT/2));
        if (rc <= min_dim/2) {
            world->bodies[i].x = x;
            world->bodies[i].y = y;

            world->bodies[i].vx = V_SCALAR * (y-HEIGHT/2) / rc;
            world->bodies[i].vy = V_SCALAR * (WIDTH/2-x) / rc;

            world->bodies[i].m = (1 / (0.025 + drand48())) * M_SCALAR;
            world->bodies[i].r = sqrt(world->bodies[i].m / M_PI) * R_SCALAR;
            i++;
        }
    }
    return world;
}

// set the foreground color given RGB values between 0..255.
void set_color(Display *disp, GC gc, int r, int g, int b){
  unsigned long int p ;

  if (r < 0) r = 0; else if (r > 255) r = 255;
  if (g < 0) g = 0; else if (g > 255) g = 255;
  if (b < 0) b = 0; else if (b > 255) b = 255;

  p = (r << 16) | (g  << 8) | (b) ;

  XSetForeground(disp, gc, p) ;
}


/* This function updates the screen with the new positions of each particle */
void draw_world(Display *disp, Pixmap back_buf, GC gc, struct world *world) {
    int i;
    double x, y, r, r2;

    // we turn off aliasing for faster draws
    set_color(disp, gc, 255, 255, 255);
    XFillRectangle(disp, back_buf, gc, 0, 0, WIDTH, HEIGHT);

    for (i = 0; i < world->num_bodies; i++) {
        r = world->bodies[i].r;
        x = world->bodies[i].x - r;
        y = world->bodies[i].y - r;
        r2 = r + r;

        // draw body
        set_color(disp, gc, 255*7/10, 255*7/10, 255*7/10);
        XFillArc(disp, back_buf, gc, x, y, r2, r2, 0, 360*64); 
        set_color(disp, gc, 0, 0, 0);
        XDrawArc(disp, back_buf, gc, x, y, r2, r2, 0, 360*64); 
    }
}

void collision_step(struct world *world) {
    int a, b;
    double r, x, y, vx, vy;
    double diff_x, diff_y, d, R;
    double norm_x, norm_y;
    double va_norm, vb_norm;
    double va_tan, vb_tan;
    double new_va_norm, new_vb_norm;

    // Impose screen boundaries by reversing direction if body is off screen
    for (a = 0; a < world->num_bodies; a++) {
        r = world->bodies[a].r;
        x = world->bodies[a].x;
        y = world->bodies[a].y;
        vx = world->bodies[a].vx;
        vy = world->bodies[a].vy;

        if (x-r < 0) { // left edge
            if (vx < 0) { world->bodies[a].vx = -C_REST * vx; }
            world->bodies[a].x = r;
        } else if (x+r > WIDTH) { // right edge
            if (vx > 0) { world->bodies[a].vx = -C_REST * vx; }
            world->bodies[a].x = WIDTH - r;
        }

        if (y-r < 0) { // bottom edge
            if (vy < 0) { world->bodies[a].vy = -C_REST * vy; }
            world->bodies[a].y = r;
        } else if (y+r > HEIGHT) { // top edge
            if (vy > 0) { world->bodies[a].vy = -C_REST * vy; }
            world->bodies[a].y = HEIGHT - r;
        }

        // check for particle `a` colliding with other particles `b`.
        for (b = 0; b < a; b++) {
            diff_x = world->bodies[b].x - world->bodies[a].x;
            diff_y = world->bodies[b].y - world->bodies[a].y;
            d = sqrt((diff_x * diff_x) + (diff_y * diff_y));

            /* Handle particle collisions */
            R = world->bodies[a].r + world->bodies[b].r; // R is the sum of the radii
            if (d <= R) {
                // TODO: Same position handling. Have not encountered this condition yet.
                if (d == 0) {
                    printf("Particles exactly overlap!!!\n");
                    continue;
                }
                norm_x = diff_x / d;
                norm_y = diff_y / d;

                // Move particles apart so there is no overlap
                world->bodies[a].x -= 1.0001 * (R - d) * norm_x;
                world->bodies[a].y -= 1.0001 * (R - d) * norm_y;

                world->bodies[b].x += 1.0001 * (R - d) * norm_x;
                world->bodies[b].y += 1.0001 * (R - d) * norm_y;

                // Calculate post-bounce velocities (in terms of the
                // bounce's normal/tangential coordinate system)
                va_norm = norm_x * world->bodies[a].vx + norm_y * world->bodies[a].vy;
                va_tan = -norm_y * world->bodies[a].vx + norm_x * world->bodies[a].vy;

                vb_norm = norm_x * world->bodies[b].vx + norm_y * world->bodies[b].vy;
                vb_tan = -norm_y * world->bodies[b].vx + norm_x * world->bodies[b].vy;

                new_va_norm = 
                    (C_REST * world->bodies[b].m * (vb_norm - va_norm) + 
                        (world->bodies[a].m * va_norm) +
                        (world->bodies[b].m * vb_norm)
                    ) / (world->bodies[a].m + world->bodies[b].m);
                new_vb_norm = 
                    (C_REST * world->bodies[a].m * (va_norm - vb_norm) + 
                        (world->bodies[a].m * va_norm) +
                        (world->bodies[b].m * vb_norm)
                    ) / (world->bodies[a].m + world->bodies[b].m);

                world->bodies[a].vx = new_va_norm * norm_x - va_tan * norm_y;
                world->bodies[a].vy = new_va_norm * norm_y + va_tan * norm_x;

                world->bodies[b].vx = new_vb_norm * norm_x - vb_tan * norm_y;
                world->bodies[b].vy = new_vb_norm * norm_y + vb_tan * norm_x;
            }
        }
    }
}

void position_step(struct world *world, double time_res) {
    int i, j;
    double d, d_cubed, diff_x, diff_y;

    /* The forces array stores the x and y components of the total force acting
     * on each body. The forces are index like this:
     *     F on body i in the x dir = F[2*i]
     *     F on body i in the y dir = F[2*i+1] */
    double *force = malloc(2 * sizeof(struct body) * world->num_bodies);
    // initialize all forces to zero
    force = memset(force, 0, 2 * sizeof(struct body) * world->num_bodies);

    /* Compute the net force on each body */
    for (i = 0; i < world->num_bodies; i++) {
        for (j = 0; j < world->num_bodies; j++) {
            if (i == j) {
                continue;
            }
            // Compute the x and y distances and total distance d between
            // bodies i and j
            diff_x = world->bodies[j].x - world->bodies[i].x;
            diff_y = world->bodies[j].y - world->bodies[i].y;
            d = sqrt((diff_x * diff_x) + (diff_y * diff_y));

            // Check if the particles are too close
            if (d < 25) {
                d = 25;
            }
            d_cubed = d * d * d;
            // Add force due to j to total force on i
            force[2*i] += GRAV * (world->bodies[i].m * world->bodies[j].m
                    / d_cubed) * diff_x;
            force[2*i+1] += GRAV * (world->bodies[i].m * world->bodies[j].m
                    / d_cubed) * diff_y;
        }
    }

    // Update the velocity and position of each body
    for (i = 0; i < world->num_bodies; i++) {
        // Update positions
        world->bodies[i].x += world->bodies[i].vx * time_res + world->prev_force[2*i] * pow(time_res,2) / (2 * world->bodies[i].m);
        world->bodies[i].y += world->bodies[i].vy * time_res + world->prev_force[2*i+1] * pow(time_res,2) / (2 * world->bodies[i].m);

        // Update velocities
        world->bodies[i].vx += (world->prev_force[2*i] + force[2*i]     ) * time_res / (2 * world->bodies[i].m);
        world->bodies[i].vy += (world->prev_force[2*i+1] + force[2*i+1] ) * time_res / (2 * world->bodies[i].m);

        // Update previous force array
        world->prev_force[2*i] = force[2*i];
        world->prev_force[2*i+1] = force[2*i+1];
    }
}

void step_world(struct world *world, double time_res) {
    position_step(world, time_res);

    // the collision process is relaxative, doing it three times helps to
    // smooth over any errors that the previous relaxations introduced
    collision_step(world);
    collision_step(world);
    collision_step(world);
}


/* Main method runs initialize() and update() */
int main(int argc, char **argv) {
    /* get num bodies from the command line */
    int num_bodies;
    num_bodies = (argc == 2) ? atoi(argv[1]) : DEF_NUM_BODIES;
    printf("Universe has %d bodies.\n", num_bodies);

    /* set up the universe */
    time_t cur_time;
    time(&cur_time);
    srand48((long)cur_time); // seed the RNG used in create_world
    struct world *world = create_world(num_bodies);

    /* set up graphics using Xlib */
    Display *disp = XOpenDisplay(NULL);
    int scr = DefaultScreen(disp);
    Window win = XCreateSimpleWindow(
            disp,
            RootWindow(disp, scr),
            0, 0,
            WIDTH, HEIGHT,
            0,
            BlackPixel(disp, scr), WhitePixel(disp, scr));
    XStoreName(disp, win, "N-Body Simulator");

    Pixmap back_buf = XCreatePixmap(disp, RootWindow(disp, scr),
            WIDTH, HEIGHT, DefaultDepth(disp, scr));
    GC gc = XCreateGC(disp, back_buf, 0, 0);

    // Make sure we're only looking for messages about closing the window
    Atom del_window = XInternAtom(disp, "WM_DELETE_WINDOW", 0);
    XSetWMProtocols(disp, win, &del_window, 1);

    XSelectInput(disp, win, StructureNotifyMask);
    XMapWindow(disp, win);
    XEvent event;
    // wait until window is mapped
    while (1) {
        XNextEvent(disp, &event);
        if (event.type == MapNotify) {
            break;
        }
    }

    struct timespec delay={0, 1000000000 / 60}; // for 60 FPS
    struct timespec remaining;
    int frame_num = 0;
    while (1) {
        // check if the window has been closed
        if (XCheckTypedEvent(disp, ClientMessage, &event)) {
            break;
        }

        // we first draw to the back buffer then copy it to the front (`win`)
        draw_world(disp, back_buf, gc, world);
        XCopyArea(disp, back_buf, win, gc, 0, 0, WIDTH, HEIGHT, 0, 0);

        step_world(world, 0.1);
        frame_num++;
        nanosleep(&delay, &remaining);
    }

    XFreeGC(disp, gc);
    XFreePixmap(disp, back_buf);
    XDestroyWindow(disp, win);
    XCloseDisplay(disp);

    return 0;
}
