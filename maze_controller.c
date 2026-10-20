/*
 * CSCI291 Project - Webots maze solver (e-puck, C)
 *
 * Strategy
 *  1. EXPLORE: left-hand wall following. At every real junction (>= 2 exits) the
 *     decision (L/S/R) is pushed on a path stack; at a dead end a U-turn (B) is
 *     pushed. After every push, "x B y" triples are collapsed
 *     (e.g. L B L -> S), so the stack always holds the DIRECT route from the
 *     start to the current spot.
 *  2. At each dead end the robot stops, averages its light sensors and stores
 *     the brightness plus a copy of the direct route to that dead end.
 *  3. The lap ends when the robot is back at the start pose (odometry).
 *  4. NAVIGATE: the robot faces its initial heading, replays the stored route
 *     of the brightest station (no searching) and stops there.
 *
 * Nothing is hard-coded to a particular map: only sensor thresholds and the
 * corridor width (CELL_HALF) need tuning - see the TUNING block.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------ constants ------------------------------ */
#define TIME_STEP      32
#define WHEEL_RADIUS   0.0205   /* m  (e-puck) */
#define AXLE_LENGTH    0.052    /* m  (e-puck) */
#define MAX_STATIONS   16
#define MAX_PATH       256

/* ------------------------------ TUNING --------------------------------- */
/* Print raw ps values (DEBUG_SENSORS 1) and adjust these for your map.      */
#define DEBUG_SENSORS     0
#define CRUISE_SPEED      4.0    /* rad/s, max is 6.28                      */
#define TURN_SPEED        2.5
#define K_STEER           0.004  /* wall-centering gain                     */
#define WALL_ON           180.0  /* side sensor above this  -> wall present */
#define WALL_OFF          110.0  /* side sensor below this  -> opening      */
#define SIDE_TARGET       260.0  /* side reading when centred next to 1 wall*/
#define FRONT_STOP_THR    300.0  /* front reading at which we stop (node)   */
#define FRONT_WALL_THR    220.0  /* front reading counted as "wall ahead"   */
#define CELL_HALF         0.06   /* m, distance from opening edge to the
                                    junction centre (~half corridor width)  */
#define LEAVE_DIST        0.08   /* m, ignore side openings after a node    */
#define START_TOL         0.12   /* m, tolerance for "back at start"        */
#define LIGHT_HIGHER_IS_BRIGHTER 0 /* check the raw ls values in your world */

/* ------------------------------ types ---------------------------------- */
typedef enum { D_S = 0, D_R = 90, D_B = 180, D_L = 270 } Dir; /* angle, deg */

typedef struct {
  double brightness;
  Dir path[MAX_PATH];
  int len;
} Station;

/* ------------------------------ globals -------------------------------- */
static WbDeviceTag ps[8], ls[8], motorL, motorR, sensL, sensR;

static Dir path[MAX_PATH];
static int pathLen = 0;

static Station stations[MAX_STATIONS];
static int stationCount = 0;

static int heading = 0;               /* quarter turns, 0 = initial heading   */
static double posX = 0.0, posY = 0.0; /* dead-reckoned position (m)          */
static double prevAngL = 0.0, prevAngR = 0.0;
static int leftWall = 0, rightWall = 0; /* latched, with hysteresis          */
static int exploring = 1;

static const int dirX[4] = {0, 1, 0, -1};
static const int dirY[4] = {1, 0, -1, 0};

/* ------------------------- low-level helpers --------------------------- */
static void init_devices(void);
static void step(void);
static void set_speed(double l, double r);
static void stop_motors(void);
static double clampd(double v, double lo, double hi);

/* sensing */
static void update_walls(void);
static double front_value(void);
static void classify_node(int *left, int *front, int *right);
static void settle(int steps);

/* motion */
static void update_odometry(void);
static int  lap_complete(void);
static void steer_forward(void);
static void advance(double dist);
static void turn_quarter(int dir);
static void turn_dir(Dir d);
static void face_initial_heading(void);
static int  drive_to_next_node(void);
static void leave_node(void);

/* path handling */
static void push_decision(Dir d);
static void reduce_path(void);

/* light */
static double measure_light(void);

/* top level */
static void explore(void);
static int  brightest_station(void);
static void navigate_to_station(int idx);

/* ------------------------------ implementation ------------------------- */
static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void init_devices(void) {
  char name[8];
  wb_robot_init();
  for (int i = 0; i < 8; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    sprintf(name, "ls%d", i);
    ls[i] = wb_robot_get_device(name);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }
  motorL = wb_robot_get_device("left wheel motor");
  motorR = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(motorL, INFINITY);
  wb_motor_set_position(motorR, INFINITY);
  set_speed(0, 0);
  sensL = wb_robot_get_device("left wheel sensor");
  sensR = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(sensL, TIME_STEP);
  wb_position_sensor_enable(sensR, TIME_STEP);
  step(); /* first step so the sensors hold valid values */
  prevAngL = wb_position_sensor_get_value(sensL);
  prevAngR = wb_position_sensor_get_value(sensR);
}

static void step(void) {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}

static void set_speed(double l, double r) {
  wb_motor_set_velocity(motorL, clampd(l, -6.28, 6.28));
  wb_motor_set_velocity(motorR, clampd(r, -6.28, 6.28));
}

static void stop_motors(void) { set_speed(0, 0); }

/* ---- sensing: e-puck ps2 = right, ps5 = left, ps0/ps7 = front ---------- */
static void update_walls(void) {
  double l = wb_distance_sensor_get_value(ps[5]);
  double r = wb_distance_sensor_get_value(ps[2]);
  if (l > WALL_ON) leftWall = 1;  else if (l < WALL_OFF) leftWall = 0;
  if (r > WALL_ON) rightWall = 1; else if (r < WALL_OFF) rightWall = 0;
#if DEBUG_SENSORS
  printf("ps0=%.0f ps2=%.0f ps5=%.0f ps7=%.0f\n",
         wb_distance_sensor_get_value(ps[0]), r, l,
         wb_distance_sensor_get_value(ps[7]));
#endif
}

static double front_value(void) {
  return 0.5 * (wb_distance_sensor_get_value(ps[0]) +
                wb_distance_sensor_get_value(ps[7]));
}

/* Which exits does the current junction have? (1 = open) */
static void classify_node(int *left, int *front, int *right) {
  *left  = wb_distance_sensor_get_value(ps[5]) < WALL_ON;
  *right = wb_distance_sensor_get_value(ps[2]) < WALL_ON;
  *front = front_value() < FRONT_WALL_THR;
}

static void settle(int steps) {
  stop_motors();
  for (int i = 0; i < steps; i++) step();
}

/* ---- odometry ---------------------------------------------------------- */
static void update_odometry(void) {
  double aL = wb_position_sensor_get_value(sensL);
  double aR = wb_position_sensor_get_value(sensR);
  double d = WHEEL_RADIUS * ((aL - prevAngL) + (aR - prevAngR)) / 2.0;
  int h = ((heading % 4) + 4) % 4;
  posX += d * dirX[h];
  posY += d * dirY[h];
  prevAngL = aL;
  prevAngR = aR;
}

/* True when the exploration lap is finished: at least one dead end seen and
 * we are back at the start pose with the initial heading.  In a tree-like
 * maze each corridor is travelled once per direction under the left-hand
 * rule, so this can only happen at the end of the lap. */
static int lap_complete(void) {
  return exploring && stationCount >= 1 && (((heading % 4) + 4) % 4) == 0 &&
         hypot(posX, posY) < START_TOL;
}

/* ---- motion ------------------------------------------------------------ */
/* One control tick of straight driving with wall centring. */
static void steer_forward(void) {
  double l = wb_distance_sensor_get_value(ps[5]);
  double r = wb_distance_sensor_get_value(ps[2]);
  double err = 0.0;
  if (leftWall && rightWall)  err = l - r;
  else if (leftWall)          err = l - SIDE_TARGET;
  else if (rightWall)         err = SIDE_TARGET - r;
  double corr = clampd(K_STEER * err, -1.5, 1.5);
  set_speed(CRUISE_SPEED + corr, CRUISE_SPEED - corr);
}

/* Drive straight for 'dist' metres (or until a front wall is reached). */
static void advance(double dist) {
  double travelled = 0.0;
  while (travelled < dist) {
    double bx = posX, by = posY;
    step();
    update_odometry();
    update_walls();
    if (front_value() >= FRONT_STOP_THR) break;
    steer_forward();
    travelled += hypot(posX - bx, posY - by);
  }
  stop_motors();
}

/* Rotate in place by 90 degrees: dir = +1 right (clockwise), -1 left. */
static void turn_quarter(int dir) {
  double target = (M_PI / 2.0) * (AXLE_LENGTH / 2.0) / WHEEL_RADIUS;
  double start = wb_position_sensor_get_value(sensL);
  for (;;) {
    step();
    update_odometry();
    double rem = target - fabs(wb_position_sensor_get_value(sensL) - start);
    if (rem < 0.02) break;
    double s = fmin(TURN_SPEED, 0.4 + 3.0 * rem);
    set_speed(dir * s, -dir * s);
  }
  stop_motors();
  heading += dir;
  settle(2);
  update_walls();
}

static void turn_dir(Dir d) {
  if (d == D_L) turn_quarter(-1);
  else if (d == D_R) turn_quarter(+1);
  else if (d == D_B) { turn_quarter(+1); turn_quarter(+1); }
}

static void face_initial_heading(void) {
  int h = ((heading % 4) + 4) % 4;
  if (h == 1) turn_quarter(-1);
  else if (h == 2) { turn_quarter(+1); turn_quarter(+1); }
  else if (h == 3) turn_quarter(+1);
}

/* Move out of the junction centre, ignoring the side openings we just left. */
static void leave_node(void) {
  advance(LEAVE_DIST);
  update_walls();
}

/* Follow the corridor until the next node (junction, corner or dead end).
 * Returns 1 when a node is reached, 0 when the exploration lap is complete. */
static int drive_to_next_node(void) {
  int prevL = leftWall, prevR = rightWall;
  for (;;) {
    step();
    update_odometry();
    update_walls();
    if (lap_complete()) { stop_motors(); return 0; }
    if (front_value() >= FRONT_STOP_THR) { settle(3); return 1; }
    /* a wall that was there has disappeared -> side opening */
    if ((prevL && !leftWall) || (prevR && !rightWall)) {
      advance(CELL_HALF);  /* go to the junction centre */
      settle(3);
      return 1;
    }
    prevL = leftWall;
    prevR = rightWall;
    steer_forward();
  }
}

/* ---- path stack -------------------------------------------------------- */
/* Collapse "x B y" into one move: angles add modulo 360. */
static void reduce_path(void) {
  while (pathLen >= 3 && path[pathLen - 2] == D_B) {
    int total = (path[pathLen - 3] + D_B + path[pathLen - 1]) % 360;
    pathLen -= 3;
    path[pathLen++] = (Dir)total;
  }
}

static void push_decision(Dir d) {
  if (pathLen < MAX_PATH) path[pathLen++] = d;
  reduce_path();
}

/* ---- light ------------------------------------------------------------- */
/* Average of the summed light sensors over 10 steps; larger = brighter. */
static double measure_light(void) {
  double acc = 0.0;
  stop_motors();
  for (int k = 0; k < 10; k++) {
    step();
    double s = 0.0;
    for (int i = 0; i < 8; i++) s += wb_light_sensor_get_value(ls[i]);
    acc += s;
  }
  acc /= 10.0;
  return LIGHT_HIGHER_IS_BRIGHTER ? acc : -acc;
}

/* ---- top level --------------------------------------------------------- */
static void explore(void) {
  exploring = 1;
  update_walls();
  leave_node(); /* get going so the first reading is clean */
  while (drive_to_next_node()) {
    int l, f, r;
    classify_node(&l, &f, &r);
    int exits = l + f + r;

    if (exits == 0) {                       /* dead end: sample the light */
      if (stationCount < MAX_STATIONS) {
        Station *s = &stations[stationCount++];
        s->brightness = measure_light();
        s->len = pathLen;                   /* direct route start -> here */
        memcpy(s->path, path, sizeof(Dir) * pathLen);
        printf("Station %d: brightness %.1f, route length %d\n",
               stationCount, s->brightness, s->len);
      }
      turn_dir(D_B);
      push_decision(D_B);
    } else {
      Dir d = l ? D_L : (f ? D_S : D_R);    /* left-hand rule */
      turn_dir(d);
      if (exits >= 2) push_decision(d);     /* only real junctions count */
    }
    leave_node();
  }
  exploring = 0;
  stop_motors();
}

static int brightest_station(void) {
  int best = 0;
  for (int i = 1; i < stationCount; i++)
    if (stations[i].brightness > stations[best].brightness) best = i;
  return best;
}

/* Replay the stored route; stop at the dead end it leads to. */
static void navigate_to_station(int idx) {
  const Station *s = &stations[idx];
  int k = 0;
  face_initial_heading();
  leave_node();
  while (drive_to_next_node()) {
    int l, f, r;
    classify_node(&l, &f, &r);
    int exits = l + f + r;
    if (exits == 0) break;                  /* reached the station */
    Dir d;
    if (exits == 1) d = l ? D_L : (f ? D_S : D_R);
    else            d = (k < s->len) ? s->path[k++] : D_S;
    turn_dir(d);
    leave_node();
  }
  stop_motors();
}

int main(void) {
  init_devices();

  explore();
  if (stationCount == 0) {
    printf("No light station found.\n");
    for (;;) step();
  }

  int best = brightest_station();
  printf("Brightest: station %d (%.1f). Heading there.\n", best + 1,
         stations[best].brightness);
  navigate_to_station(best);
  printf("Arrived at the brightest station.\n");

  for (;;) { stop_motors(); step(); }
  return 0;
}
