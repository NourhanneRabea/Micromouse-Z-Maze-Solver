
#ifndef __UTILITIES__
#define __UTILITIES__

#include "stdio.h"
#include <math.h>
#define v_Walls_BASE ((ROWS - 1) * 10 + (COLS))     // last address of the eeprom to Save the Cells values
#define h_Walls_BASE 2 * ((ROWS - 1) * 10 + (COLS)) // last address of the eeprom to Save the V_Walls values
// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4

#define SPEED_OF_SOUND_CM_PER_US 0.0343 // Speed of sound in centimeters per microsecond
#define MM_PER_CM 10

#define NAV_switch_Pin 0
#define RUN_switch_Pin 1
#define CLR_EEPROM_switch_Pin 2

#define SMALLEST_DISTANCE 0
#define THRESHOLD_DISTANCE 75
#define LARGEST_DISTANCE 2880

// Define constants for maze dimensions
#define ROWS 16
#define COLS 16
#define Maze_Border 255 // A large number to prevent out-of-range
#define x_fin 8
#define y_fin 8

// Define Ultrasonic pins
#define lecho PA9
#define ltrigger PA10

#define fecho PA11
#define ftrigger PA12

#define recho PA15
#define rtrigger PB3

#define Motor_A_6 PB9  //          IN4
#define Motor_A_5 PB7 // Channel A IN3
#define Motor_B_4 PB8  // Channel B IN2
#define Motor_B_3 PB6 //          IN1

typedef enum DIRECTION
{
  DONTCARE,
  NORTH,
  SOUTH,
  EAST,
  WEST
} E_Direction_t;

#define M_distance 100
#define DONE_Navigating 3
#define WINDOW_SIZE 10 // Window size for the moving average filter
extern _Bool MovedCell;

extern uint8_t x_cor;
extern uint8_t y_cor;
extern uint8_t trail[ROWS][COLS];
extern uint8_t deadend[ROWS][COLS];
extern uint8_t pot_field[ROWS][COLS];
extern uint8_t FirtPath;

extern uint8_t Blocking_V_Walls[ROWS + 1][COLS];
extern uint8_t Blocking_H_Walls[ROWS][COLS + 1];
extern uint8_t decision;
extern uint8_t Navegating_times;
extern uint8_t way_left;
extern uint8_t way_front;
extern uint8_t way_right;

extern uint8_t pot_north;
extern uint8_t pot_east;
extern uint8_t pot_south;
extern uint8_t pot_west;

extern uint8_t trail_north;
extern uint8_t trail_east;
extern uint8_t trail_south;
extern uint8_t trail_west;

extern unsigned long start_time;
extern unsigned long previousTime;
extern float total_displacement;
extern double ax_samples[WINDOW_SIZE];
extern int sample_index;

extern float ax_offset, ay_offset, az_offset;
extern float gyroXcal, gyroYcal, gyroZcal;

extern float yaw;
extern float PrevYaw;
extern float Accelration;
extern float Velocity;
extern int Displacement;
extern _Bool t_flag;

// extern uint8_t Cell[ROWS][COLS];
// extern uint8_t PATH[ROWS][COLS];

extern uint8_t isNavigating;


#endif //__UTILITIES__