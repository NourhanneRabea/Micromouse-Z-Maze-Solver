
#include "ultrasonic.h"
#include "timer.h"
#include "mpuFunctions.h"
#include "motor_control.h"
#include "Floodfill.h"

uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];
uint8_t trail[ROWS][COLS];

uint8_t FirtPath = 0;
uint8_t Blocking_V_Walls[ROWS + 1][COLS]; // Vertical walls Readings
uint8_t Blocking_H_Walls[ROWS][COLS + 1]; // Horizontal walls Readings
uint8_t decision;
uint8_t way_left;
uint8_t way_front;
uint8_t way_right;

// // For every position of the robot, the potential field of every cell surrounding the robot
// // will be summoned. It'll be easier for us to determined where to go with this variable.
uint8_t pot_north;
uint8_t pot_east;
uint8_t pot_south;
uint8_t pot_west;
uint8_t trail_north;
uint8_t trail_east;
uint8_t trail_south;
uint8_t trail_west;


// Function to initialize maze cells for flood-fill algorithm

void Cells_init_FloodFill(void)
{
  for (uint8_t i = 0; i < ROWS; i++)
  {
    for (uint8_t j = 0; j < COLS; j++)
    {
      pot_field[i][j] = (uint8_t)abs(8 - i) + (uint8_t)abs(8 - j);
    }
  }
}
// Check if the micromouse has reached the finish cell
void finishcheck()
{
  if (x_cor == x_fin && y_cor == y_fin)
  {
    if (Navegating_times < DONE_Navigating)
    {
      Navegating_times++;
      delay(15 * 1000);
      x_cor = 0;
      y_cor = 0;
    }
  }
}
// Function to set a vertical wall at a specific location
void setVerticalWall(uint8_t x, uint8_t y)
{
  if (x >= 0 && x <= ROWS && y >= 0 && y < COLS)
  {
    // Write a value (e.g., 1) to the calculated EEPROM address
    // EEPROM.write(v_Walls_BASE + (x * ROWS + y), 1);
    Blocking_V_Walls[x][y] = 1;
  }
}
// Function to set a horizontal wall at a specific location
void setHorizontalWall(uint8_t x, uint8_t y)
{
  if (x >= 0 && x < ROWS && y >= 0 && y <= COLS)
  {
    // Write a value (e.g., 1) to the calculated EEPROM address
    // EEPROM.write(h_Walls_BASE + (x * ROWS + y), 1);
    Blocking_H_Walls[x][y] = 1;
  }
}
// Function to determine wall types and update maze values
void DetermineWallTypes(uint8_t x, uint8_t y)
{
  // Determine wall types and update maze values based on the current facing direction
  if (CurrentDirection() == NORTH)
  {
    if (way_left)
      setVerticalWall(x, y);
    if (way_front)
      setHorizontalWall(x, y + 1);
    if (way_right)
      setVerticalWall(x + 1, y);
  }
  else if (CurrentDirection() == EAST)
  {
    if (way_left)
      setHorizontalWall(x, y + 1);
    if (way_front)
      setVerticalWall(x + 1, y);
    if (way_right)
      setHorizontalWall(x, y);
  }
  else if (CurrentDirection() == SOUTH)
  {
    if (way_left)
      setVerticalWall(x + 1, y);
    if (way_front)
      setHorizontalWall(x, y);
    if (way_right)
      setVerticalWall(x, y);
  }
  else if (CurrentDirection() == WEST)
  {
    if (way_left)
      setHorizontalWall(x, y);
    if (way_front)
      setVerticalWall(x, y);
    if (way_right)
      setHorizontalWall(x, y + 1);
  }
}
// Check for walls using ultrasonic sensors and update variables
void wallcheck()
{
  Oop();
  // Check if there are walls in different directions
  way_left = wall_present(ltrigger_GPIO_Port,ltrigger_Pin,lecho_GPIO_Port, lecho_Pin);
  way_front = wall_present(ftrigger_GPIO_Port,ftrigger_Pin,fecho_GPIO_Port, fecho_Pin);
  way_right = wall_present(rtrigger_GPIO_Port,rtrigger_Pin,recho_GPIO_Port, recho_Pin);
  if (isNavigating && (Navegating_times != DONE_Navigating))
  {
    DetermineWallTypes(x_cor, y_cor);
  }
}
// Function to set all maze-related arrays to zero
void array_null()
{
  for (uint8_t n = 0; n < ROWS; n++)
  {
    for (uint8_t m = 0; m < COLS; m++)
    {
      trail[m][n] = 0;
      deadend[m][n] = 0;
      pot_field[m][n] = 0;
    }
  }
}
void Close_Cell_After_Deadend()
{
  if (CurrentDirection() == NORTH)
    setHorizontalWall(x_cor, y_cor);
  else if (CurrentDirection() == SOUTH)
    setHorizontalWall(x_cor, y_cor + 1);
  else if (CurrentDirection() == EAST)
    setVerticalWall(x_cor, y_cor);
  else if (CurrentDirection() == WEST)
    setVerticalWall(x_cor + 1, y_cor);
}
// Handle one-way paths
void oneway()
{
  if (way_left == 0)
  {
    // If the left path is open, turn left and move forward
    rotate_left();
    forward();
  }
  else if (way_right == 0)
  {
    // If the right path is open, turn right and move forward
    rotate_right();
    forward();
  }
  else if (way_front == 0)
  {
    // If the front path is open, move forward
    forward();
  }
}
// Perform a half-turn (rotate 180 degrees clockwise)
void half_turn()
{

  if (CurrentDirection() == NORTH)
  {
    while (CurrentDirection() == SOUTH)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == EAST)
  {
    while (CurrentDirection() == WEST)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == SOUTH)
  {
    while (CurrentDirection() == NORTH)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == WEST)
  {
    while (CurrentDirection() == EAST)
      Movements(RIGHT, 100, 100);
  }
}
void dead_end()
{
  // Execute a half-turn to retrace the path
  half_turn();
  wallcheck();
  decision = way_left + way_right + way_front;
  while (decision == 2)
  {
    deadend[x_cor][y_cor] = 1;
    trail[x_cor][y_cor]++;
    // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    oneway();
    Close_Cell_After_Deadend();
    wallcheck();
    decision = way_left + way_right + way_front;
  }
}
void Go_Back()
{
  half_turn();
  while (x_cor != 0 && y_cor != 0)
  {
    wallcheck();
    // Find_Lowest_Path();
  }
}

void two_way()
{
  // This subroutine will triggered when there are only two possible ways oppened.
  // Potential field will of every cell is important here in this case.

  // Access the array elements if within bounds
  pot_north = (y_cor + 1 < ROWS) ? pot_field[x_cor][y_cor + 1] : Maze_Border;
  pot_east = (x_cor + 1 < COLS) ? pot_field[x_cor + 1][y_cor] : Maze_Border;
  pot_south = (y_cor - 1 >= 0) ? pot_field[x_cor][y_cor - 1] : Maze_Border;
  pot_west = (x_cor - 1 >= 0) ? pot_field[x_cor - 1][y_cor] : Maze_Border;

  // Every possible way in the junction need to be listed so the program will know exactly
  // where to move.
  // Here are some possible way:

  // ------------------
  // Left way BLOCKED
  // ------------------
  if (way_left == 1)
  {
    // Facing north
    if (CurrentDirection() == NORTH)
    {
      // Deadend: Front
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        forward();
      }
      // if you didnt move there before
      else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor + 1])
      {
        if (pot_north < pot_east)
        {
          forward();
        }
        else if (pot_north >= pot_east)
        {
          rotate_right();
          forward();
        }
      }
      // if not entered the prev if then you must enter 1 of the following 2
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        //    if not
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the front cell before?
      {
        //    if not
        forward();
      }
    }

    // Facing east
    else if (CurrentDirection() == EAST)
    {
      // Deadend: Front
      if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
      {
        if (pot_east < pot_south)
        {
          forward();
        }
        else if (pot_east >= pot_south)
        {
          rotate_right();
          forward();
        }
      }
      // if not entered the prev if then you must enter 1 of the following 2
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
      {
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the benith cell before?
      {
        rotate_right();
        forward();
      }
    }
    // Facing South
    else if (CurrentDirection() == SOUTH)
    {
      // Deadend: Front
      if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        forward();
      }
      else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
      {
        if (pot_south <= pot_east)
        {
          forward();
        }
        else if (pot_south > pot_east)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        forward();
      }
    }

    // Facing west
    else if (CurrentDirection() == WEST)
    {
      // Deadend: Front
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        forward();
      }
      else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor + 1])
      {
        if (pot_north < pot_west)
        {
          rotate_right();
          forward();
        }
        else if (pot_north >= pot_west)
        {
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_right();
        forward();
      }
    }
  }
  // ------------------
  // front way BLOCKED
  // ------------------
  else if (way_front == 1)
  {
    // Facing north
    if (CurrentDirection() == NORTH)
    {
      // Deadend: Left
      if ((deadend[x_cor - 1][y_cor] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_east)
        {
          rotate_left();
          forward();
        }
        else if (pot_west >= pot_east)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_right();
        forward();
      }
    }

    // Facing east
    else if (CurrentDirection() == EAST)
    {
      // Deadend: Left
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor][y_cor - 1] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_left();
        forward();
      }

      else if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
      {
        if (pot_north < pot_south)
        {
          rotate_left();
          forward();
        }
        else if (pot_north >= pot_south)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
      }
    }

    // Facing South
    else if (CurrentDirection() == SOUTH)
    {
      // Deadend: Left
      if ((deadend[x_cor - 1][y_cor] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_east)
        {
          rotate_right();
          forward();
        }
        else if (pot_west >= pot_east)
        {
          rotate_left();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
      }
    }

    // Facing west
    else if (CurrentDirection() == WEST)
    {
      // Deadend: Left
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor][y_cor - 1] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor][y_cor + 1])
      {
        if (pot_north <= pot_south)
        {
          rotate_right();
          forward();
        }
        else if (pot_north > pot_south)
        {
          rotate_left();
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
      }
    }
  }
  // ------------------
  // right way BLOCKED
  // ------------------
  else if (way_right == 1)
  {
    // Facing North
    if (CurrentDirection() == NORTH)
    {
      // Deadend: Left
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_north)
        {
          rotate_left();
          forward();
        }
        else if (pot_west >= pot_north)
        {
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
      {
        // rotate_right();
        forward();
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
      }
    }
    // Facing East
    else if (CurrentDirection() == EAST)
    {
      // Deadend: Left
      if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] == trail[x_cor + 1][y_cor])
      {
        if (pot_north < pot_east)
        {
          rotate_left();
          forward();
        }
        else if (pot_north >= pot_east)
        {
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        forward();
      }
    }

    // Facing South
    else if (CurrentDirection() == SOUTH)
    {
      // Deadend: Left
      if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor + 1][y_cor])
      {
        if (pot_east < pot_south)
        {
          rotate_left();
          forward();
        }
        else if (pot_east >= pot_south)
        {
          forward();
        }
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the leading cell before?
      {
        forward();
      }
    }

    // Facing West
    else if (CurrentDirection() == WEST)
    {
      // Deadend: Left
      // Deadend: Left
      if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor - 1][y_cor])
      {
        if (pot_south < pot_west)
        {
          rotate_left();
          forward();
        }
        else if (pot_south >= pot_west)
        {
          forward();
        }
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
      {
        forward();
      }
    }
  }
}
void three_way()
{
  // This subroutine is triggered when there are 3 possible ways open.

  // Access the array elements if within bounds
  pot_north = (y_cor + 1 < ROWS) ? pot_field[x_cor][y_cor + 1] : Maze_Border;
  pot_east = (x_cor + 1 < COLS) ? pot_field[x_cor + 1][y_cor] : Maze_Border;
  pot_south = (y_cor - 1 >= 0) ? pot_field[x_cor][y_cor - 1] : Maze_Border;
  pot_west = (x_cor - 1 >= 0) ? pot_field[x_cor - 1][y_cor] : Maze_Border;

  trail_north = (y_cor + 1 < ROWS) ? trail[x_cor][y_cor + 1] : Maze_Border;
  trail_east = (x_cor + 1 < COLS) ? trail[x_cor + 1][y_cor] : Maze_Border;
  trail_south = (y_cor - 1 >= 0) ? trail[x_cor][y_cor - 1] : Maze_Border;
  trail_west = (x_cor - 1 >= 0) ? trail[x_cor - 1][y_cor] : Maze_Border;

  if (CurrentDirection() == NORTH)
  {
    if (trail_east < trail_north && trail_east < trail_west)
    {
      rotate_right();
      forward();
    }
    else if ((trail_north < trail_east && trail_north < trail_west))
    {
      forward();
    }
    else if (trail_west < trail_east && trail_west < trail_north)
    {
      rotate_left();
      forward();
    }
    else if (trail_east == trail_north && trail_east == trail_west)
    {
      if (pot_east < pot_north && pot_east < pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north < pot_east && pot_north < pot_west)
      {
        forward();
      }
      else if (pot_west < pot_east && pot_west < pot_north)
      {
        rotate_left();
        forward();
      }
      else if (pot_east == pot_north && pot_east == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north || pot_east == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north == pot_west)
      {
        forward();
      }
    }
    else if (trail_east == trail_north)
    {
      if (pot_north < pot_east)
      {
        forward();
      }
      else if (pot_north >= pot_east)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_east == trail_west)
    {
      if (pot_west < pot_east)
      {
        rotate_left();
        forward();
      }
      else if (pot_west >= pot_east)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_north == trail_west)
    {
      if (pot_north <= pot_west)
      {
        forward();
      }
      else if (pot_north > pot_west)
      {
        rotate_left();
        forward();
      }
    }
  }

  else if (CurrentDirection() == EAST)
  {
    if (trail_east < trail_north && trail_east < trail_south)
    {
      forward();
    }
    else if (trail_north < trail_east && trail_north < trail_south)
    {
      rotate_left();
      forward();
    }
    else if (trail_south < trail_east && trail_south < trail_north)
    {
      rotate_right();
      forward();
    }
    else if (trail_east == trail_north && trail_east == trail_south)
    {
      if (pot_east < pot_north && pot_east < pot_south)
      {
        forward();
      }
      else if (pot_north < pot_east && pot_north < pot_south)
      {
        rotate_left();
        forward();
      }
      else if (pot_south < pot_east && pot_south < pot_north)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north && pot_east == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north || pot_east == pot_south)
      {
        forward();
      }
      else if (pot_north == pot_south)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_east == trail_north)
    {
      if (pot_north < pot_east)
      {
        rotate_left();
        forward();
      }
      else if (pot_north >= pot_east)
      {
        forward();
      }
    }
    else if (trail_east == trail_south)
    {
      if (pot_south < pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_south >= pot_east)
      {
        forward();
      }
    }
    else if (trail_south == trail_north)
    {
      if (pot_north < pot_south)
      {
        rotate_left();
        forward();
      }
      else if (pot_north >= pot_south)
      {
        rotate_right();
        forward();
      }
    }
  }
  else if (CurrentDirection() == SOUTH)
  {
    if (trail_east < trail_south && trail_east < trail_west)
    {
      rotate_left();
      forward();
    }
    else if (trail_south < trail_east && trail_south < trail_west)
    {
      forward();
    }
    else if (trail_west < trail_east && trail_west < trail_south)
    {
      rotate_right();
      forward();
    }
    else if (trail_east == trail_south && trail_east == trail_west)
    {
      if (pot_east < pot_south && pot_east < pot_west)
      {
        rotate_left();
        forward();
      }
      else if (pot_south < pot_east && pot_south < pot_west)
      {
        forward();
      }
      else if (pot_west < pot_east && pot_west < pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_south && pot_south == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_south == pot_west || pot_west == pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_south)
      {
        forward();
      }
    }

    else if (trail_east == trail_south)
    {
      if (pot_south <= pot_east)
      {
        forward();
      }
      else if (pot_south > pot_east)
      {
        rotate_left();
        forward();
      }
    }
    else if (trail_east == trail_west)
    {
      if (pot_west <= pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_west > pot_east)
      {
        rotate_left();
        forward();
      }
    }
    else if (trail_south == trail_west)
    {
      if (pot_south < pot_west)
      {
        forward();
      }
      else if (pot_south >= pot_west)
      {
        rotate_right();
        forward();
      }
    }
  }
  else if (CurrentDirection() == WEST)
  {
    if (trail_west < trail_north && trail_west < trail_south)
    {
      forward();
    }
    else if ((trail_north < trail_west && trail_north < trail_west))
    {
      rotate_right();
      forward();
    }
    else if (trail_south < trail_west && trail_south < trail_north)
    {
      rotate_left();
      forward();
    }
    else if (trail_west == trail_north && trail_south == trail_west)
    {
      if (pot_west < pot_north && pot_west < pot_south)
      {
        forward();
      }
      else if (pot_north < pot_south && pot_north < pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_south < pot_west && pot_south < pot_north)
      {
        rotate_left();
        forward();
      }
      else if (pot_west == pot_north && pot_west == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_west == pot_north || pot_north == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_north == pot_west)
      {
        forward();
      }
    }

    else if (trail_west == trail_north)
    {
      if (pot_north <= pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north > pot_west)
      {
        forward();
      }
    }
    else if (trail_west == trail_south)
    {
      if (pot_south < pot_west)
      {
        rotate_left();
        forward();
      }
      else if (pot_south >= pot_west)
      {
        forward();
      }
    }
    else if (trail_north == trail_west)
    {
      if (pot_north <= pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north > pot_west)
      {
        forward();
      }
    }
  }
}
// Handle dead ends and find a way out
void decisions()
{
  // Decision-making is a quite important subroutine. With information from the wall below,
  // the algorithm now can make decisions on whether to move forward, rotate, or make a turn.
  decision = way_left + way_front + way_right;
  if (decision == 0)
  {
    // When there are 3 possible ways open, three_way subroutine will be triggered.
    // It'll never happen in any 5x5 maze. So this three_way subroutine won't
    // needs to be developed.
    three_way();
  }
  else if (decision == 1)
  {
    // When there are more than one way to move, it'll execute two_way subroutine.
    two_way();
  }
  else if (decision == 2)
  {
    // When there is one way to move, it'll execute oneway subroutine.
    oneway();
  }
  else if (decision == 3)
  {
    // When there is no way to move, it'll execute dead_end subroutine.
    dead_end();
  }
}
void firstPath()
{

  wallcheck();
  decision = way_left + way_right + way_front;
  while (decision == 2)
  {
    deadend[x_cor][y_cor] = 1;
    trail[x_cor][y_cor] = 200;
    // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    oneway();
    Close_Cell_After_Deadend();

    wallcheck();
    decision = way_left + way_right + way_front;
  }
  FirtPath = 1;
}
void Maze_Navigate()
{
  // This function is responsible for navigating the maze.
  // Check if the maze solving is finished.

  finishcheck();

  if (!FirtPath)
    firstPath();
  // Check for walls and update maze information.
  wallcheck();
  // Make navigation decisions based on the current state.
  decisions();
  // Perform the next move based on the decisions.
}
void Maze_Run()
{
  // Find the lowest path through the maze.
  // Find_Lowest_Path();
  wallcheck();
}
