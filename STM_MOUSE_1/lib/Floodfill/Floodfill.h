

#ifndef FloodFill
#define FloodFill

#include <stdint.h>
#include <stdlib.h>
void Cells_init_FloodFill(void);
void finishcheck(void);
void setVerticalWall(uint8_t x, uint8_t y);
void setHorizontalWall(uint8_t x, uint8_t y);
void DetermineWallTypes(uint8_t x, uint8_t y);
void wallcheck(void);
void array_null(void);
void Close_Cell_After_Deadend(void);
void oneway(void);
void half_turn(void);
void dead_end(void);
void Go_Back(void);
void two_way();
void three_way();
void decisions();
void firstPath();
void Maze_Navigate();
void Maze_Run();

#endif // FloodFill