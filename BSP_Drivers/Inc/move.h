//
// Created by Haoyee_yin on 2023/8/1.
//

#ifndef PROJECT_MOVE_H
#define PROJECT_MOVE_H

#include "main.h"


//×ø±êÊý¾Ý
//int Point_0[2]={0,0}, Point_1[2]={50,275}, Point_2[2]={200,125}, Point_3[2]={275,200},\
//    Point_4[2]={350,-25}, Point_5[2]={350,275}, Point_6[2]={275,50}, Point_7[2]={125,50},\
//    Point_8[2]={125,200},Point_9[2]={50,125},Point_10[2]={200,-25},Point_11[2]={200,275},\
//    Point_12[2]={350,275};


void position_move(uint8_t num,float* x_expect_t,float* y_expect_t);
void Laser_dotting_base(void);

void Hatch_on();
void Hatch_off();

void Light_on();
void Light_off();

void Openmv_Data_change(uint8_t openmv_rebuf1,uint8_t openmv_rebuf2,float dis_x,float dis_y,float* fire_x,float* fire_y);

#endif //PROJECT_MOVE_H
