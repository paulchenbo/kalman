/*
 * main.c
 *
 *  Created on: 2015年2月25日
 *      Author: 陈博
 */

#include <stdio.h>
#include <stdlib.h>
#include "kalman.h"
float T_temp;
kalman1_state kal;

int main(void) {
    FILE *fp1;  //定义文件流指针，用于打开读取的文件
    char text[1024];//定义一个字符串数组，用于存储读取的字符
    fp1 = fopen("test.txt","r");//只读方式打开文件a.txt
    T_temp=1;
    KalmanFilterInit();
    kalman1_init(&kal,1,1);
    while(fgets(text,1024,fp1)!=NULL)//逐行读取fp1所指向文件中的内容到text中
    {
        T_temp=atof(text)+1;
       // T_temp=KalmanFilter(T_temp);
        T_temp=kalman1_filter(&kal,T_temp);
        sprintf(text,"%f",T_temp);
        puts(text);//输出到屏幕
    }
    fclose(fp1);//关闭文件a.txt，有打开就要有关闭
    return EXIT_SUCCESS;
}
