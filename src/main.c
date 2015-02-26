/*
 * main.c
 *
 *  Created on: 2015��2��25��
 *      Author: �²�
 */

#include <stdio.h>
#include <stdlib.h>
#include "kalman.h"
float T_temp;
kalman1_state kal;

int main(void) {
    FILE *fp1;  //�����ļ���ָ�룬���ڴ򿪶�ȡ���ļ�
    char text[1024];//����һ���ַ������飬���ڴ洢��ȡ���ַ�
    fp1 = fopen("test.txt","r");//ֻ����ʽ���ļ�a.txt
    T_temp=1;
    KalmanFilterInit();
    kalman1_init(&kal,1,1);
    while(fgets(text,1024,fp1)!=NULL)//���ж�ȡfp1��ָ���ļ��е����ݵ�text��
    {
        T_temp=atof(text)+1;
       // T_temp=KalmanFilter(T_temp);
        T_temp=kalman1_filter(&kal,T_temp);
        sprintf(text,"%f",T_temp);
        puts(text);//�������Ļ
    }
    fclose(fp1);//�ر��ļ�a.txt���д򿪾�Ҫ�йر�
    return EXIT_SUCCESS;
}
