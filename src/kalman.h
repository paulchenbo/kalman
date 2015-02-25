/*
 * kalman.h
 *
 *  Created on: 2015Äê2ÔÂ25ÈÕ
 *      Author: ³Â²©
 */

#ifndef KALMAN_H_
#define KALMAN_H_

typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} kalman1_state;

void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);

float KalmanFilter(float Measure);
void KalmanFilterInit(void);
#endif /* KALMAN_H_ */
