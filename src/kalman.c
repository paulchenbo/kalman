/*
 * kalman.c
 *
 *  Created on: 2015年2月25日
 *      Author: 陈博
 */
#include <math.h>
#include "kalman.h"

float KalmanGain;//  卡尔曼增益
float EstimateCovariance;//估计协方差
float MeasureCovariance;//测量协方差
float EstimateValue;//估计值
void KalmanFilterInit(void)
{
    EstimateValue = 0;
    EstimateCovariance = 1;
    MeasureCovariance = 2;
}
float KalmanFilter(float Measure)
{
    //计算卡尔曼增益
    KalmanGain=EstimateCovariance*sqrt(1/(EstimateCovariance*EstimateCovariance+MeasureCovariance*MeasureCovariance));
    //计算本次滤波估计值
    EstimateValue=EstimateValue+KalmanGain * (Measure-EstimateValue);
    //更新估计协方差
    EstimateCovariance=sqrt(1-KalmanGain) * EstimateCovariance;
    //更新测量方差
    MeasureCovariance=sqrt(1-KalmanGain) * MeasureCovariance;
    //返回估计值
    return EstimateValue;
}
/*
 * @brief
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1;
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 *   state - Klaman filter structure
 *   init_x - initial x state value
 *   init_p - initial estimated error convariance
 * @outputs
 * @retval
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}

/*
 * @brief
 *   1 Dimension Kalman filter
 * @inputs
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs
 * @retval
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

