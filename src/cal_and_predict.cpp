#include <math.h>
#include <tgmath.h>
#include <complex.h>

#include "cal_and_predict.h"

void S_X(float S_x[5], int N)
{
    for (int i = 0; i < 5; i++)
    {
        S_x[i] = 0;
    }
    for (int x = 0; x < N; x++)
    {
        S_x[1] += x;
        S_x[2] += pow(x, 2);
        S_x[3] += pow(x, 3);
        S_x[4] += pow(x, 4);
    }

    S_x[0] = N * S_x[4] * S_x[2] + S_x[3] * S_x[2] * S_x[1] + S_x[3]* S_x[2] * S_x[1] - S_x[2] * S_x[2] * S_x[2] - S_x[4] * S_x[1] * S_x[1] - N * S_x[3] *S_x[3];
}

float predict(float *abc, float *data, float S_x[5], int N)
{
    float S_y = 0, S_xy = 0, S_x2y = 0;//
    float Delta_1 = 0, Delta_2 = 0, Delta_3 = 0;//
    abc[0] = 0; abc[1] = 0; abc[2] = 0;

    for (int x = 0; x < N; x++) {
        S_y += data[x];
        S_xy += x * data[x];
        S_x2y += x * x * data[x];
    }
    // считаем главный и 3 частных оператора матрицы и вычисляем коэфф. при полиноме
    Delta_1 = N * S_x2y * S_x[2] + S_xy * S_x[1] * S_x[2] + S_y * S_x[3] * S_x[1] - S_y * S_x[2] * S_x[2] - S_x[1] * S_x[1] * S_x2y - N * S_x[3] * S_xy;
    Delta_2 = N * S_x[4] * S_xy + S_x[3] * S_x[2] * S_y + S_x[2] * S_x[1] * S_x2y - S_x[2] * S_x[2] * S_xy - S_x[4] * S_y * S_x[1] - N * S_x[3] * S_x2y;
    Delta_3 = S_x[4] * S_x[2] * S_y + S_x[3] * S_x[1] * S_x2y + S_x[3] * S_x[2] * S_xy - S_x[2] * S_x[2] * S_x2y - S_x[4] * S_x[1] * S_xy - S_x[3] * S_x[3] * S_y;

    abc[0] = Delta_1 / S_x[0]; abc[1] = Delta_2 / S_x[0]; abc[2] = Delta_3 / S_x[0];

    float prediction = ((N)*(N)*abc[0] + (N)* abc[1] + abc[2]);
    return prediction;
}

//struct Cal add_cal(struct Cal first, struct Cal second) {
//    struct Cal result;
//    result.offset = vctr_summ(first.offset , (mtrx_vctr_mltp(mtrx_1(first.sens), second.offset)));
//    result.sens = mtrx_mltp(second.sens, first.sens);
//    return result;
//}

struct Vec callibrate(struct Vec Data , struct Cal calib)
{
return  mtrx_vctr_mltp(calib.sens, (vctr_diff(Data, calib.offset)));
}
