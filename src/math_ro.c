
#include "inc_defs.h"
#include "math_ro.h"
#include <math.h>
#include <tgmath.h>
#include <complex.h>
#include "uart.h"

// прямая задача
float SIGNAL(struct METROLOGY *metrology, uint16_t n, uint16_t freq, float ro) {
    float L1 = metrology->L1[n] / 1000.0;
    float L2 = metrology->L2[n] / 1000.0;
    float omegamu0sigma = (0.0078957 * metrology->F[freq]) / ro;
    complex float ik = I * csqrtf(I * omegamu0sigma);
    complex float SGN = cexpf(ik*(L2 - L1)) * ((1.0 - ik * L2) / (1.0 - ik * L1));
    return cargf(SGN);
}

//обратная задача МПП "золотое сечение"
float RO_ARG(struct METROLOGY *metrology, float PH,  uint16_t n, uint16_t freq) {
    float Ro0;
    float epsilon_ARG = 0.0000005; // точность  фазы
    float epsilon_INF = 0.000000005; // точность  фазы
    float  Ro_0 = 0.01, Ro_max = 7000.0;  //мин. и мах. значение ус для расчета
    bool stop_epsl_ARG = false;
    bool stop_epsl_INF = false;
    float  ro_0 = Ro_0;
    float  ro_max = Ro_max;

    do {
        float X1 = ro_0 + 0.382*(ro_max - ro_0);
        float X2 = ro_max - 0.382*(ro_max - ro_0);
        float A = PH - SIGNAL(metrology, n, freq, X1);//
        float B = PH - SIGNAL(metrology, n, freq, X2);//
        if (fabs(A) > fabs(B)) { ro_0 = X1; }
        else { ro_max = X2; }
        if (fabs(A) < epsilon_ARG || fabs(B) < epsilon_ARG) { stop_epsl_ARG = true; }
        else if (fabs(A - B) < epsilon_INF) { stop_epsl_INF = true; }

    } while (stop_epsl_ARG == false && stop_epsl_INF == false);
    if (stop_epsl_ARG == true) Ro0 = (ro_0 + ro_max) / 2;
    if (stop_epsl_INF == true) Ro0 = 7200;//-100
    ro_0 = Ro_0;
    ro_max = Ro_max;
    return  Ro0;

}

//определяет, как симметризировать, в зависимости от числа работающих передатчиков
uint16_t formula_simmetry(float K[4][4], uint16_t condition) {
    uint16_t result;
    //condition = condition >> 4;
    if (condition == 0b00001111) {// работают все 4 передатчика
        K[T1][T1] = +0.75;  K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00;
        K[T2][T1] = +0.25;  K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00;
        K[T3][T1] = +0.00;  K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25;
        K[T4][T1] = +0.00;  K[T4][T2] = -0.25; K[T4][T3] = +0.50; K[T4][T4] = +0.75;
        result = 0b00001111;
    }
    else if (condition == 0b00000111) {// работают 2,3,4 передатчики
        K[T1][T1] = +0.00; K[T1][T2] = +1.25; K[T1][T3] = +0.50; K[T1][T4] = -0.75;
        K[T2][T1] = +0.00; K[T2][T2] = +0.75; K[T2][T3] = +0.50; K[T2][T4] = -0.25;
        K[T3][T1] = +0.00; K[T3][T2] = +0.25; K[T3][T3] = +0.50; K[T3][T4] = +0.25;
        K[T4][T1] = +0.00; K[T4][T2] = -0.25; K[T4][T3] = +0.50; K[T4][T4] = +0.75;
        result = 0b00000111;
    }
    else if (condition == 0b00001110) {// работают 1,2,3 передатчики
        K[T1][T1] = +0.75; K[T1][T2] = +0.50; K[T1][T3] = -0.25; K[T1][T4] = +0.00;
        K[T2][T1] = +0.25; K[T2][T2] = +0.50; K[T2][T3] = +0.25; K[T2][T4] = +0.00;
        K[T3][T1] = -0.25; K[T3][T2] = +0.50; K[T3][T3] = +0.75; K[T3][T4] = +0.00;
        K[T4][T1] = -0.75; K[T4][T2] = +0.50; K[T4][T3] = +1.25; K[T4][T4] = +0.00;
        result = 0b00001110;
    }
    else if (condition == 0b00001011) {// работают 1,3,4 передатчики
        K[T1][T1] = +1.25; K[T1][T2] = 0.00; K[T1][T3] = -0.75; K[T1][T4] = 0.50;
        K[T2][T1] = +0.75; K[T2][T2] = 0.00; K[T2][T3] = -0.25; K[T2][T4] = 0.50;
        K[T3][T1] = +0.25; K[T3][T2] = 0.00; K[T3][T3] = +0.25; K[T3][T4] = 0.50;
        K[T4][T1] = -0.25; K[T4][T2] = 0.00; K[T4][T3] = +0.75; K[T4][T4] = 0.50;
        result = 0b00001011;
    }
    else if (condition == 0b00001101) {// работают 1,2,4 передатчики
        K[T1][T1] = +0.50; K[T1][T2] = +0.75; K[T1][T3] = +0.00; K[T1][T4] = -0.25;
        K[T2][T1] = +0.50; K[T2][T2] = +0.25; K[T2][T3] = +0.00; K[T2][T4] = +0.25;
        K[T3][T1] = +0.50; K[T3][T2] = -0.25; K[T3][T3] = +0.00; K[T3][T4] = +0.75;
        K[T4][T1] = -0.50; K[T4][T2] = -0.75; K[T4][T3] = +0.00; K[T4][T4] = +1.25;
        result = 0b00001101;
    }
    else if (condition == 0b00000000) {// выводим несимметризованные значения для всех передатчиков
        K[T1][T1] = 1.00; K[T1][T2] = 0.00; K[T1][T3] = 0.00; K[T1][T4] = 0.00;
        K[T2][T1] = 0.00; K[T2][T2] = 1.00; K[T2][T3] = 0.00; K[T2][T4] = 0.00;
        K[T3][T1] = 0.00; K[T3][T2] = 0.00; K[T3][T3] = 1.00; K[T3][T4] = 0.00;
        K[T4][T1] = 0.00; K[T4][T2] = 0.00; K[T4][T3] = 0.00; K[T4][T4] = 1.00;
        result = 0b00000000;
    }

    else {// работает меньше трех передатчиков
        //находим рабочие передатчики и для них выводим несимметризованные значения, для нерабочих фаза равна 0
        bool k4 = (condition >> 0) & 1u;
        bool k3 = (condition >> 1) & 1u;
        bool k2 = (condition >> 2) & 1u;
        bool k1 = (condition >> 3) & 1u;
        K[T1][T1] = 1.00*k1; K[T1][T2] = 0.00; K[T1][T3] = 0.00; K[T1][T4] = 0.00;
        K[T2][T1] = 0.00; K[T2][T2] = 1.00*k2; K[T2][T3] = 0.00; K[T2][T4] = 0.00;
        K[T3][T1] = 0.00; K[T3][T2] = 0.00; K[T3][T3] = 1.00*k3; K[T3][T4] = 0.00;
        K[T4][T1] = 0.00; K[T4][T2] = 0.00; K[T4][T3] = 0.00; K[T4][T4] = 1.00*k4;
        result = 0b00000000;
    }
    return result;
}

// переворачивает фазы, и нормирует к воздуху

void ph_norm_air(float *d_PH, float *d_PH_NORM, struct METROLOGY *metro, uint16_t freq) {
    uint16_t freq_idx = 0;
    for (int i = 0; i < 4; i++) {
        while(d_PH[i] > PI) d_PH[i] -= 2*PI;
        while(d_PH[i] <= -PI) d_PH[i] += 2*PI;
    }
    if(freq == 0) freq_idx = 0;
    else if(freq == 1)freq_idx = 4;
    // нормализуем сигнал к воздуху
    d_PH_NORM[0] -= ((float)(metro->Air_zz[freq_idx + 0]))/57297.0; //
    d_PH_NORM[1]  = -d_PH[1] + ((float)(metro->Air_zz[freq_idx + 1]))/ 57297.0;//
    d_PH_NORM[2] -= ((float)(metro->Air_zz[freq_idx + 2]))/57297.0; //
    d_PH_NORM[3]  = -d_PH[3] + ((float)(metro->Air_zz[freq_idx + 3]))/ 57297.0;//
}


//симметризует по формуле
uint16_t simmetry(float *d_PH, float *d_PH_SMT, uint16_t Avaria) {
    //если работаем без проверки аварийности -принудительно выставляем 0иb00001111
    //uint16_t condition = Avaria;
    uint16_t condition = 0x000f;

    float K[4][4] = { 0.0, };
    uint16_t formula = formula_simmetry(K, condition);
    for (int i = 0; i < 4; i++) {
        d_PH_SMT[i] = K[i][0] * d_PH[0] + K[i][1] * d_PH[1] + K[i][2] * d_PH[2] + K[i][3] * d_PH[3];
    }
    return formula;
}


//--------------------------------------------------------------------------------------------------------

