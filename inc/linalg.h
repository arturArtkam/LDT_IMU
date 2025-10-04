#ifndef LINALG_H_INCLUDED
#define LINALG_H_INCLUDED

#include <math.h> // Для sqrtf

// --- Структуры ---
struct Vec {
    float X, Y, Z;
};

struct Matrix_3_3 {
    float XX, XY, XZ;
    float YX, YY, YZ;
    float ZX, ZY, ZZ;
};

// --- Функции для векторов ---
/* Вычисляет скалярное произведение двух векторов */
float sclr_mltp(struct Vec a, struct Vec b);
/* Вычисляет сумму двух векторов */
struct Vec vctr_summ(struct Vec a, struct Vec b);
/* Вычисляет разность двух векторов */
struct Vec vctr_diff(struct Vec a, struct Vec b);
/* Вычисляет векторное произведение двух векторов */
struct Vec vctr_mltp(struct Vec a,  struct Vec b);
/* Умножает вектор на число (скаляр) */
struct Vec vctr_mltp_n(float a, struct Vec b);
/* Находит длину (модуль) вектора */
float modul(struct Vec a);
/* Находит косинус угла между двумя векторами */
float vctr_cos(struct Vec a, struct Vec b);

// --- Функции для матриц ---
/*  Умножение матрицы на вектор*/
struct Vec mtrx_vctr_mltp(struct Matrix_3_3 m, struct Vec v);
/* Нахождение обратной матрицы */
struct Matrix_3_3 mtrx_1(struct Matrix_3_3 m);
/* Умножение двух матриц */
struct Matrix_3_3 mtrx_mltp(struct Matrix_3_3 m, struct Matrix_3_3 m1);
/* "Деление" матриц. Операция реализована как умножение первой матрицы на матрицу, обратную второй.*/
struct Matrix_3_3 mtrx_dv(struct Matrix_3_3 m, struct Matrix_3_3 m1);


#endif /* LINALG_H_INCLUDED */
