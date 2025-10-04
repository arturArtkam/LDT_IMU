#ifndef LINALG_H_INCLUDED
#define LINALG_H_INCLUDED

#include <math.h> // ��� sqrtf

// --- ��������� ---
struct Vec {
    float X, Y, Z;
};

struct Matrix_3_3 {
    float XX, XY, XZ;
    float YX, YY, YZ;
    float ZX, ZY, ZZ;
};

// --- ������� ��� �������� ---
/* ��������� ��������� ������������ ���� �������� */
float sclr_mltp(struct Vec a, struct Vec b);
/* ��������� ����� ���� �������� */
struct Vec vctr_summ(struct Vec a, struct Vec b);
/* ��������� �������� ���� �������� */
struct Vec vctr_diff(struct Vec a, struct Vec b);
/* ��������� ��������� ������������ ���� �������� */
struct Vec vctr_mltp(struct Vec a,  struct Vec b);
/* �������� ������ �� ����� (������) */
struct Vec vctr_mltp_n(float a, struct Vec b);
/* ������� ����� (������) ������� */
float modul(struct Vec a);
/* ������� ������� ���� ����� ����� ��������� */
float vctr_cos(struct Vec a, struct Vec b);

// --- ������� ��� ������ ---
/*  ��������� ������� �� ������*/
struct Vec mtrx_vctr_mltp(struct Matrix_3_3 m, struct Vec v);
/* ���������� �������� ������� */
struct Matrix_3_3 mtrx_1(struct Matrix_3_3 m);
/* ��������� ���� ������ */
struct Matrix_3_3 mtrx_mltp(struct Matrix_3_3 m, struct Matrix_3_3 m1);
/* "�������" ������. �������� ����������� ��� ��������� ������ ������� �� �������, �������� ������.*/
struct Matrix_3_3 mtrx_dv(struct Matrix_3_3 m, struct Matrix_3_3 m1);


#endif /* LINALG_H_INCLUDED */
