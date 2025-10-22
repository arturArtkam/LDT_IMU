#ifndef CAL_AND_PREDICT_H_INCLUDED
#define CAL_AND_PREDICT_H_INCLUDED

#include "linalg.h"

/* ������ �������� offset � ������� ����������������/�������� sens.*/
struct Cal
{
    struct Vec offset;
    struct Matrix_3_3 sens;
};

//3. ������������� � ���������������
//��� ������� ��������� ����� ���������� ��������� ��� ���������� �������� ������ ������� (�������� y = ax^2 + bx + c),
// ������� ��������� ������� ��������� ����� ������� ������ data.
//S_X: �������������� ��������� ����� �������� x (�� 1 �� 4) � ������������ �������� ������� ������� ���������.
// ��� �������� ��� �����������, ����� �� ������������� ���� � �� �� �������� �����������.
//predict: ������� ������������ a, b, c ��� ��������, ����� ������� �������� ��������� �� �������� �������.
// ����� ���������� ������������� ������� ������ ������� �������� y ��� ��������� ����� x = N.
//4. ����������
//��� ������� ������������ ��� ���������� ������, ���������� � �������� (��������, ������������� ��� ���������). ���������� ������ �������� � ���� �������� �� �������� (offset) � ����������������/������� (sensitivity).
//callibrate:
//add_cal: ���������� ��� ����������. ��� ����� ���� �������, ���� � ��� ����, ��������, ��������� ���������� � �� ������ �������� � ��� ����������������. ����� ������� ���������������� �������� ������������� ���� ������, � ����� ����� ����������� � ������ ��� ����������� ��������������.

/* ��������� ���������� � ������� "�����" ������ Data.
������� �� ������ ���������� ��������, � ����� ��������� ���������� �� ������� ����������������.
�������: ���������������_������ = sens * (�����_������ - offset).*/
Vec callibrate(Vec Data, Cal calib);
float predict(float *abc, float *data, float S_x[5], int N);
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples, const float* s_x);
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples);
float predict_component_circ(const float* circ_history_buffer, int buffer_size, int head_idx, int K, float lag_samples);
void S_X(float S_x[5], int N);

#endif /* CAL_AND_PREDICT_H_INCLUDED */
