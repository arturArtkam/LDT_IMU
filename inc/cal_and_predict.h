#ifndef CAL_AND_PREDICT_H_INCLUDED
#define CAL_AND_PREDICT_H_INCLUDED

#include "linalg.h"

/* Вектор смещения offset и матрица чувствительности/поворота sens.*/
struct Cal
{
    struct Vec offset;
    struct Matrix_3_3 sens;
};

//3. Аппроксимация и прогнозирование
//Эти функции реализуют метод наименьших квадратов для нахождения полинома второй степени (параболы y = ax^2 + bx + c),
// который наилучшим образом описывает набор входных данных data.
//S_X: Предварительно вычисляет суммы степеней x (от 1 до 4) и определитель основной матрицы системы уравнений.
// Это делается для оптимизации, чтобы не пересчитывать одни и те же значения многократно.
//predict: Находит коэффициенты a, b, c для параболы, решая систему линейных уравнений по формулам Крамера.
// После нахождения коэффициентов функция делает прогноз значения y для следующей точки x = N.
//4. Калибровка
//Эти функции используются для калибровки данных, получаемых с сенсоров (например, акселерометра или гироскопа). Калибровка обычно включает в себя поправку на смещение (offset) и чувствительность/поворот (sensitivity).
//callibrate:
//add_cal: Объединяет две калибровки. Это может быть полезно, если у вас есть, например, заводская калибровка и вы хотите добавить к ней пользовательскую. Новая матрица чувствительности является произведением двух старых, а новый офсет вычисляется с учетом уже примененных преобразований.

/* Применяет калибровку к вектору "сырых" данных Data.
Сначала из данных вычитается смещение, а затем результат умножается на матрицу чувствительности.
Формула: Откалиброванные_данные = sens * (Сырые_данные - offset).*/
Vec callibrate(Vec Data, Cal calib);
float predict(float *abc, float *data, float S_x[5], int N);
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples, const float* s_x);
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples);
float predict_component_circ(const float* circ_history_buffer, int buffer_size, int head_idx, int K, float lag_samples);
void S_X(float S_x[5], int N);

#endif /* CAL_AND_PREDICT_H_INCLUDED */
