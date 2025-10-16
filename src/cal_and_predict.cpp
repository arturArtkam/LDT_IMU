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

    S_x[0] = N * S_x[4] * S_x[2] + S_x[3] * S_x[2] * S_x[1] + S_x[3] * S_x[2] * S_x[1] - S_x[2] * S_x[2] * S_x[2] - S_x[4] * S_x[1] * S_x[1] - N * S_x[3] * S_x[3];
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

    abc[0] = Delta_1 / S_x[0];
    abc[1] = Delta_2 / S_x[0];
    abc[2] = Delta_3 / S_x[0];

    float prediction = ((N)*(N)*abc[0] + (N)* abc[1] + abc[2]);
    return prediction;
}

/**
 * @brief Аппроксимирует историю компоненты (sin или cos) полиномом 2-й степени
 *        и предсказывает ее значение в будущем.
 * @param history_buffer Указатель на начало буфера с историей значений (например, sin_mtf_buffer).
 * @param buffer_size    Общий размер буфера (N).
 * @param K              Размер окна для аппроксимации.
 * @param lag_samples    Количество отсчетов, на которое нужно предсказать вперед для компенсации задержки.
 * @param s_x            Указатель на предварительно рассчитанный массив сумм степеней для окна размера K.
 *                       Формат: [main_det, sum_x, sum_x2, sum_x3, sum_x4].
 * @return               Предсказанное значение компоненты.
 */
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples, const float* s_x)
{
    // --- 1. Проверки на корректность ---
    if (K <= 2) {
        // Недостаточно точек для аппроксимации параболой, возвращаем последнее известное значение
        return history_buffer[buffer_size - 1];
    }

    const float main_det = s_x[0];
    if (std::fabs(main_det) < 1e-9f) { // Защита от деления на ноль
        return history_buffer[buffer_size - 1];
    }

    // --- 2. Определяем окно данных для анализа ---
    // Нам нужны последние K значений из буфера истории
    const float* data_window = history_buffer + (buffer_size - K);

    // --- 3. Рассчитываем суммы, зависящие от Y ---
    float S_y = 0.0f;
    float S_xy = 0.0f;
    float S_x2y = 0.0f;

    for (int i = 0; i < K; ++i) {
        S_y   += data_window[i];
        S_xy  += static_cast<float>(i) * data_window[i];
        S_x2y += static_cast<float>(i) * static_cast<float>(i) * data_window[i];
    }

    // --- 4. Рассчитываем коэффициенты a, b, c по формулам Крамера (из вашего оригинального кода) ---
    // Формулы находят коэффициенты для параболы: y = a*x^2 + b*x + c

    const float s_x1 = s_x[1];
    const float s_x2 = s_x[2];
    const float s_x3 = s_x[3];
    const float s_x4 = s_x[4];

    // Определитель для 'c' (свободный член)
    float delta_c = (s_x4 * s_x2 * S_y    + s_x3 * s_x1 * S_x2y + s_x2 * s_x3 * S_xy -
                     s_x2 * s_x2 * S_x2y  - s_x1 * s_x4 * S_xy  - s_x3 * s_x3 * S_y);

    // Определитель для 'b' (линейный член)
    float delta_b = (s_x4 * K * S_xy      + s_x2 * s_x1 * S_x2y + s_x3 * S_y * s_x2 -
                     s_x2 * s_x2 * S_xy   - s_x1 * K * S_x2y    - s_x3 * S_x2y * K);

    // Определитель для 'a' (квадратичный член)
    float delta_a = (K * s_x2 * S_x2y     + s_x1 * S_xy * s_x2  + s_x1 * S_y * s_x3 -
                     s_x2 * s_x2 * S_y    - s_x1 * s_x1 * S_x2y - K * s_x3 * S_xy);

    float c = delta_c / main_det;
    float b = delta_b / main_det;
    float a = delta_a / main_det;

    // --- 5. Делаем предсказание в нужной точке ---
    float predict_at_point = static_cast<float>(K) + lag_samples;
    float prediction = a * predict_at_point * predict_at_point + b * predict_at_point + c;

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
