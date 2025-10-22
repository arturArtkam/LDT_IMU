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
    // ������� ������� � 3 ������� ��������� ������� � ��������� �����. ��� ��������
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
 * @brief �������������� ������� ���������� (sin ��� cos) ��������� 2-� �������
 *        � ������������� �� �������� � �������.
 * @param history_buffer ��������� �� ������ ������ � �������� �������� (��������, sin_mtf_buffer).
 * @param buffer_size    ����� ������ ������ (N).
 * @param K              ������ ���� ��� �������������.
 * @param lag_samples    ���������� ��������, �� ������� ����� ����������� ������ ��� ����������� ��������.
 * @param s_x            ��������� �� �������������� ������������ ������ ���� �������� ��� ���� ������� K.
 *                       ������: [main_det, sum_x, sum_x2, sum_x3, sum_x4].
 * @return               ������������� �������� ����������.
 */
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples, const float* s_x)
{
    // --- 1. �������� �� ������������ ---
    if (K <= 2) {
        // ������������ ����� ��� ������������� ���������, ���������� ��������� ��������� ��������
        return history_buffer[buffer_size - 1];
    }

    const float main_det = s_x[0];
    if (std::fabs(main_det) < 1e-9f) { // ������ �� ������� �� ����
        return history_buffer[buffer_size - 1];
    }

    // --- 2. ���������� ���� ������ ��� ������� ---
    // ��� ����� ��������� K �������� �� ������ �������
    const float* data_window = history_buffer + (buffer_size - K);

    // --- 3. ������������ �����, ��������� �� Y ---
    float S_y = 0.0f;
    float S_xy = 0.0f;
    float S_x2y = 0.0f;

    for (int i = 0; i < K; ++i) {
        S_y   += data_window[i];
        S_xy  += static_cast<float>(i) * data_window[i];
        S_x2y += static_cast<float>(i) * static_cast<float>(i) * data_window[i];
    }

    // --- 4. ������������ ������������ a, b, c �� �������� ������� (�� ������ ������������� ����) ---
    // ������� ������� ������������ ��� ��������: y = a*x^2 + b*x + c

    const float s_x1 = s_x[1];
    const float s_x2 = s_x[2];
    const float s_x3 = s_x[3];
    const float s_x4 = s_x[4];

    // ������������ ��� 'c' (��������� ����)
    float delta_c = (s_x4 * s_x2 * S_y    + s_x3 * s_x1 * S_x2y + s_x2 * s_x3 * S_xy -
                     s_x2 * s_x2 * S_x2y  - s_x1 * s_x4 * S_xy  - s_x3 * s_x3 * S_y);

    // ������������ ��� 'b' (�������� ����)
    float delta_b = (s_x4 * K * S_xy      + s_x2 * s_x1 * S_x2y + s_x3 * S_y * s_x2 -
                     s_x2 * s_x2 * S_xy   - s_x1 * K * S_x2y    - s_x3 * S_x2y * K);

    // ������������ ��� 'a' (������������ ����)
    float delta_a = (K * s_x2 * S_x2y     + s_x1 * S_xy * s_x2  + s_x1 * S_y * s_x3 -
                     s_x2 * s_x2 * S_y    - s_x1 * s_x1 * S_x2y - K * s_x3 * S_xy);

    float c = delta_c / main_det;
    float b = delta_b / main_det;
    float a = delta_a / main_det;

    // --- 5. ������ ������������ � ������ ����� ---
    float predict_at_point = static_cast<float>(K) + lag_samples;
    float prediction = a * predict_at_point * predict_at_point + b * predict_at_point + c;

    return prediction;
}

/**
 * @brief ������������� � �������� ���������� "������" ���������� ��� ��� �������� 2-� �������.
 *
 * ��� ������� �������� ������ ��������� ���������� Python-������
 * predict_component_coordinate_independent. ��� �������������� �������
 * ���������� (sin ��� cos) � ������������� �� �������� � �������,
 * ����������� ��������. �������� ����� ������ ������� ���������,
 * ��� ������ �� ����������� �� ���������� �������� X.
 *
 * @param history_buffer ��������� �� ������ ������ � �������� �������� (��������, sin_mtf_buffer).
 * @param buffer_size    ����� ������ ������ (N).
 * @param K              ������ ���� ��� ������������� (���������� ��������� �����).
 * @param lag_samples    ���������� ��������, �� ������� ����� ����������� ������.
 * @return               ������������� �������� ����������.
 */
float predict_component(const float* history_buffer, int buffer_size, int K, float lag_samples)
{
    // --- 1. �������� �� ������������ ---
    if (K <= 2) {
        // ������������ ����� ��� ������������� ���������, ���������� ��������� ��������� ��������
        return history_buffer[buffer_size - 1];
    }

    // --- 2. ���������� ���� ������ ��� ������� � �������� ������� ��������� ---
    // ��� ����� ��������� K �������� �� ������ �������
    const float* y = history_buffer + (buffer_size - K);
    // ������� �������� ��� x=[0, 1, ..., K-1] ������ ����� (K-1)/2.
    // ��� ��� ����� ��� ������������� ������.
    const float x_mean = static_cast<float>(K - 1) / 2.0f;

    // --- 3. ������������ ����������� ����� � ����� (���������) ������� ��������� ---
    // ��� 5 ���� ����������� �� ���� ������ �� ������.
    float sx2_prime = 0.0f;
    float sx4_prime = 0.0f;
    float sy = 0.0f;
    float sx_prime_y = 0.0f;
    float sx2_prime_y = 0.0f;

    for (int i = 0; i < K; ++i)
    {
        // x_prime - ��� ������������� ���������� x, ��������� � ������
        const float x_prime_i = static_cast<float>(i) - x_mean;
        const float y_i = y[i];
        const float x_prime_i_sq = x_prime_i * x_prime_i;

        sx2_prime += x_prime_i_sq;
        sx4_prime += x_prime_i_sq * x_prime_i_sq; // (x_prime^2)^2 = x_prime^4
        sy += y_i;
        sx_prime_y += x_prime_i * y_i;
        sx2_prime_y += x_prime_i_sq * y_i;
    }

    // --- 4. ������ ���������� ������� ��������� ��� ������������� a', b', c' ---
    // ����������� b' (�������� ���� � ��������� �������) ��������� �����
    if (std::fabs(sx2_prime) < 1e-9f)  // ������ �� ������� �� ����
    {
        return history_buffer[buffer_size - 1];
    }
    const float b_prime = sx_prime_y / sx2_prime;

    // ��� a' � c' ������ ������� 2x2. ������� ������� �� ������������.
    const float det = static_cast<float>(K) * sx4_prime - sx2_prime * sx2_prime;
    if (std::fabs(det) < 1e-9f)
    {
        return history_buffer[buffer_size - 1];
    }

    // ������� c' (��������� ����) � a' (������������ ����) �� ������� �������
    const float c_prime = (sx4_prime * sy - sx2_prime * sx2_prime_y) / det;
    const float a_prime = (static_cast<float>(K) * sx2_prime_y - sx2_prime * sy) / det;

    // --- 5. ������ ������������ ---
    // ������� ����� � ������, ���������� ������� ���������
    const float predict_at_point_x = static_cast<float>(K) + lag_samples;

    // ��������� �� � �����, ������������� ������� ���������
    const float predict_at_point_x_prime = predict_at_point_x - x_mean;

    // ��������� ������������ �� ��������� �������� � ��������� �������
    const float prediction = a_prime * predict_at_point_x_prime * predict_at_point_x_prime +
                           b_prime * predict_at_point_x_prime +
                           c_prime;

    return prediction;
}

/**
 * @brief ������������� � �������� ���������� "������" ���������� ���,
 *        �������������� ��� ������ � ��������� �������.
 *
 * @param circ_history_buffer ��������� �� ������ ���������� ������.
 * @param buffer_size         ����� ������ ������ (N).
 * @param head_idx            ������ "������" ������ (���� ���� �������� ��������� ��������).
 * @param K                   ������ ���� ��� �������������.
 * @param lag_samples         ���������� �������� ��� ������������ ������.
 * @return                    ������������� �������� ����������.
 */
float predict_component_circ(const float* circ_history_buffer, int buffer_size, int head_idx, int K, float lag_samples)
{
    // --- 1. �������� �� ������������ ---
    if (K <= 2) {
        return circ_history_buffer[head_idx];
    }

    // --- 2. ������������ �����, "�������" ������ �� ���������� ������ ---
    // ��� �������� ������� �� ���������� ������.
    const float x_mean = static_cast<float>(K - 1) / 2.0f;

    float sx2_prime = 0.0, sx4_prime = 0.0, sy = 0.0, sx_prime_y = 0.0, sx2_prime_y = 0.0;

    // ����������� K ���, ����� �������� K ��������� ��������
    for (int i = 0; i < K; ++i) {
        // ��������� ������ � ��������� ������, �������� "�����" �� ������
        // (head_idx - (K - 1) + i) - ��� "�����������" ������ �� ������ � ������
        // �������� % buffer_size "������������" ������������� �������
        int current_idx = (head_idx - (K - 1) + i + buffer_size) % buffer_size;

        const float y_i = circ_history_buffer[current_idx];

        // x_prime - ��� ������������� ���������� x, ��� i - ����� ����� � ���� [0..K-1]
        const float x_prime_i = static_cast<float>(i) - x_mean;
        const float x_prime_i_sq = x_prime_i * x_prime_i;

        sx2_prime += x_prime_i_sq;
        sx4_prime += x_prime_i_sq * x_prime_i_sq;
        sy += y_i;
        sx_prime_y += x_prime_i * y_i;
        sx2_prime_y += x_prime_i_sq * y_i;
    }

    // --- 3. ������ ������� ��������� (���� ���� ���� �������� ��� ���������) ---
    if (std::fabs(sx2_prime) < 1e-9) return circ_history_buffer[head_idx];
    const float b_prime = sx_prime_y / sx2_prime;

    const float det = static_cast<float>(K) * sx4_prime - sx2_prime * sx2_prime;
    if (std::fabs(det) < 1e-9) return circ_history_buffer[head_idx];

    const float c_prime = (sx4_prime * sy - sx2_prime * sx2_prime_y) / det;
    const float a_prime = (static_cast<float>(K) * sx2_prime_y - sx2_prime * sy) / det;

    // --- 4. ������ ������������ (���� ���� ���� �������� ��� ���������) ---
    const float predict_at_point_x = static_cast<float>(K) + lag_samples;
    const float predict_at_point_x_prime = predict_at_point_x - x_mean;

    const float prediction = a_prime * predict_at_point_x_prime * predict_at_point_x_prime +
                           b_prime * predict_at_point_x_prime +
                           c_prime;

    return static_cast<float>(prediction);
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
