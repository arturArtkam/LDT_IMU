#include "main.h"
#include "prj_logger.h"
#include <math.h>

constexpr size_t MA_BUFFER_SIZE = 32; //���� ����������� �������� ��� ������
constexpr size_t MLD_BUFFER_SIZE = 32; // ���� ���
constexpr size_t PREDICTION_BUFFER_SIZE = 128; // ������ ������ ��� ��������� ���������
constexpr uint32_t PULSE_WIDTH = 3; // ������ �������� � ��
constexpr uint32_t BETH_PULSE_DELAY = 32; // ����� ����� ����������

static_assert((MA_BUFFER_SIZE > 0) && ((MA_BUFFER_SIZE & (MA_BUFFER_SIZE - 1)) == 0),
              "MA_BUFFER_SIZE must be a power of two for bitwise operations to work.");
static_assert((MLD_BUFFER_SIZE > 0) && ((MLD_BUFFER_SIZE & (MLD_BUFFER_SIZE - 1)) == 0),
              "MLD_BUFFER_SIZE must be a power of two for bitwise operations to work.");
static_assert((PREDICTION_BUFFER_SIZE > 0) && ((PREDICTION_BUFFER_SIZE & (PREDICTION_BUFFER_SIZE - 1)) == 0),
              "PREDICTION_BUFFER_SIZE must be a power of two for bitwise operations to work.");

// --- ��������� ��� �������� ������������� ������������� ---
Cal G_offset_sens = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
Cal M_offset_sens = { 0, 0, 0, 0.157f, 0, 0, 0, 0.157f, 0, 0, 0, 0.157f };
Cal W_offset_sens = { 0, 0, 0, 0.001221726f, 0, 0, 0, 0.001221726f, 0, 0, 0, 0.001221726f }; //rad per digit

// --- ������, ����������� �� ������ �������� ---
struct CalculatedData
{
    // ��������������� � ��������������� �������
    Vec G_ma = {0.0f, 0.0f, 0.0f}, M_ma = {0.0f, 0.0f, 0.0f}, W_ma = {0.0f, 0.0f, 0.0f};
    Vec G = {0.0f, 0.0f, 0.0f}, M = {0.0f, 0.0f, 0.0f}, W = {0.0f, 0.0f, 0.0f};

    // ����������� ��������
    float G_modul = 0.0f;
    float M_modul = 0.0f;
    float W_g = 0.0f; // ������� �������� �� ��� Z
    float Wg_1000 = 0.0f; // ������� �������� � ���/��
    float Wg_offset = 0.0f; // �������� ���� ���������

    // ����������� ����
    float angle_aps = 0.0f;
    float angle_zen = 0.0f;
    float angle_azm = 0.0f;
    float angle_aps_m = 0.0f;
    float MTF = 0.0f;
    float G_M_angle = 0.0f; // ������� ����� ��������� � �������������� ������

    // ���� � ��������
    volatile float angle_zen_deg = 0.0f, angle_aps_deg = 0.0f, angle_azm_deg = 0.0f, angle_aps_m_deg = 0.0f, MTF_deg = 0.0f;
};
// --- ��������� ��������� ��������� ---
struct ApsAlgorithmState
{
    // ��������� ��������� ��������
    volatile bool is_moving = false;
    uint32_t no_mov_delay_tim = 0;

    // ��������� ��������� ���������������
    int32_t circ_idx = 0;
    float mtf_buffer[PREDICTION_BUFFER_SIZE] = {0.0f}; // ����� ��� ��������� ���������
    float sin_mtf_buffer[PREDICTION_BUFFER_SIZE] = {0.0f}; // ����� ��� ��������� ���������
    float cos_mtf_buffer[PREDICTION_BUFFER_SIZE] = {0.0f}; // ����� ��� ��������� ���������
    float prediction = 0.0f; // ��� �������������� ��������

    // ��������� ������� APS
    Aps_state mode = APS_READY; // ������� ����� (enum Aps_state)
    float start_APS = 0;
    float aps_point_arr[16] = {0.0f};
    uint16_t aps_idx = 0; // ������� ������� ������� ���������

    // ������� � �������� APS
    volatile uint32_t tim = 0;
    volatile uint32_t pulse = 0;
    volatile uint32_t beth_pulse_delay_tim = 0;
    volatile float angle_path_wg = 0; // ���������� ������� ����
    bool interrupt_by_angle_path = false;

    // --- ��������� ��������� ---
    float final_MTF_m_p = 0.0f; // ��������� �������� MTF (���������� + �������)
};

//���������
struct Set settings;
//static CalibrationData callibrations;
static CalculatedData metrics;
static ApsAlgorithmState aps_state;

//int delta; //����������� 1, ���� ������� ���������� �� ���������� ������� ����
static float aps_delta = M_PI / 360; // - 1 ������ (� ����� ��������� ������������ ������ ������)

// ��� �����, ����� ����������� �������� � ������������� ������ � ����������� ��������
static Vec G_Summa = {0, 0, 0};
static Vec M_Summa = {0, 0, 0};
static Vec W_Summa = {0, 0, 0};
static Vec G_Data[MA_BUFFER_SIZE];
static Vec M_Data[MA_BUFFER_SIZE];
static Vec W_Data[MA_BUFFER_SIZE];

//��� ��������� ��������� � ���
static float g_modul_buff[MLD_BUFFER_SIZE];
static float m_modul_buff[MLD_BUFFER_SIZE];
static float w_modul_buff[MLD_BUFFER_SIZE];
static float slo_modul_g;
static float slo_modul_m;
static float slo_modul_w;

static float K_predict = 0.5;
static uint16_t i_ma = 0;

void read_cal_and_settings()
{
//   FLASH_ReadCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
//   FLASH_ReadSet(&settings);
    G_fram::read_buf(&G_offset_sens, Fram_markup::G_OFFSET_ADDR, sizeof(G_offset_sens));
    G_fram::read_buf(&M_offset_sens, Fram_markup::M_OFFSET_ADDR, sizeof(M_offset_sens));
    G_fram::read_buf(&W_offset_sens, Fram_markup::W_OFFSET_ADDR, sizeof(W_offset_sens));
    G_fram::read_buf(&settings, Fram_markup::SETTINGS_ADDR, sizeof(settings));

    if (isnan(G_offset_sens.offset.X)) G_offset_sens.offset.X = 3909340;
    if (isnan(G_offset_sens.offset.Y)) G_offset_sens.offset.Y = 3912131;
    if (isnan(G_offset_sens.offset.Z)) G_offset_sens.offset.Z = 3944069;
    if (isnan(G_offset_sens.sens.XX)) G_offset_sens.sens.XX = 1;
    if (isnan(G_offset_sens.sens.YX)) G_offset_sens.sens.YX = 0;
    if (isnan(G_offset_sens.sens.ZX)) G_offset_sens.sens.ZX = 0;
    if (isnan(G_offset_sens.sens.XY)) G_offset_sens.sens.XY = 0;
    if (isnan(G_offset_sens.sens.YY)) G_offset_sens.sens.YY = 1;
    if (isnan(G_offset_sens.sens.ZY)) G_offset_sens.sens.ZY = 0;
    if (isnan(G_offset_sens.sens.XZ)) G_offset_sens.sens.XZ = 0;
    if (isnan(G_offset_sens.sens.YZ)) G_offset_sens.sens.YZ = 0;
    if (isnan(G_offset_sens.sens.ZZ)) G_offset_sens.sens.ZZ = 1;

    if (isnan(M_offset_sens.offset.X)) M_offset_sens.offset.X = -4910770;
    if (isnan(M_offset_sens.offset.Y)) M_offset_sens.offset.Y = -3802600;
    if (isnan(M_offset_sens.offset.Z)) M_offset_sens.offset.Z = -3421118;
    if (isnan(M_offset_sens.sens.XX)) M_offset_sens.sens.XX = 1;
    if (isnan(M_offset_sens.sens.YX)) M_offset_sens.sens.YX = 0;
    if (isnan(M_offset_sens.sens.ZX)) M_offset_sens.sens.ZX = 0;
    if (isnan(M_offset_sens.sens.XY)) M_offset_sens.sens.XY = 0;
    if (isnan(M_offset_sens.sens.YY)) M_offset_sens.sens.YY = 0.897;
    if (isnan(M_offset_sens.sens.ZY)) M_offset_sens.sens.ZY = 0;
    if (isnan(M_offset_sens.sens.XZ)) M_offset_sens.sens.XZ = 0;
    if (isnan(M_offset_sens.sens.YZ)) M_offset_sens.sens.YZ = 0;
    if (isnan(M_offset_sens.sens.ZZ)) M_offset_sens.sens.ZZ = 1;

    if (isnan(W_offset_sens.offset.X)) W_offset_sens.offset.X = 0;
    if (isnan(W_offset_sens.offset.Y)) W_offset_sens.offset.Y = 0;
    if (isnan(W_offset_sens.offset.Z)) W_offset_sens.offset.Z = 0;
    if (isnan(W_offset_sens.sens.XX)) W_offset_sens.sens.XX = 0.001221726;
    if (isnan(W_offset_sens.sens.YX)) W_offset_sens.sens.YX = 0;
    if (isnan(W_offset_sens.sens.ZX)) W_offset_sens.sens.ZX = 0;
    if (isnan(W_offset_sens.sens.XY)) W_offset_sens.sens.XY = 0;
    if (isnan(W_offset_sens.sens.YY)) W_offset_sens.sens.YY = 0.001221726;
    if (isnan(W_offset_sens.sens.ZY)) W_offset_sens.sens.ZY = 0;
    if (isnan(W_offset_sens.sens.XZ)) W_offset_sens.sens.XZ = 0;
    if (isnan(W_offset_sens.sens.YZ)) W_offset_sens.sens.YZ = 0;
    if (isnan(W_offset_sens.sens.ZZ)) W_offset_sens.sens.ZZ = 0.001221726;

    if (settings.MA_WINDOW_SIZE == 0xFFFFFFFF) settings.MA_WINDOW_SIZE = 32;
    if (settings.MLD_WINDOW_SIZE == 0xFFFFFFFF) settings.MLD_WINDOW_SIZE = 32;
    if (settings.N == 0xFFFFFFFF) settings.N = 128;
    if (settings.K == 0xFFFFFFFF) settings.K = 128;
    if (settings.AUTO_DELTA == 0xFFFFFFFF) settings.AUTO_DELTA = 1;
    if (settings. INTERRUPT_BY_ANGLE_PATH == 0xFFFFFFFF) settings. INTERRUPT_BY_ANGLE_PATH = 0;
    if (settings.MOVEMENT_CMP_VALUE == 0xFFFF) settings.MOVEMENT_CMP_VALUE = 300;
    if (isnan(settings.W_MIN)) settings.W_MIN = 3.14;
    if (isnan(settings.W_MAX)) settings.W_MAX = 18.84;
    if (isnan(settings.K_PREDICT)) settings.K_PREDICT = 0.5;
    if (isnan(settings.APS_DELTA)) settings.APS_DELTA = M_PI / 360.0;
    if (isnan(settings.HISTORY_ANGLE)) settings.HISTORY_ANGLE = (20.0 * M_PI) / 360.0;
    if (isnan(settings.K_delta)) settings.K_delta = 2.5;
}


/**
 * @brief �������� ���� (� ��������) � ������������� ��������� [-PI, PI].
 *        ���������� fmod � ������������ ��� ������� ������.
 * @param angle_rad ���� ��� "�������������".
 * @return ���� � ��������� [-PI, PI].
 */
__STATIC_FORCEINLINE float wrap_to_pi(float angle_rad)
{
    // ����� �������� NaN � inf
    if (isnan(angle_rad) || isinf(angle_rad))
    {
        return 0.0f;
    }

    float remainder = fmod(angle_rad + M_PI, 2.0f * M_PI);
    if (remainder < 0.0f)
    {
        remainder += 2.0f * M_PI;
    }

    return remainder - M_PI;
}

void fill_aps_buff()
{
    // ��������� ����� ��� 16 ������� ��������� ��� ������ ���������� �� ���������
    // � �������� �� �������� � ��������� �� -PI �� PI �������� PI (�� �������� atan2)
    if (settings.ROT == -1)
    {
        for (int idx = 0; idx < 16; idx++)
        {
            aps_state.aps_point_arr[15 - idx] = wrap_to_pi((idx + 1) * M_PI / 8.0f);
        }
    }
    else if (settings.ROT == 1)
    {
        for (int idx = 0; idx < 16; idx++)
        {
            aps_state.aps_point_arr[idx] = wrap_to_pi(idx * M_PI / 8.0f);
        }
    }
    else
    {
        while (1);
    }

    for (int idx = 0; idx < 16; idx++)
    {
        print(g_dbg_uart, "idx ", idx, " ", aps_state.aps_point_arr[idx]);
    }
}

void run_aps(Vec& axel_raw, Vec& mag_raw, Vec& gyro_raw, aps_callback_t callback)
{
    const int WMIN = settings.W_MIN; // rad/c
    const int WMAX = settings.W_MAX; // rad/c
    const int AUTO_DELTA = settings.AUTO_DELTA;
    const float HISTORY_ANGLE = settings.HISTORY_ANGLE;

    K_predict = settings.K_PREDICT;
    aps_delta = settings.APS_DELTA;

    //���������� �������
    //��������  ��������   i ������ ������� ���� �� ����� ���� �������� ���� ����������� ��������
    G_Summa  = vctr_diff(G_Summa, G_Data[i_ma]);
    M_Summa  = vctr_diff(M_Summa, M_Data[i_ma]);
    W_Summa  = vctr_diff(W_Summa, W_Data[i_ma]);

    //��������� i ������ ������� ���� ����������� ��������
    G_Data[i_ma] = axel_raw;
    M_Data[i_ma] = mag_raw;
    W_Data[i_ma] = gyro_raw;

    // ������� �����  G M
    G_Summa = vctr_summ(G_Summa, G_Data[i_ma]);
    M_Summa = vctr_summ(M_Summa, M_Data[i_ma]);
    W_Summa = vctr_summ(W_Summa, W_Data[i_ma]);

    // ���������� �����������  �������� i ������ ������� � ����� ���� �������� ���� ����������� ��������
    metrics.G_ma = vctr_mltp_n(1.0 / MA_BUFFER_SIZE, G_Summa);
    metrics.M_ma = vctr_mltp_n(1.0 / MA_BUFFER_SIZE, M_Summa);
    metrics.W_ma = vctr_mltp_n(1.0 / MA_BUFFER_SIZE, W_Summa);

    //������������� ����������
    //������ �������� offset � �������� �� �������, ��� ������� ��������� - ��� �������� ����,
    //XY=YX,XZ=ZX,ZY=YZ - ����� ���������, ���������� �� ���������a������� ����.
    metrics.G = callibrate(metrics.G_ma, G_offset_sens);
    metrics.M = callibrate(metrics.M_ma, M_offset_sens);
    metrics.W = callibrate(metrics.W_ma, W_offset_sens);

    metrics.W_g = metrics.W.Z;

    // ���������� �������� ���� ��������� (������������ �� �������).
    metrics.W_g -= metrics.Wg_offset;
    metrics.Wg_1000 = metrics.W_g / 1000.0f; ///rad per 1mc

    //��������� ���, ����� ��������� �a� ����������
    metrics.G_modul = modul(metrics.G);
    metrics.M_modul = modul(metrics.M);
//            metrics.W_modul = W_g;

    // ��������� ��� ������� ���������� ���� ����� �� 32 ��������� ���������.
    // � ���������� ����������� ������������ ��� ���������� ���������� �����
    // K_predict ��� ��������� ����(������) �� ������������.
    float aver_modul_g = 0, summ_modul_g = 0;
    float aver_modul_w = 0, summ_modul_w = 0;
    slo_modul_g = 0;
    slo_modul_m = 0;
    slo_modul_w = 0;

    // ������������ ������ �� �������
    const size_t bytes_to_move = sizeof(float) * (MLD_BUFFER_SIZE - 1);

    memmove(m_modul_buff, &m_modul_buff[1], bytes_to_move);
    memmove(g_modul_buff, &g_modul_buff[1], bytes_to_move);
    memmove(w_modul_buff, &w_modul_buff[1], bytes_to_move);

    //����� � ����� ������
//            m_modul_buff[MLD_BUFFER_SIZE - 1] = M_modul;
    g_modul_buff[MLD_BUFFER_SIZE - 1] = metrics.G_modul;
//            w_modul_buff[MLD_BUFFER_SIZE - 1] = W_modul;
    //������� �������
    for (size_t i = 0; i < MLD_BUFFER_SIZE; i++)
    {
//                aver_modul_m +=  m_modul_buff[i];
        aver_modul_g += g_modul_buff[i];
        aver_modul_w += w_modul_buff[i];
    }
//            aver_modul_m /= MLD_BUFFER_SIZE;
    aver_modul_g /= MLD_BUFFER_SIZE;
    aver_modul_w /= MLD_BUFFER_SIZE;
    //������� �������������� ����������
    for (size_t i = 0; i < MLD_BUFFER_SIZE; i++)
    {
//                summ_modul_m += fabs(m_modul_buff[i] - aver_modul_m);
        summ_modul_g += fabs(g_modul_buff[i] - aver_modul_g);
//                summ_modul_w += fabs(w_modul_buff[i] - aver_modul_w);
    }
//            slo_modul_m = summ_modul_m / MLD_BUFFER_SIZE;
    slo_modul_g = summ_modul_g / MLD_BUFFER_SIZE;
//            slo_modul_w = summ_modul_w / MLD_BUFFER_SIZE;

    // ��� ��� �������� ��������
    if (slo_modul_g < settings.MOVEMENT_CMP_VALUE)
    {
        aps_state.no_mov_delay_tim++;
    }
    else
    {
        aps_state.no_mov_delay_tim = 0;
        aps_state.is_moving = true;
    }

    if (aps_state.no_mov_delay_tim > 1000)
    {
        aps_state.no_mov_delay_tim = 0;
        aps_state.is_moving = false;
//      print(g_dbg_uart, "G_M_angle: ", wrap_to_pi(metrics.angle_aps_m - metrics.angle_aps));
//      print(g_dbg_uart, "GX:", metrics.G.X, ", GY:", metrics.G.Y, ", GZ:", metrics.G.Z, ", MX:", metrics.M.X, ", MY:", metrics.M.Y, ", MZ:", metrics.M.Z);
    }

    // ���������� �����
    metrics.angle_aps = atan2(metrics.G.X, metrics.G.Y);
    metrics.angle_aps_m = atan2(metrics.M.X, metrics.M.Y);
    // �� �������
    if (!aps_state.is_moving)
    {
        metrics.G_M_angle = wrap_to_pi(metrics.angle_aps_m - metrics.angle_aps); //���������� ������� ����� �������� � �������������� ������������ ������
        if (fabs(metrics.W.Z) < 0.1)
            //Wg_offset = W.Z; // ���������� �������� ���� ���������
            metrics.Wg_offset = aver_modul_w;
    }

    // ������ MTF � ������������� � �������� (-PI, PI] (Magnetic Tool Face - ���� ��������� �����������)
    metrics.MTF = wrap_to_pi(metrics.angle_aps_m);// - metrics.G_M_angle);

    // �������� ����
    metrics.angle_zen = atan2(sqrt(metrics.G.X * metrics.G.X + metrics.G.Y * metrics.G.Y), metrics.G.Z);
    // ���� ���������� �� ��������� ��������� 6 �������� ...
    if (fabs(metrics.angle_zen) >= 0.1) //rad
    {
        metrics.angle_azm = M_PI - atan2((metrics.M.Y * metrics.G.X - metrics.M.X * metrics.G.Y) * metrics.G_modul,
                                    (metrics.M.X * metrics.G.X * metrics.G.Z + metrics.M.Y * metrics.G.Y * metrics.G.Z - metrics.M.Z * metrics.G.X * metrics.G.X - metrics.M.Z * metrics.G.Y * metrics.G.Y)); //pi-
    }
    // ... ���� �����������, �� ������ ������� � ����������� ���������
    else
    {
        metrics.angle_azm = metrics.angle_aps_m;
    }

    metrics.angle_aps_deg = (metrics.angle_aps * 57.295779513f);
    metrics.angle_aps_m_deg = (metrics.angle_aps_m * 57.295779513f);
    metrics.angle_zen_deg = (metrics.angle_zen * 57.295779513f);
    metrics.angle_azm_deg = (metrics.angle_azm * 57.295779513f);
    metrics.MTF_deg = (metrics.MTF * 57.295779513f);

// --- �������� �������� ---
//�������������� ���������� � ����� ������������������ MTF (MTF_C) AX2+BX+C
//� ������ ������� ��� ������� �����. � ����������� ����� �������, �����
//� �������[K] �������������� ���������� ������� ��������� dFi rad
//(������ dFi = history_angle = 22 ���� = PI / 8), ���������� �� �������� ��������.
//��� ����� ������� ������ �������� omega � �������� �� tick(� ��� 1 ��).
//� ������������ ��� ����� ����� ������ ��������� dFi/omega. ���� � ���������
//������ ������ (��� omega < PI / 8*128 ��� < ~ 0.175 ����/��) �� � ��������� =
//������� ������.
//������� �������� �� ��� Z ������� �������������� � �������� 30-180 ��/���,
//��� 180-1080 ����/�, ��� PI - 6*PI rad/c, ��� ~ 0.0031416-0,0188496 rad/tick
//��� ����� ��������� ��������� � ����� ������ � �������� 128 - 22.
    // --- �������� �������� � ���������� �������� ---
    // ������ ����� �������� � "������" ������
    aps_state.sin_mtf_buffer[aps_state.circ_idx] = sin(metrics.MTF);
    aps_state.cos_mtf_buffer[aps_state.circ_idx] = cos(metrics.MTF);

    // ���������� ������� ���� ��� �������������.
    int K = static_cast<int>(fabs(HISTORY_ANGLE / metrics.Wg_1000));
    if (K < 3) K = 3;
    if ((size_t)K > PREDICTION_BUFFER_SIZE) K = PREDICTION_BUFFER_SIZE;

    // ���������� ��������
    const float ma_filter_lag_samples = (MA_BUFFER_SIZE - 1) / 2.0f;

    // ������������ ���������, ��������� ������� ��� ��������� �������
    float sin_pred = predict_component_circ(aps_state.sin_mtf_buffer, PREDICTION_BUFFER_SIZE, aps_state.circ_idx, K, ma_filter_lag_samples);
    float cos_pred = predict_component_circ(aps_state.cos_mtf_buffer, PREDICTION_BUFFER_SIZE, aps_state.circ_idx, K, ma_filter_lag_samples);

    // �������������� ���� �� ������������� ���������
    aps_state.prediction = atan2(sin_pred, cos_pred);

    // �������� ������ "������" �� ��������� �������. ������� ������, ���� N - ������� ������ (128 - ��� 2^7):
    aps_state.circ_idx = (aps_state.circ_idx + 1) & (PREDICTION_BUFFER_SIZE - 1);

    if (AUTO_DELTA == 1)
        aps_delta = settings.APS_DELTA + fabs(metrics.Wg_1000 * settings.K_delta);

    float diff = wrap_to_pi(aps_state.prediction - metrics.MTF);
    float K_predict = (fabs(diff) > aps_delta) ? 1.0f : 0.5f;
    aps_state.final_MTF_m_p = wrap_to_pi(metrics.MTF + diff * K_predict - metrics.G_M_angle);

    static int8_t saved_idx = 0;
    static int16_t led_timer = 0;

    for (int8_t idx = 0; idx < 16; idx++)
    {
        if ((aps_state.final_MTF_m_p > aps_state.aps_point_arr[idx] - aps_delta
            && aps_state.final_MTF_m_p < aps_state.aps_point_arr[idx] + aps_delta)
            && saved_idx != idx)
        {
            saved_idx = idx;
            led_timer = 50;
            G_red_led::hi();
            print(g_dbg_uart, "APS ", idx, "-> pre:", aps_state.prediction, ", mtf:", metrics.MTF, ", G-M:", metrics.G_M_angle, ", fin:", aps_state.final_MTF_m_p, ", K:", K_predict, ", Wg:", metrics.W_g, ", ", aps_state.aps_point_arr[idx], "(+/-", aps_delta, ")");
            if (callback != nullptr)
                callback(reinterpret_cast<uint8_t*>(&metrics));
            break;
        }
    }

    if (G_red_led::read() && --led_timer == 0)
        G_red_led::lo();

    i_ma = (i_ma + 1) & (MA_BUFFER_SIZE - 1);
}
//            //������������ ����� �� �������
//            for (int i = 0; i < N - 1; i++)
//            {
//                aps_state.mtf_buffer[i] = aps_state.mtf_buffer[i + 1];//??? ������ �������� MTF[N-1] MTF_C[N-2]
//            }
//            // ���� ������������� ����� +_ PI ������� ���� ����� �� 2PI
//            if (aps_state.mtf_buffer[N - 1] - metrics.MTF < -M_PI)
//            {
//                for (int i = 0; i < N - 1; i++)
//                {
//                    aps_state.mtf_buffer[i] += 2 * M_PI;
//                }
//            }
//            //����� � ����� ������ ������� MTF_C
//            aps_state.mtf_buffer[N - 1] = metrics.MTF;
//            //������� �
//            //omega = (MTF_buff[N - 5] - MTF_buff[N - 1]) / 4;
//            //K = (int)fabs(history_angle / omega);
//            K = (int)fabs(HISTORY_ANGLE / metrics.Wg_1000); //rad per 1mc
////            MA_delay = metrics.Wg_1000 * MA_WINDOW_SIZE / 4.0; //rad ������� �������� �� ���������� ������� ������� �� ��������
//            if (K < 1) K = 1;
//            if (K > N) K = N - 1;
//            // ���������� ��������
//            const float ma_filter_lag_samples = (MA_WINDOW_SIZE - 1) / 2.0f;
//            // ����������� ����� ������� ������� � � ������� ��� ��������
////            aps_state.prediction = predict(abc, &aps_state.mtf_buffer[N - K - 1], S_x[K], K);
//            aps_state.prediction = predict_component(aps_state.mtf_buffer, N, K, ma_filter_lag_samples);
//// --- ����� ��������� ��������� ---
//
//            if (AUTO_DELTA == 1)
//                aps_delta = settings.APS_DELTA + fabs(metrics.Wg_1000 * settings.K_delta);
//            //��������� ���������� � ����������������� ���� � ��������� � ����� K_predict
//            //� �������� ������� ���������� �� �������
//            if (fabs(metrics.MTF - aps_state.prediction) > aps_delta)
//            {
//                K_predict = 1;
////                print(g_dbg_uart, "K_predict == 1");
//            }
//            else
//                K_predict = 0.5;
//
//            aps_state.final_MTF_m_p = (metrics.MTF * (1 - K_predict) + K_predict * aps_state.prediction);

        // ������������ ������ sin � cos
        // (����� ������������ memmove ��� �������������, �� ���� for ����� ��������)
//            for (int i = 0; i < N - 1; i++)
//            {
//                aps_state.sin_mtf_buffer[i] = aps_state.sin_mtf_buffer[i + 1];
//                aps_state.cos_mtf_buffer[i] = aps_state.cos_mtf_buffer[i + 1];
//            }
////            G_green_led::hi();
////            memmove(
////                aps_state.sin_mtf_buffer,           // 1. ��������� �� ������ (���� ��������)
////                &aps_state.sin_mtf_buffer[1],       // 2. ��������� �� ������ ������� (������ ��������)
////                sizeof(float) * (N - 1)             // 3. ���������� ���� ��� �����������
////            );
////
////            // ��� cos_mtf_buffer:
////            memmove(
////                aps_state.cos_mtf_buffer,
////                &aps_state.cos_mtf_buffer[1],
////                sizeof(float) * (N - 1)
////            );
////            G_green_led::lo();
////            // ���������� � ����� ����� ��������
////            aps_state.sin_mtf_buffer[N - 1] = sin(metrics.MTF);
////            aps_state.cos_mtf_buffer[N - 1] = cos(metrics.MTF);
////
////            // ��������� K (��� � ������)
////            int K = static_cast<int>(fabs(HISTORY_ANGLE / metrics.Wg_1000));
////            if (K < 3) K = 3;       // ��� �������� ����� ������� 3 �����
////            if (K > N) K = N;
////
////            // ���������� ��������
////            const float ma_filter_lag_samples = (MA_WINDOW_SIZE - 1) / 2.0f;
////
////            // ������������ ������ ���������� ��������
////            float sin_pred = predict_component(aps_state.sin_mtf_buffer, N, K, ma_filter_lag_samples);
////            float cos_pred = predict_component(aps_state.cos_mtf_buffer, N, K, ma_filter_lag_samples);
////
////            // �������������� ���� �� ������������� ���������
////            aps_state.prediction = atan2(sin_pred, cos_pred);


//            G_green_led::hi(); // ���������� ���������, ����� �������� ����� ���������� ����� �����

//}

//            if ((aps_state.final_MTF_m_p > aps_state.aps_point_arr[aps_state.aps_idx] - aps_delta
//                && aps_state.final_MTF_m_p < aps_state.aps_point_arr[aps_state.aps_idx] + aps_delta)
//                || aps_state.interrupt_by_angle_path == true) //��������� � �����
//            {
//                print(g_dbg_uart, "APS ", aps_state.aps_idx, ": ", aps_state.prediction, ", ", aps_state.final_MTF_m_p, ", ", aps_state.aps_point_arr[aps_state.aps_idx], ", ", aps_delta);
//                aps_state.aps_idx = (aps_state.aps_idx + 1) & (16 - 1);
//                G_red_led::toggle();
//            }

//        if ((i_ma & (MA_WINDOW_SIZE - 1)) == 0) G_green_led::toggle();
//}

//            /* �������� ������ ������ ����� ��������� APS (APS - angular position system) */
//            //������ ������� �� ���������� ��� � PI/8
//            if (aps_state.mode == APS_READY)
//            {
//                error_msg = 0x00;
//                error_sector_msg = 0x0000;
//                aps_state.tim = 0;//
//                //��� �������� !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/////                i_16_tWg = (int16_t)(W_g * 572.96);
//                /* ���� �������� ���������� (��������) */
//                if (metrics.W_g > WMIN)
//                {
//                    G_pin_int::lo();
//                    G_red_led::lo();
//                    aps_state.mode = APS_WAIT_NEXT_POINT;
////                    delay = MA_delay; //������� ��������
////                    turnStartTime = Now;
//                    aps_state.rot = -1.0; // ��� ������ � ����� ����� ���������
//                    aps_state.angle_path_wg = 0;
//                    // ����� � ������������� ����
//                    aps_state.start_APS = metrics.MTF;// �������� ���� ����� �������� ���
//                    //��� ��������!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/////                    fApsStartAngle = MTF * rot;
/////                    i_16_tApsStartAngle = (int16_t)(MTF * rot * 572.96);
//                    //��������� ����� ��� 16 ������� ��������� ��� ������ ���������� �� ���������
//                    // � �������� �� �������� � ��������� �� -PI �� PI �������� PI (�� �������� atan2)
//                    for (int idx = 0; idx < 16; idx++)
//                    {
//                        aps_state.aps_point_arr[idx] = aps_state.start_APS + idx * aps_state.rot * M_PI / 8.0;
//                        while (aps_state.aps_point_arr[idx] <= -M_PI) aps_state.aps_point_arr[idx] += 2*M_PI;
//                    }
//
//                    aps_point = aps_state.aps_point_arr[0]; //aps_point - ����������� � uart ����� ������ �o��� �������
//                    G_pin_int::hi();
//                    G_red_led::hi();
//                    // ---  ��� ����������� ������������� ������� ---
////                    cyclogram_state = TX_DIRECT_MESS;
//                    aps_idx_out = aps_state.aps_idx;
//                    variables_reset();
////------------------------------------------------------------------------------------
//                    aps_state.pulse = 0;//���������� ������k
//                    aps_state.aps_idx++;// ������������� ������� ������� ������� ���������
//                #ifdef MAX_W_STOP
//                    beth_pulse_delay_tim = 0;//����� ����� ����������- �������������� -
//                #endif
//                }//end if(W_g > WMIN)
//                /* ���� �������� ������������ (��� �������� ��� ������� ��������) */
//                else
//                {
////------------------------------------------------------------------------------------
//                    // ��� ����������� ������������� �������
//                    // �������������� ���������
////                     = TX_NOT_DIRECT_MESS;
//                    variables_reset();
//                    aps_state.mode = APS_COMPLETE;
////------------------------------------------------------------------------------------
/////                    bitWrite(error_msg, MIN_W_ERR_BIT, 1);
/////                    bitWrite(error_msg, VIBRATION_BIT, bIsMoving);
//                }
//            } /* if (aps_state.mode == APS_READY) */
//            /* �������� ���������� ��������� ������� �����. */
//            else if (aps_state.mode == APS_WAIT_NEXT_POINT)
//            {
//            #ifdef MAX_W_STOP
//                beth_pulse_delay_tim ++;//����� ����� ����������- �������������� -
//            #endif
//                aps_state.pulse++;//������������ �������� �� ���������� � �� - ��������������- � � ���� ���������� 1 ��
//                /*  ���� ������� ������������ �������� ������ ������ ��������� */
//                if (aps_state.pulse > PULSE_WIDTH) //PULSE_WIDTH - ������������ �������� �� ����������
//                {
//                    G_pin_int::lo();
//                    G_red_led::lo();
//                }
//                /* �� ������� ������ ���� �� 5000 ��� */
//                aps_state.tim++;
//                if (aps_state.tim > 5000)
//                {
//                    aps_state.tim = 0;
//                    aps_state.mode = APS_READY;
////------------------------------------------------------------------------------------
//                    // ��� ����������� ������������� �������
////                    cyclogram_state = RX_SURVEY;
////------------------------------------------------------------------------------------
////                    delay = 0;
/////                    bitWrite(error_msg, TIMEOUT_ERR_BIT, 1);
//                }
//
//                aps_state.angle_path_wg += metrics.Wg_1000;  //������� ���� �� ������ ������� � �������� (Wg_1000 - ������� �������� ���/�������)
//                //angle_path_wg += ( w_modul_buff[MLD_WINDOW_SIZE-1] + w_modul_buff[MLD_WINDOW_SIZE-2])/2.0;
//
//                if (aps_state.angle_path_wg > (M_PI / 8 + 4.0 * aps_delta) && settings.INTERRUPT_BY_ANGLE_PATH == 1)
//                {
//                    aps_state.interrupt_by_angle_path = true;//��������� �������
//                }
//
//                /* ����� � ������������� ���� */
//                aps_point = aps_state.aps_point_arr[aps_state.aps_idx]; //aps_point - ����������� � uart ����� ������ �o��� �������
//
//                if ((aps_state.final_MTF_m_p > aps_state.aps_point_arr[aps_state.aps_idx] - aps_delta
//                    && aps_state.final_MTF_m_p < aps_state.aps_point_arr[aps_state.aps_idx] + aps_delta)
//                    || aps_state.interrupt_by_angle_path == true) //��������� � �����
//                {
//                #ifdef MAX_W_STOP
//                    if (beth_pulse_delay_tim <= BETH_PULSE_DELAY)
//                    {
//                        aps_state.mode = APS_COMPLETE;
/////                        bitWrite(error_msg, MAX_W_ERR, 1);
//                    }
//                    else
//                    {
//                #endif
//                        G_pin_int::hi();
//                        G_red_led::hi();
////------------------------------------------------------------------------------------
//                        // ��� ����������� ������������� �������
////                        cyclogram_state = TX_DIRECT_MESS;
//                        aps_idx_out = aps_state.aps_idx;
//                        variables_reset();
////------------------------------------------------------------------------------------
//                        aps_state.pulse = 0;//���������� ������� ������� ��� �������� ����������
/////                        bitWrite(error_msg, NOT_REACH_END, 1);
/////                        bitWrite(error_msg, K_PREDIKT_1, (int)(K_predict));
/////                        bitWrite(error_sector_msg, aps_idx, interrupt_by_angle_path);
//                        //aps_arr[aps_idx] = (angle_path_wg-PI/8)/(slo_modul_w);//�� ������� 16 ��������
//                        //aps_m_arr[aps_idx] = (angle_path_wg-PI/8)/APS_DELTA;
//                        aps_state.interrupt_by_angle_path = false;
//                        aps_state.angle_path_wg = 0;//�������� ������� ���� � ������ ����������
//                        aps_state.aps_idx++;// ������������� ������� ������� ������� ���������
//                        /* C����� ������ ������, 16 ��������� ������ */
//                        if (aps_state.aps_idx == 16)//���� ������� ������ ������
//                        {
////                            aps_m_arr[16] = aps_delta * 57.296;//����� ������ �o��� �������
/////                            bitWrite(error_msg, NOT_REACH_END, 0);
//                            aps_state.aps_idx = 0;
//                            aps_state.mode = APS_COMPLETE;
//
//                            G_pin_int::lo(); //GPIO_WritePin(PIN_INTERRUPT, 0);
//                            G_red_led::lo(); //GPIO_WritePin(LED_2, 0);
//                        }
//                #ifdef MAX_W_STOP //max_W_stop
//                    }
//                    beth_pulse_delay_tim = 0;
//                #endif
//                }
//            }//end if APS_WAIT_NEXT_POINT
//
////------------------------------------------------------------------------------------
//// ��� ����������� ������������� �������
//            tx_work_units_cntr++;
////------------------------------------------------------------------------------------
//
//
////        }// end if STATE_NORMAL
//
//
////        if(i_ma < (MA_WINDOW_SIZE - 1))
////            i_ma++;
////        else i_ma = 0;
//        i_ma = (i_ma + 1) & (MA_WINDOW_SIZE - 1);
//
//    } //end if not idle
//
//     GPIO_WritePin(LED_1, 0);
//}
