#include "main.h"
#include <math.h>

float Gx, Gy, Gz, Bx, By, Bz, Wx, Wy, Wz;
float W_g;
float Wg_offset = 0;
float Wg_1000 = 0;

//extern uint64_t Now;
extern uint64_t turnStartTime;
extern float aps_arr[17];
float aps_m_arr[17];
//extern uartPrintMode uPm;
Aps_state apsMMode;
volatile bool bIsMoving = false;
//�������� �� ����������, ���� ������� � �������� �������� �� ����
int MA_WINDOW_SIZE;  //���� ����������� �������� ��� ������ � ��� 32
int MLD_WINDOW_SIZE; //���� C��
int delta; //����������� 1, ���� ������� ���������� �� ���������� ������� ����
float APS_DELTA = M_PI / 360; // - 1 ������ (� ����� ��������� ������������ ������ ������)
float WMIN = 0.63;// rad/c
float WMAX = 18;// 6 ��.�

// ��� �����, ����� ����������� �������� � ������������� ������ � ����������� ��������
int Wxyz[3];
struct Vec G_raw, M_raw, W_raw, G_ma, M_ma, W_ma, G, M, W ;
struct Vec G_Summa = {0, 0, 0}, M_Summa = {0, 0, 0}, W_Summa = {0, 0, 0}, G_Data[32], M_Data[32], W_Data[32] ;

//��� ����������
struct Cal G_offset_sens = { 3909340, 3912131, 3944069, 1, 0, 0, 0, 1, 0, 0, 0, 1 },
           M_offset_sens = { -4910770, -3802600, -3421118, 1, 0, 0, 0, 0.897, 0, 0, 0, 1 },
           W_offset_sens = { 0, 0, 0, 0.001221726, 0, 0, 0, 0.001221726, 0, 0, 0, 0.001221726 }; //rad per digit

//           G_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 },
//           M_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 },
//           W_offset_sens_add = { 0,0,0,1,0,0,0,1,0,0,0,1 };
//���������
struct Set settings;
//����
float angle_aps, angle_zen, angle_azm, angle_aps_m, MTF, MTF_m_p, start_APS = 0;
float angle_zen_deg, angle_aps_deg, angle_azm_deg, angle_aps_m_deg, MTF_deg;

//��� ��������� ��������� � ���
float G_modul, M_modul, W_modul;
float omega;
float g_modul_buff[32], m_modul_buff[32], w_modul_buff[32], slo_modul_g, slo_modul_m, slo_modul_w;//��� ���
float MTF_buff[128];//����� ��� ��������� ���������
float abc[3] = {0.0f};//�����. ������. �������� 2�� ������� abc[0]*�*� + abc[1]*� + abc[2]
//float delta = PI / 180; //����� - 1 ������
float history_angle = (20 * M_PI) / 180; //������� �������� ������ ������������
float prediction;// ��� �������������� ��������
int N = 128, K = 128;
float K_predict = 0.5;
extern float S_x[130][5];//�������������� ����������� ����� ��� ��������
float aps_point = 0;
float aps_point_arr[16] = {0.0f};
float G_M_angle = 0;
uint16_t aps_idx = 0;
float sector = M_PI / 8;
float idx;
float rot = 1;
volatile uint32_t  tim = 0, pulse = 0, beth_pulse_delay_tim = 0, no_mov_delay_tim = 0;
volatile float angle_path_wg = 0;
const uint32_t PULSE_WIDTH = 3;//������ �������� � ��
const uint32_t BETH_PULSE_DELAY = 32 ;//����� ����� ����������
volatile bool start_circle;
float MA_delay = 0;
volatile float delay = 0;
uint16_t error_sector_msg = 0;
uint16_t error_msg = 0;
bool interrupt_by_angle_path = false;

//------------------------------------------------------------------------------------
// ��� ����������� ������������� �������
uint16_t aps_idx_out;
//extern CYCLOGRAM cyclogram_state;
extern uint16_t tx_work_time_units;
extern uint16_t N_Tx;
uint16_t next_tx_start_units;
extern uint16_t operate_byte;
extern uint16_t freq;
int16_t tx_work_units_cntr;

bool last_Tx_is_sent;
//extern bool finish;
//extern uint16_t next_Tx_N ;

void variables_reset(void)
{
    tx_work_units_cntr = -1;
    next_tx_start_units = 0;
    last_Tx_is_sent = false;
//    finish = false;
//    next_Tx_N = 0;
}

void read_cal_and_settings()
{
//   FLASH_ReadCal(&G_offset_sens, &M_offset_sens, &W_offset_sens);
//   FLASH_ReadSet(&settings);
    G_fram::read_buf(&G_offset_sens, Fram_markup::G_OFFSET_ADDR, sizeof(Cal));
    G_fram::read_buf(&M_offset_sens, Fram_markup::M_OFFSET_ADDR, sizeof(Cal));
    G_fram::read_buf(&W_offset_sens, Fram_markup::W_OFFSET_ADDR, sizeof(Cal));
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
    if (settings.auto_delta == 0xFFFFFFFF) settings.auto_delta = 1;
    if (settings. INTERRUPT_BY_ANGLE_PATH == 0xFFFFFFFF) settings. INTERRUPT_BY_ANGLE_PATH = 0;
    if (settings.MOVEMENT_CMP_VALUE == 0xFFFF) settings.MOVEMENT_CMP_VALUE = 300;
    if (isnan(settings.W_MIN)) settings.W_MIN = 3.14;
    if (isnan(settings.W_MAX)) settings.W_MAX = 18.84;
    if (isnan(settings.K_predict)) settings.K_predict = 0.5;
    if (isnan(settings.APS_DELTA)) settings.APS_DELTA = M_PI / 360.0;
    if (isnan(settings.history_angle)) settings.history_angle = (20.0 * M_PI) / 360.0;
    if (isnan(settings.K_delta)) settings.K_delta = 2.5;
}

extern "C" void EXTI0_IRQHandler()
{
    static uint16_t i_filter = 0;
    N = settings.N;
    K = settings.K;
    K_predict = settings.K_predict;
    MA_WINDOW_SIZE = settings.MA_WINDOW_SIZE;
    MLD_WINDOW_SIZE = settings.MLD_WINDOW_SIZE;
    WMIN = settings.W_MIN;// rad/c
    WMAX = settings.W_MAX;// rad/c
    APS_DELTA = settings.APS_DELTA;
    history_angle = settings.history_angle;

////    if((bExtADCReady)&&(device_state != STATE_IDLE))
    {
        auto mag = g_mag.read_xyz();
        auto axel = g_axel.read_xyz();
        auto gyro = g_gyro.read_xyz();

        //���������� �������
        //��������  ��������   i ������ ������� ���� �� ����� ���� �������� ���� ����������� ��������
        G_Summa  = vctr_diff(G_Summa, G_Data[i_filter]);
        M_Summa  = vctr_diff(M_Summa, M_Data[i_filter]);
        W_Summa  = vctr_diff(W_Summa, W_Data[i_filter]);

        //��������� i ������ ������� ���� ����������� ��������
        G_Data[i_filter] = {(float)axel.a_x, (float)axel.a_y, (float)axel.a_z}; //G_raw;
        M_Data[i_filter] = {(float)mag.X, (float)mag.Y, (float)mag.Z}; //M_raw;
        W_Data[i_filter] = {(float)gyro.w_x, (float)gyro.w_y, (float)gyro.w_z}; //W_raw;

        // ������� �����  G M
        G_Summa = vctr_summ(G_Summa, G_Data[i_filter]);
        M_Summa = vctr_summ(M_Summa, M_Data[i_filter]);
        W_Summa = vctr_summ(W_Summa, W_Data[i_filter]);

        /* ���������� �����������  �������� i ������ ������� � ����� ���� �������� ���� ����������� �������� */
        G_ma = vctr_mltp_n(1.0 / MA_WINDOW_SIZE, G_Summa);
        M_ma = vctr_mltp_n(-1.0 / MA_WINDOW_SIZE, M_Summa);
        W_ma = vctr_mltp_n(1.0 / MA_WINDOW_SIZE, W_Summa);

        G = G_ma;
        M = M_ma;
        W = W_ma;

//        if (device_state == STATE_NORMAL)
//        {
            //������������� ����������
            //������ �������� offset � �������� �� �������, ��� ������� ��������� - ��� �������� ����,
            //XY=YX,XZ=ZX,ZY=YZ - ����� ���������, ���������� �� ���������a������� ����.
            G = callibrate(G, G_offset_sens);
            M = callibrate(M, M_offset_sens);
            W = callibrate(W, W_offset_sens);

            W_g = W.Z;

            /* ���������� �������� ���� ��������� (������������ �� �������). */
            W_g -= Wg_offset;
            Wg_1000 = W_g / 1000.0; ///rad per 1mc

            //��������� ���, ����� ��������� �a� ����������
            G_modul = modul(G);
            M_modul = modul(M);
            W_modul = W_g;

            //��������� ��� ������� ���������� ���� ����� �� 32 ��������� ���������.
            //� ���������� ����������� ������������ ��� ���������� ���������� �����
            //K_predict ��� ��������� ����(������) �� ������������.
            // 10 mkc
            float aver_modul_m = 0, summ_modul_m = 0;
            float aver_modul_g = 0, summ_modul_g = 0;
            float aver_modul_w = 0, summ_modul_w = 0;
            slo_modul_g = 0;
            slo_modul_m = 0;
            slo_modul_w = 0;

            //������������ ����� �� �������
            for (int i = 0; i < MLD_WINDOW_SIZE - 1; i++)
            {
                m_modul_buff[i] = m_modul_buff[i + 1];
                g_modul_buff[i] = g_modul_buff[i + 1];
                w_modul_buff[i] = w_modul_buff[i + 1];
            }

            //����� � ����� ������
//            m_modul_buff[MLD_WINDOW_SIZE - 1] = M_modul;
            g_modul_buff[MLD_WINDOW_SIZE - 1] = G_modul;
//            w_modul_buff[MLD_WINDOW_SIZE - 1] = W_modul;
            //������� �������
            for (int i = 0; i < MLD_WINDOW_SIZE; i++)
            {
//                aver_modul_m +=  m_modul_buff[i];
                aver_modul_g +=  g_modul_buff[i];
                aver_modul_w +=  w_modul_buff[i];
            }
//            aver_modul_m /= MLD_WINDOW_SIZE;
            aver_modul_g /= MLD_WINDOW_SIZE;
            aver_modul_w /= MLD_WINDOW_SIZE;
            //������� �������������� ����������
            for (int i = 0; i < MLD_WINDOW_SIZE; i++)
            {
//                summ_modul_m += fabs(m_modul_buff[i] - aver_modul_m);
                summ_modul_g += fabs(g_modul_buff[i] - aver_modul_g);
//                summ_modul_w += fabs(w_modul_buff[i] - aver_modul_w);
            }
//            slo_modul_m = summ_modul_m / MLD_WINDOW_SIZE;
            slo_modul_g = summ_modul_g / MLD_WINDOW_SIZE;
//            slo_modul_w = summ_modul_w / MLD_WINDOW_SIZE;

            /* ��������� ��� ��� �������� �������� */
            if (slo_modul_g < settings.MOVEMENT_CMP_VALUE)
            {
                no_mov_delay_tim++;
            }
            else
            {
                no_mov_delay_tim = 0;
                bIsMoving = true;
            }

            if (no_mov_delay_tim > 1000)
            {
                no_mov_delay_tim = 0;
                bIsMoving = false;
            }

            //��������� ����
            angle_aps = atan2(G.Y, G.X);
            angle_aps_m = atan2(M.Y, M.X);
            // �� �������
            if (!bIsMoving)
            {
                G_M_angle = angle_aps_m - angle_aps; //���������� ������� ����� �������� � �������������� ������������ ������
                if (fabs(W.Z) < 0.1)
                    //Wg_offset = W.Z; // ���������� �������� ���� ���������
                    Wg_offset = aver_modul_w;
            }

            /* ������ MTF (Magnetic Tool Face - ���� ��������� �����������) */
            MTF = angle_aps_m - G_M_angle;
            /* ������������ MTF � �������� (-PI, PI] */
            while (MTF > M_PI) MTF -= 2*M_PI;
            while (MTF <= -M_PI)  MTF += 2*M_PI;
            //�������� ����
            angle_zen = atan2(sqrt(G.X * G.X + G.Y * G.Y), G.Z);
            /* ���� ���������� �� ��������� ��������� 6 �������� */
            if (fabs(angle_zen) >= 0.1) //rad
                angle_azm = M_PI - atan2((M.Y*G.X - M.X*G.Y)*G_modul, (M.X*G.X*G.Z + M.Y*G.Y*G.Z - M.Z*G.X*G.X - M.Z*G.Y*G.Y)); //pi-
            /* ���� �����������, �� ������ ������� � ����������� ��������� */
            else angle_azm = angle_aps_m;

            angle_aps_deg = (angle_aps*57.296);
            angle_aps_m_deg = (angle_aps_m*57.296);
            angle_zen_deg = (angle_zen*57.296);
            angle_azm_deg = (angle_azm*57.296);
            MTF_deg = (MTF*57.296);

//�������� ��������
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

            //������������ ����� �� �������
            for (int i = 0; i < N - 1; i++)
            {
                MTF_buff[i] = MTF_buff[i + 1];//??? ������ �������� MTF[N-1] MTF_C[N-2]
            }
            // ���� ������������� ����� +_ PI ������� ���� ����� �� 2PI
            if (MTF_buff[N - 1] - MTF < -M_PI)
            {
                for (int i = 0; i < N - 1; i++)
                {
                    MTF_buff[i] += 2*M_PI;
                }
            }
            //����� � ����� ������ ������� MTF_C
            MTF_buff[N - 1] = MTF;
            //������� �
            omega = (MTF_buff[N - 5] - MTF_buff[N - 1]) / 4;
            //K = (int)fabs(history_angle / omega);
            K = (int)fabs(history_angle / Wg_1000);//rad per 1mc
            MA_delay = Wg_1000 * MA_WINDOW_SIZE / 4.0; //rad ������� �������� �� ���������� ������� ������� �� ��������
            if (K < 1) K = 1;
            if (K > N) K = N - 1;
            // ����������� ����� ������� ������� � � ������� ��� ��������
            prediction = predict(abc, &MTF_buff[N - K - 1], S_x[K], K);
/////����� ��������� ��������� //////////////////////////////////////////////////////////////////////////////////////////////

            if (settings.auto_delta == 1)
                APS_DELTA = settings.APS_DELTA + fabs(Wg_1000 * settings.K_delta);
            //��������� ���������� � ����������������� ���� � ��������� � ����� K_predict
            //� �������� ������� ���������� �� �������
            if (fabs(MTF - prediction) > APS_DELTA)
                K_predict = 1;
            else
                K_predict = 0.5;

            MTF_m_p = (MTF * (1 - K_predict) + K_predict * prediction);

            /* �������� ������ ������ ����� ��������� APS (APS - angular position system) */
            //������ ������� �� ���������� ��� � PI/8
            if (apsMMode == APS_READY)
            {
                error_msg = 0x00;
                error_sector_msg = 0x0000;
                tim = 0;//
                //��� �������� !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
///                i_16_tWg = (int16_t)(W_g * 572.96);
                /* ���� �������� ���������� (��������) */
                if (W_g > WMIN)
                {
                    G_pin_int::lo();
                    G_red_led::lo();
                    apsMMode = APS_WAIT_NEXT_POINT;
                    delay = MA_delay; //������� ��������
//                    turnStartTime = Now;
                    rot = -1.0; // ��� ������ � ����� ����� ���������
                    angle_path_wg = 0;
                    // ����� � ������������� ����
                    start_APS = MTF;// �������� ���� ����� �������� ���
                    //��� ��������!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
///                    fApsStartAngle = MTF * rot;
///                    i_16_tApsStartAngle = (int16_t)(MTF * rot * 572.96);
                    //��������� ����� ��� 16 ������� ��������� ��� ������ ���������� �� ���������
                    // � �������� �� �������� � ��������� �� -PI �� PI �������� PI (�� �������� atan2)
                    for (int idx = 0; idx < 16; idx++)
                    {
                        aps_point_arr[idx] = start_APS + idx * rot * M_PI / 8.0;
                        while (aps_point_arr[idx] <= -M_PI) aps_point_arr[idx] += 2*M_PI;
                    }

                    aps_point = aps_point_arr[0]; //aps_point - ����������� � uart ����� ������ �o��� �������
                    G_pin_int::hi();
                    G_red_led::hi();
//------------------------------------------------------------------------------------
                    // ��� ����������� ������������� �������
//                    cyclogram_state = TX_DIRECT_MESS;
                    aps_idx_out = aps_idx;
                    variables_reset();
//------------------------------------------------------------------------------------
                    pulse = 0;//���������� ������k
                    aps_idx++;// ������������� ������� ������� ������� ���������
                #ifdef MAX_W_STOP
                    beth_pulse_delay_tim = 0;//����� ����� ����������- �������������� -
                #endif
                }//end if(W_g > WMIN)
                /* ���� �������� ������������ (��� �������� ��� ������� ��������) */
                else
                {
//------------------------------------------------------------------------------------
                    // ��� ����������� ������������� �������
                    // �������������� ���������
//                     = TX_NOT_DIRECT_MESS;
                    variables_reset();
                    apsMMode = APS_COMPLETE;
//------------------------------------------------------------------------------------
///                    bitWrite(error_msg, MIN_W_ERR_BIT, 1);
///                    bitWrite(error_msg, VIBRATION_BIT, bIsMoving);
                }
            } /* if (apsMMode == APS_READY) */
            /* �������� ���������� ��������� ������� �����. */
            else if (apsMMode == APS_WAIT_NEXT_POINT)
            {
            #ifdef MAX_W_STOP
                beth_pulse_delay_tim ++;//����� ����� ����������- �������������� -
            #endif
                pulse++;//������������ �������� �� ���������� � �� - ��������������- � � ���� ���������� 1 ��
                /*  ���� ������� ������������ �������� ������ ������ ��������� */
                if (pulse > PULSE_WIDTH) //PULSE_WIDTH - ������������ �������� �� ����������
                {
                    G_pin_int::lo();
                    G_red_led::lo();
                }
                /* �� ������� ������ ���� �� 5000 ��� */
                tim++;
                if (tim > 5000)
                {
                    tim = 0;
                    apsMMode = APS_READY;
//------------------------------------------------------------------------------------
                    // ��� ����������� ������������� �������
//                    cyclogram_state = RX_SURVEY;
//------------------------------------------------------------------------------------
                    delay = 0;
///                    bitWrite(error_msg, TIMEOUT_ERR_BIT, 1);
                }

                angle_path_wg += Wg_1000;  //������� ���� �� ������ ������� � �������� (Wg_1000 - ������� �������� ���/�������)
                //angle_path_wg += ( w_modul_buff[MLD_WINDOW_SIZE-1] + w_modul_buff[MLD_WINDOW_SIZE-2])/2.0;

                if (angle_path_wg > (M_PI / 8 + 4.0*APS_DELTA) && settings.INTERRUPT_BY_ANGLE_PATH == 1)
                {
                    interrupt_by_angle_path = true;//��������� �������
                }

                /* ����� � ������������� ���� */
                aps_point = aps_point_arr[aps_idx]; //aps_point - ����������� � uart ����� ������ �o��� �������

                if ((MTF_m_p > aps_point_arr[aps_idx] - APS_DELTA && MTF_m_p < aps_point_arr[aps_idx] + APS_DELTA)
                        || interrupt_by_angle_path == true) //��������� � �����
                {
                #ifdef MAX_W_STOP
                    if (beth_pulse_delay_tim <= BETH_PULSE_DELAY)
                    {
                        apsMMode = APS_COMPLETE;
///                        bitWrite(error_msg, MAX_W_ERR, 1);
                    }
                    else
                    {
                #endif
                        G_pin_int::hi();
                        G_red_led::hi();
//------------------------------------------------------------------------------------
                        // ��� ����������� ������������� �������
//                        cyclogram_state = TX_DIRECT_MESS;
                        aps_idx_out = aps_idx;
                        variables_reset();
//------------------------------------------------------------------------------------
                        pulse = 0;//���������� ������� ������� ��� �������� ����������
///                        bitWrite(error_msg, NOT_REACH_END, 1);
///                        bitWrite(error_msg, K_PREDIKT_1, (int)(K_predict));
///                        bitWrite(error_sector_msg, aps_idx, interrupt_by_angle_path);
                        //aps_arr[aps_idx] = (angle_path_wg-PI/8)/(slo_modul_w);//�� ������� 16 ��������
                        //aps_m_arr[aps_idx] = (angle_path_wg-PI/8)/APS_DELTA;
                        interrupt_by_angle_path = false;
                        angle_path_wg = 0;//�������� ������� ���� � ������ ����������
                        aps_idx++;// ������������� ������� ������� ������� ���������
                        /* C����� ������ ������, 16 ��������� ������ */
                        if (aps_idx == 16)//���� ������� ������ ������
                        {
                            aps_m_arr[16] = APS_DELTA * 57.296;//����� ������ �o��� �������
///                            bitWrite(error_msg, NOT_REACH_END, 0);
                            aps_idx = 0;
                            apsMMode = APS_COMPLETE;

                            G_pin_int::lo(); //GPIO_WritePin(PIN_INTERRUPT, 0);
                            G_red_led::lo(); //GPIO_WritePin(LED_2, 0);
                        }
                #ifdef MAX_W_STOP //max_W_stop
                    }
                    beth_pulse_delay_tim = 0;
                #endif
                }
            }//end if APS_WAIT_NEXT_POINT

//------------------------------------------------------------------------------------
// ��� ����������� ������������� �������
            tx_work_units_cntr++;
//------------------------------------------------------------------------------------


//        }// end if STATE_NORMAL


        if(i_filter < (MA_WINDOW_SIZE - 1))
            i_filter++;
        else i_filter = 0;

    } //end if not idle

    // GPIO_WritePin(LED_1, 0);
}
