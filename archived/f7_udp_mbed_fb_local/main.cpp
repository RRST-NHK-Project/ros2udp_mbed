/*
キャチロボ2024
ROS2から角度司令を受信する
エンコーダーの値をもとに角度に対してPIDをかける
メイン基板V2はMD3,MD4が使えない
PIDはタイマー割り込みに変更予定
2024/09/10
*/

/*
MD1: シューティングコンベアθ軸
MD2: シューティングコンベア送り
MD3: 使用不可
MD4: 使用不可
MD5: シューティングコンベアr軸
MD6: 空き
MD7: ソーティングコンベア詰まり防止
MD8: ソーティングコンベア送り
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

//---------------------------QEI---------------------------//
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X2_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X2_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X2_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X2_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X2_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X2_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/
//---------------------------QEI---------------------------//

using ThisThread::sleep_for;

void receive(UDPSocket *receiver);

//---------------------------ピンの割り当て---------------------------//
// PWM
PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PC_7);
PwmOut MD4P(PC_6);
PwmOut MD5P(PC_8);
PwmOut MD6P(PC_9);

// DIR
DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PD_5);
DigitalOut MD4D(PD_6);
DigitalOut MD5D(PD_7);
DigitalOut MD6D(PC_10);

//サーボ
PwmOut SERVO1(PB_1);
PwmOut SERVO2(PB_6);
PwmOut SERVO3(PD_13);
PwmOut SERVO4(PD_12);

//スイッチ
DigitalIn SW1(PF_15);
DigitalIn SW2(PG_14);
DigitalIn SW3(PG_9);
DigitalIn SW4(PE_7);

//パイロットランプ（普通のデジタルIOとしても使用可）
DigitalOut PL_1(PF_12);
DigitalOut PL_2(PF_13);

// CAN
CAN can{PD_0, PD_1, (int)1e6}; // rd,td,1Mhz
//---------------------------ピンの割り当て---------------------------//

//---------------------------For PID---------------------------//
int Pulse[7] = {0, 0, 0, 0, 0, 0, 0};
int last_Pulse[7] = {0, 0, 0, 0, 0, 0, 0};
int dt = 0;
double dt_d = 0; // casted dt
double deg[7];

double target[7] = {0, 0, 0, 0, 0, 0, 0};
double Kp;
double Ki;
double Kd;
double Error[7] = {0, 0, 0, 0, 0, 0, 0};
double last_Error[7] = {0, 0, 0, 0, 0, 0, 0};
double Integral[7] = {0, 0, 0, 0, 0, 0, 0};
double Differential[7] = {0, 0, 0, 0, 0, 0, 0};
double Output[7] = {0, 0, 0, 0, 0, 0, 0};
double deg_limit;
double pwm_limit; // PWM出力制限　絶対に消すな
//---------------------------For PID---------------------------//

double mdd[9];
double mdp[9];

int main() {

  //---------------------------PWM Settings---------------------------//
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz) * 10^3
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */
  //---------------------------PWM Settings---------------------------//

  using namespace std::chrono;

  Timer t; // PIDで使うためのタイマー
  t.start();
  SocketAddress source;
  char buffer[64];

  int data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  //---------------------------PID parameters---------------------------//
  //ゲインを一括で設定する。個別で設定する場合はPIDのfor内に記述する
  Kp = 0.075;          // Pゲイン
  Ki = 0.0;          // Iゲイン
  Kd = 0.025;          // Dゲイン
  deg_limit = 360.0; //上限
  pwm_limit = 0.1; // MDに出力されるduty比の上限を設定する。安全装置の役割を持つ
                   //動作に影響するようなら#defineに変更
  //---------------------------PID parameters---------------------------//

  while (1) {
    //---------------------------エンコーダーの値をもとに角度（度数法）を計算---------------------------//
    Pulse[1] = ENC1.getPulses();
    Pulse[2] = ENC2.getPulses();
    Pulse[3] = ENC3.getPulses();
    Pulse[4] = ENC4.getPulses();
    Pulse[5] = ENC5.getPulses();
    Pulse[6] = ENC6.getPulses();

    //---------------------------エンコーダーの値をもとに角度（度数法）を計算---------------------------//

    t.stop(); //タイマーを停止する、回転数の計算とPIDに使う
    dt = duration_cast<milliseconds>(t.elapsed_time())
             .count(); // msで前回からの経過時間を取得

    //---------------------------PID---------------------------//

    data[1] = 4096;

    dt_d = (double)dt;

    int i = 1;
    target[i] = (double)data[i];
    Error[i] = target[i] - (Pulse[i]); // P制御,目標値と現在値の差分をとる
    Integral[i] += ((Error[i] + last_Error[i]) * dt_d /
                    2); // I制御,差分の時間積分、台形で近似して計算
    Differential[i] = (Error[i] - last_Error[i]) / dt_d; // D制御,差分の時間微分

    last_Error[i] = Error[i];

    const double IntegralLimit = 2.0; // 適切な範囲を設定
    if (Integral[i] > IntegralLimit) {
      Integral[i] = IntegralLimit;
    } else if (Integral[i] < -IntegralLimit) {
      Integral[i] = -IntegralLimit;
    }

    Output[i] = ((Kp * Error[i]) + (Ki * Integral[i]) +
                 (Kd * Differential[i])); // PID制御,ゲインをかけて足し合わせる

    mdp[i] = Output[i] / deg_limit;

    // 安全のためPWMの出力を制限　絶対に消すな
    if (mdp[i] > 0) {
      mdd[i] = 1;
    } else if (mdp[i] < 0) {
      mdd[i] = 0;
    }
    mdp[i] = fabs(mdp[i]);

    if (mdp[i] > pwm_limit) {
      mdp[i] = pwm_limit;
    }
    // end

        // MDに出力

    MD1D = mdd[1];
    MD2D = mdd[2];
    MD3D = mdd[3];
    MD4D = mdd[4];
    MD5D = mdd[5];
    MD6D = mdd[6];

    MD1P = mdp[1];
    MD2P = mdp[2];
    MD3P = mdp[3];
    MD4P = mdp[4];
    MD5P = mdp[5];
    MD6P = mdp[6];

    //---------------------------PID---------------------------//

    t.reset(); //タイマーをリセット
    t.start(); //タイマーを開始

    printf("%lf, %d, %lf, %lf, %lf\n", mdp[1], Pulse[1], Error[1], Integral[1],
           Differential[1]);

    // モーターがうまく回らないときは要調整、短すぎるとPIDがうまく動かず、長すぎるとレスポンスが悪くなる
    osDelay(10);

    //---------------------------モタドラに出力---------------------------//
  }



}

