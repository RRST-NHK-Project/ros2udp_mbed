/*
RRST NHK2025
IPアドレスは適宜変更すること
垂直MD基板用にピンを変更
エンコーダーから計算した変位と速度をUDPで送信する
2025/02/05
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

#define PI 3.141592653589793

//---------------------------QEI---------------------------//
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X4_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X4_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X4_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X4_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X4_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/

void receive(UDPSocket *receiver);

// マッピング関数
int map(int value, int inMin, int inMax, int outMin, int outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

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

/*
//電磁弁(サーボから借りてる)
DigitalOut SV1(PB_1);
DigitalOut SV2(PB_6);
DigitalOut SV3(PD_13);
DigitalOut SV4(PD_12);
*/

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

// グローバル変数の定義
float Pulse[6]; // エンコーダーのパルス格納用
float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[mm/s]
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 変位[m]

float period = 10; // 制御周期[ms]
float R = 80;      // オムニ直径[mm]
int PPRx4 = 8192;  // エンコーダーのResolution

double mdd[9]; // MDに出力する方向指令を格納
double mdp[9]; // MDに出力するduty比を格納

const char *recievefromIP = nullptr; //ネットワーク切断検知用

int main() {
  // 送信データ
  char sendData[32];

  // PWM周波数の設定
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

  // ネットワーク設定
  // 送信先のIPアドレスとポート
  const char *destinationIP = "192.168.8.215";
  const uint16_t destinationPort = 4000;

  // 自機のIPアドレスとポート
  const char *myIP = "192.168.128.215";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス
  SocketAddress destination, source, myData;
  // UDP通信関係のクラス
  UDPSocket udp;
  // 受信用スレッド
  Thread receiveThread;

  /* マイコンのネットワーク設定 */
  // DHCPはオフにする（静的にIPなどを設定するため）
  net.set_dhcp(false);
  // IPなど設定
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error >_<\n");
    return -1;
  } else {
    printf("Network connection success ^_^\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  // メインループ（送信用）
  while (1) {
    using namespace std::chrono;

    // エンコーダーの値を取得
    // Pulse[1] = float(ENC1.getPulses());
    // Pulse[2] = float(ENC2.getPulses());
    // Pulse[2] = float(ENC3.getPulses());
    // Pulse[2] = float(ENC4.getPulses());

    Pulse[1] = 1;
    Pulse[2] = 2;
    Pulse[3] = 3;
    Pulse[4] = 4;

    v[1] = Pulse[1] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[2] = Pulse[2] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[3] = Pulse[3] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算
    v[4] = Pulse[4] * (R * PI / PPRx4) *
           (1000 / period); // エンコーダーのパルスから速度[mm/s]を計算

    d[1] += Pulse[1] * R * PI / PPRx4 / 1000; //変位[m]
    d[2] += Pulse[2] * R * PI / PPRx4 / 1000; //変位[m]
    d[3] += Pulse[3] * R * PI / PPRx4 / 1000; //変位[m]
    d[4] += Pulse[4] * R * PI / PPRx4 / 1000; //変位[m]

    // エンコーダーをリセット
    ENC1.reset();
    ENC2.reset();
    ENC3.reset();
    ENC4.reset();

    // 速度データをカンマ区切りの文字列に変換
    char sendData[128]; // 送信データを格納する配列
    snprintf(sendData, sizeof(sendData), "%f,%f,%f,%f,%f,%f,%f,%f,", v[1], v[2],
             v[3], v[4], d[1], d[2], d[3], d[4]);

    // 送信データを表示（デバッグ用）
    //printf("Sending (%d bytes): %s\n", strlen(sendData), sendData);

    // UDP送信
    if (const int result =
            udp.sendto(destination, sendData, strlen(sendData)) < 0) {
      printf("send Error: %d\n", result); // エラー処理
    }

    ThisThread::sleep_for(period); // 制御周期に合わせて待機
  }

  // スレッドの終了を待つ
  receiveThread.join();

  // UDPソケットを閉じ、ネットワーク接続を切断
  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) { // UDP受信スレッド

  using namespace std::chrono;

  SocketAddress source;
  char buffer[64];

  int data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  while (1) {
    memset(buffer, 0, sizeof(buffer));
    if (const int result =
            receiver->recvfrom(&source, buffer, sizeof(buffer)) < 0) {
      printf("Receive Error : %d", result);
    } else {

      recievefromIP = source.get_ip_address();
      // printf("%s\n",recievefromIP);

      //以下受信した文字列をカンマ区切りでintに変換
      char *ptr;
      int ptr_counter = 1;
      // カンマを区切りに文字列を分割
      // 1回目
      ptr = strtok(buffer, ",");
      data[1] = atoi(ptr); // intに変換

      // 2回目以降
      while (ptr != NULL) {
        ptr_counter++;
        // strtok関数により変更されたNULLのポインタが先頭
        ptr = strtok(NULL, ",");
        data[ptr_counter] = atoi(ptr); // intに変換

        // ptrがNULLの場合エラーが発生するので対処
        if (ptr != NULL) {
          // printf("%s\n", ptr);
        }
      }
      //ここまで

      // printf("%d\n",data[7]);

      //方向成分と速度成分を分離
      for (int i = 1; i <= 8; i++) {
        if (data[i] >= 0) {
          mdd[i] = 1;
        } else {
          mdd[i] = 0;
        }
        mdp[i] = fabs((data[i]) / 100.0);
      }
    }

    // int servo_pulse = map(data[7], 0, 270, 500, 2500);
    SERVO1.pulsewidth_us(map(data[5], 0, 270, 500, 2500));
    SERVO2.pulsewidth_us(map(data[6], 0, 270, 500, 2500));
    SERVO3.pulsewidth_us(map(data[7], 0, 270, 500, 2500));
    SERVO4.pulsewidth_us(map(data[8], 0, 270, 500, 2500));
    // printf("%d\n", data[1]);

    // printf("%f\n",mdd[6]);
    // printf("%lf, %lf, %lf, %lf\n", mdp[1], mdp[2], mdp[3], mdp[4]);
    // printf("%d, %d, %d, %d, %d, %d,, %d, %d\n", data[1], data[2], data[3],
    //        data[4], data[5], data[6], data[7], data[8]);

    // MDに出力

    MD1D = mdd[1];
    MD2D = mdd[2];
    MD3D = mdd[3];
    MD4D = mdd[4];
    // MD5D = mdd[3];
    // MD6D = mdd[4];

    MD1P = mdp[1];
    MD2P = mdp[2];
    MD3P = mdp[3];
    MD4P = mdp[4];
    // MD5P = mdp[3];
    // MD6P = mdp[4];
  }
}
