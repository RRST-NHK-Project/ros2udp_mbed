/*
NHK2025
エンコーダーの値から速度[mm/s]を計算しUDPでROS2に投げる
2024/10/18
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include <cstdio>  // snprintf
#include <cstring> // strlen

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
//---------------------------QEI---------------------------//

using ThisThread::sleep_for;
Ticker ticker;

//---------------------------For PID---------------------------//
int Pulse[7];
int last_Pulse[7];
float v[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //速度の格納[m/s]
float RPM[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

float period = 0.01; // 制御周期
float R = 0.05;     //オムニ直径
int PPR = 2048;     //エンコーダーのResolution

int main() {

  //---------------------------UDP Settings---------------------------//

  // 送信先情報(F7)
  const char *destinationIP = "192.168.8.195";
  const uint16_t destinationPort = 4000;

  // 自機情報
  const char *myIP = "192.168.8.216";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス（構造体でいいのでは？）
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
    printf("Network connection Error (>_<)\n");
    return -1;
  } else {
    printf("Network connection success (^_^)\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 送信データ
  char sendData[32];

  //---------------------------UDP Settings---------------------------//

  while (1) {

    using namespace std::chrono;
    
    Pulse[1] = ENC1.getPulses();
    Pulse[2] = ENC2.getPulses();
    Pulse[3] = ENC3.getPulses();
    Pulse[4] = ENC4.getPulses();
    Pulse[5] = ENC5.getPulses();
    Pulse[6] = ENC6.getPulses();

    for (int i = 1; i <= 6; i++) {
        v[i] = Pulse[i] * R * PI * 1.0 / period / PPR * 1000;
    }

    ENC1.reset();
    ENC2.reset();
    ENC3.reset();
    ENC4.reset();
    ENC5.reset();
    ENC6.reset();
    

    //sendData[0] = '\0'; // 文字列を初期化

    for (int i = 0; i < 7; i++) {
      char temp[32]; // 一時的なバッファ

      // RPM値を文字列に変換
      sprintf(temp, "%f", v[i]);

      if (i == 0) {
        strcpy(sendData, temp);
      } else {
        strcat(sendData, ",");
        strcat(sendData, temp);
      }
    }

    //printf("%f\n",v[1]);

    if (const int result =
            udp.sendto(destination, sendData, sizeof(sendData)) < 0) {
      printf("send Error: %d\n", result);
    }
    ThisThread::sleep_for(period);
  }

  udp.close();
  net.disconnect();
}
