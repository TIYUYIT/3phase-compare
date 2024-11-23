#include <math.h>
#include <soc/gpio_struct.h>
// 200.1Hzまで
#define DPHI_SIZE 2010  // 正弦波の周波数*10の値。速度に応じたsinの位相の変化量を格納する
#define FREQ_SIZE 3000
float speed_max = 200;

const uint32_t PHASE_FULL = pow(2, 31);  // 位相の最大値
const uint32_t PHASE_SIZE = pow(2, 11);  // 正弦波の解像度
// 正弦波pwm用のピン、制御ピン
const uint8_t Pin_UH = 25, Pin_UL = 19, Pin_VH = 26, Pin_VL = 18, Pin_WH = 27, Pin_WL = 5, Pin_ctl = 32;

float triValue[PHASE_SIZE];      // 比較用三角波配列(非同期モードで使用)
float sinValuesU_H[PHASE_SIZE];  // 三相の正弦波用配列の宣言
float sinValuesV_H[PHASE_SIZE];
float sinValuesW_H[PHASE_SIZE];

int randomValue[PHASE_SIZE];

hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
const uint32_t timer_clk1 = 400000;  // 割り込み用タイマー1の周波数
int interruptFreq = 20000;           // 割り込み周波数

int sin_max = 512, tri_max = 2048;
volatile int sync_mode = 0;  // 回転モード-0:非同期、1 : 1パルス、3 : 3パルス、5 : 5パルス、9 : 9パルスなど
volatile float speed = 0;    // 正弦波の周波数
volatile uint8_t accel = 0;
volatile int tri_freq = 400;
volatile uint32_t sin_phase = 0;               // 正弦波の位相(V_H) 1 ~ 2^31
volatile uint32_t tri_phase = 0;               // 三角波の位相
int dPhi_sin[DPHI_SIZE], dPhi_tri[FREQ_SIZE];  // 位相の変化量

volatile boolean isStop = true; // 使ってない


// 割り込み関数
void IRAM_ATTR onTimer1() {
  if (speed > 0.01) {
    // 位相進める。
    sin_phase = sin_phase + dPhi_sin[int(speed * 10)];
    sin_phase = sin_phase & (PHASE_FULL - 1);  // 剰余の計算
    tri_phase = tri_phase + dPhi_tri[tri_freq];
    tri_phase = tri_phase & (PHASE_FULL - 1);

    // モードの判定、三角波の位相上書き
    if (sync_mode == 1) { // 1パルスだけ流れる電流を減らすために比較する三角波をひっくり返していません。
      tri_phase = (sin_phase * sync_mode) & (PHASE_FULL - 1);
      // GPIO操作
      SwitchGPIO((float)sinValuesU_H[sin_phase >> 20] * sin_max, (float)triValue[tri_phase >> 20] * tri_max, Pin_UH);
      SwitchGPIO((float)sinValuesV_H[sin_phase >> 20] * sin_max, (float)triValue[tri_phase >> 20] * tri_max, Pin_VH);
      SwitchGPIO((float)sinValuesW_H[sin_phase >> 20] * sin_max, (float)triValue[tri_phase >> 20] * tri_max, Pin_WH);
    } else if (sync_mode != 0) { // 同期モード
      tri_phase = (sin_phase * sync_mode) & (PHASE_FULL - 1);
      // GPIO操作
      SwitchGPIO((float)sinValuesU_H[sin_phase >> 20] * sin_max, (float)-triValue[tri_phase >> 20] * tri_max, Pin_UH);
      SwitchGPIO((float)sinValuesV_H[sin_phase >> 20] * sin_max, (float)-triValue[tri_phase >> 20] * tri_max, Pin_VH);
      SwitchGPIO((float)sinValuesW_H[sin_phase >> 20] * sin_max, (float)-triValue[tri_phase >> 20] * tri_max, Pin_WH);
    } else { // 非同期モード
      // GPIO操作
      SwitchGPIO((float)sinValuesU_H[sin_phase >> 20] * sin_max + randomValue[sin_phase >> 20], (float)-triValue[tri_phase >> 20] * tri_max, Pin_UH);
      SwitchGPIO((float)sinValuesV_H[sin_phase >> 20] * sin_max + randomValue[sin_phase >> 20], (float)-triValue[tri_phase >> 20] * tri_max, Pin_VH);
      SwitchGPIO((float)sinValuesW_H[sin_phase >> 20] * sin_max + randomValue[sin_phase >> 20], (float)-triValue[tri_phase >> 20] * tri_max, Pin_WH);
    }
  } else {
    StopPWM();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Pin_UH, OUTPUT);
  pinMode(Pin_UL, OUTPUT);
  pinMode(Pin_VH, OUTPUT);
  pinMode(Pin_VL, OUTPUT);
  pinMode(Pin_WH, OUTPUT);
  pinMode(Pin_WL, OUTPUT);
  pinMode(Pin_ctl, INPUT);
  // 三角波の値の配列 -0.5 ~ 0.5
  for (int i = 0; i < PHASE_SIZE; i++) {
    float valuefortri = (float)(2.0 / PHASE_SIZE) * i;
    if (valuefortri >= 1.5) {
      valuefortri = (float)valuefortri - 2.0;
    } else if (valuefortri > 0.5 && valuefortri < 1.5) {
      valuefortri = (float)1.0 - valuefortri;
    }
    triValue[i] = (float)valuefortri;
    randomValue[i] = random(-10, 10);
  }

  // 3相のsinの値用の配列の作成 // -0.5 ~ 0.5
  for (int x = 0; x < PHASE_SIZE; x++) {
    float s = (float)(360.0 * x) / PHASE_SIZE;
    float valueU_H = (sin((s - 120.0) * DEG_TO_RAD) / 2.0);  // 正弦波
    float valueV_H = (sin((s)*DEG_TO_RAD) / 2.0);
    float valueW_H = (sin((s + 120.0) * DEG_TO_RAD) / 2.0);
    sinValuesU_H[x] = valueU_H;
    sinValuesV_H[x] = valueV_H;
    sinValuesW_H[x] = valueW_H;
  }

  for (int x = 0; x < DPHI_SIZE; x++) {
    dPhi_sin[x] = (float)((x / 10.0) * PHASE_FULL) / interruptFreq;
  }

  for (int x = 0; x < FREQ_SIZE; x++) {
    dPhi_tri[x] = x * (float)(PHASE_FULL / interruptFreq);
    //Serial.println(dPhi_tri[x]);
  }
  Serial.println("Setup finished");

  // タイマーの登録
  timer1 = timerBegin(timer_clk1);         // 使用するタイマー周波数
  timerAttachInterrupt(timer1, onTimer1);  //割り込み関数の登録
  timerAlarm(timer1, timer_clk1 / interruptFreq, true, 0);
  ChangeFreq(400);
}

void loop() {  //あとはcmdの実装だけ
  int cmd = analogRead(Pin_ctl);
  if (speed >= -0.09) {
    if (cmd >= 1500 && cmd <= 2500) { delay(40); }  // 定速
    else if (cmd < 500 && speed < speed_max) {     // 力行
      speed += 0.1;
      accel = 3;
      //Serial.println("3+");
      delay(20);
      isStop = false;
    } else if (cmd < 1000 && speed < speed_max) {
      speed += 0.1;
      accel = 2;
      //Serial.println("2+");
      delay(40);
    } else if (cmd < 1500 && speed < speed_max) {
      if (speed == 0) {
        speed = 2;
      }
      speed += 0.1;
      accel = 1;
      //Serial.println("1+");
      delay(60);
    }

    else if (cmd > 3500 && speed > 0.01) {
      speed -= 0.1;
      accel = -3;
      //Serial.println("3-");
      delay(20);
    }  // 減速
    else if (cmd > 3000 && speed > 0.01) {
      speed -= 0.1;
      accel = -2;
      //Serial.println("2-");
      delay(40);
    } else if (cmd > 2500 && speed > 0.01) {
      speed -= 0.1;
      accel = -1;
      //Serial.println("1-");
      delay(60);
    } else {
      accel = 0;
      delay(40);
    }

    if (speed < speed_max) { JR209(speed); }
    else {
      speed = speed_max;
    }  // 速度の矯正
  }

  if (speed <= 0.001) {
    speed = 0;
    isStop = true;
  }  // // 速度の矯正
  else if (speed <= speed_max) {
    isStop = false;
  }  //speed!=0
}

// 高速なGPIO操作をしたい
void SwitchGPIO(float sinV, float triV, int Pin) {
  if (sinV > triV) {
    GPIO.out_w1ts = 1 << Pin;
  } else {
    GPIO.out_w1tc = 1 << Pin;
  }
}

void ChangeFreq(int freq) {
  tri_freq = freq;
}

void StopPWM() {
  GPIO.out_w1tc = 1 << Pin_UH;
  GPIO.out_w1tc = 1 << Pin_UL;
  GPIO.out_w1tc = 1 << Pin_VH;
  GPIO.out_w1tc = 1 << Pin_VL;
  GPIO.out_w1tc = 1 << Pin_WH;
  GPIO.out_w1tc = 1 << Pin_WL;
}

void JR207(float speed) {
  sin_max = 1024 + speed * 20; // V/f一定ではないですが...
  if (speed > 0.1 && speed <= 10) {
    sync_mode = 0;
    ChangeFreq(365);
  } else if (speed > 10 && speed <= 30) {
    sync_mode = 15;
  } else if (speed > 30 && speed <= 43) {
    sync_mode = 9;
  } else if (speed > 43 && speed <= 52) {
    sync_mode = 5;
  } else if (speed > 52 && speed <= 60) {
    sync_mode = 3;
  } else if (speed > 60) {
    sync_mode = 1;
  }
}
