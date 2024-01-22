/*
* ats_bell_ringer
* フォトリフレクタを踏んだら一定時間ATSのベルとチャイムを鳴動させる制御装置
* 2024.2.1 7M4MON
*/

#define NUM_SENS 2      // センサーの数

/* ピンの定義 */
#define PIN_BELL 8
#define PIN_CHIME 9
#define PIN_SENS_0 A1
#define PIN_SENS_1 A2
#define PIN_THRES_0 A0
#define PIN_THRES_1 A3
#define PIN_PH_LED_0 10
#define PIN_PH_LED_1 11

/* 時間関連の定義 */
#define POLLING_INTERVAL_MS   100
#define RINGING_BELL_MS      1000
#define RINGING_CHIME_MS     3800

#include <MsTimer2.h>

// センサーは2個ですが配列にしておいた。構造体にした方がスッキリするかも。
int16_t last_sens[NUM_SENS] = {0,0};
int16_t history_sens[NUM_SENS][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int16_t avg_sens[NUM_SENS] ;    // get_average() で計算した値を格納する
int16_t threshold[NUM_SENS];    // 毎回、半固定ボリュームから読み込む
uint8_t pin_sens[NUM_SENS] = {PIN_SENS_0, PIN_SENS_1};
uint8_t pin_thres[NUM_SENS] = {PIN_THRES_0, PIN_THRES_1};
uint8_t pin_ph_led[NUM_SENS] = {PIN_PH_LED_0, PIN_PH_LED_1};

int8_t ring_idx = 0;
bool start_bell_flag = false;

// リングバッファ[8個]の平均を返す
int16_t get_average( int16_t *ary ){
    int16_t avg = 0;
    for(uint8_t i = 0; i<8; i++){
        avg += *ary++;
    }
    avg >>= 3;  // 8個なので1/8する。
    return avg;
}

// 列車がフォトカプラを踏んだかどうかを検出する
bool detect_train(uint8_t n){
    history_sens[n][ring_idx] = last_sens[n];
    avg_sens[n] = get_average(history_sens[n]);
    digitalWrite(pin_ph_led[n], HIGH);
    delay(5);
    last_sens[n] = analogRead(pin_sens[n]);
    threshold[n] = analogRead(pin_thres[n]) >> 2;
    digitalWrite(pin_ph_led[n], LOW);
    return (avg_sens[n] - last_sens[n] > threshold[n] ) ?  true : false;
}

// 現在の状態をシリアルで通知する（デバッグ用）
// ちなみに、列車なし 970、列車あり 850 くらいなので、しきい値は 60 くらいで良さそう。
void print_sens_status(uint8_t n){
    Serial.print(n);
    Serial.print(", av:");
    Serial.print(avg_sens[n]);
    Serial.print(", th:");
    Serial.print(threshold[n]);
    Serial.print(", ls:");
    Serial.println(last_sens[n]);
}

// タイマ割り込みで呼び出されてセンサーの読み込みを行う。
void interrpt_proc(){
    bool detect = false;
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint8_t n = 0; n < NUM_SENS; n++){
        detect |= detect_train(n);
    }
    if (detect) start_bell_flag = true;     // falseに戻すのはメインループ
    digitalWrite(LED_BUILTIN, detect);

    ring_idx++;
    ring_idx &= 0b111;  // 0 - 7 の範囲に収める
    
    if (ring_idx < NUM_SENS){
        print_sens_status(ring_idx);    // ８回に１回、現在のステータスを通知する(分散処理)
    }

}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BELL, OUTPUT);
    pinMode(PIN_CHIME, OUTPUT);
    digitalWrite(PIN_BELL, HIGH);       // active low
    digitalWrite(PIN_CHIME, HIGH);      // active low
    pinMode(PIN_PH_LED_0, OUTPUT);
    pinMode(PIN_PH_LED_1, OUTPUT);
    digitalWrite(PIN_PH_LED_0, LOW);
    digitalWrite(PIN_PH_LED_1, LOW);
    Serial.begin(115200);
    //タイマ割り込みでセンサを読み取り
    MsTimer2::set(POLLING_INTERVAL_MS, interrpt_proc); // 
    MsTimer2::start();
}

void loop() {
    if (start_bell_flag){
        digitalWrite(PIN_BELL, LOW);
        digitalWrite(PIN_CHIME, LOW);
        delay(RINGING_BELL_MS);
        digitalWrite(PIN_BELL, HIGH);
        delay(RINGING_CHIME_MS);
        digitalWrite(PIN_CHIME, HIGH);
        start_bell_flag = false;
    }
}
