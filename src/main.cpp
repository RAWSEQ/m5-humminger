/*
 * M5 Humminger
 * https://pages.switch-science.com/letsiot/vibration/
 * https://ambidata.io/samples/m5stack/sound/
 */
#include <M5Core2.h>
#include <M5GFX.h>
#include "arduinoFFT.h"
#include <BLEMidi.h>

#define MIC 33

#define RHZ_B 520
#define RHZ_T 1350
#define RHZ_D 45
#define NOT_B 53
#define NOT_T 68

#define MON_H 70
#define MON_W 320
#define MON_DMAX 5.0

#define SAMPLING_FREQUENCY 40000
const uint16_t FFTsamples = 256; // サンプル数は2のべき乗

#define RC_AFF 0.9

double vReal[FFTsamples];
double vImag[FFTsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);

unsigned int sampling_period_us;
bool is_ble = false;
bool peak_on = false;
double rc_prev = 0;

int rchz = 0;
int n_nt = 0;
int d_nt = 0;

M5GFX disp;
M5Canvas c_hz(&disp);
M5Canvas c_mon(&disp);

Button btn_note(50, 60, 220, 110, false, "HUMMING", {BLACK, 0xC618, 0xC618}, {0x7BEF, WHITE, WHITE});

// FFT サンプリング
void sample(int nsamples)
{
  for (int i = 0; i < nsamples; i++)
  {
    unsigned long t = micros();
    vReal[i] = (double)analogRead(MIC) / 15000;
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us)
      ;
  }
}

// BLE 接続イベント
void onConnected()
{
  is_ble = true;
}

// BLE 切断イベント
void onDisconnected()
{
  is_ble = false;
}

// 画面描画
void drawChart(int nsamples)
{
  int band_width = floor(MON_W / nsamples);
  int band_pad = band_width - 1;
  c_mon.fillRect(0, 0, 320, 70, BLACK);
  for (int band = 0; band < nsamples; band++)
  {
    int hpos = band * band_width;
    float d = vReal[band];
    if (d > MON_DMAX)
      d = MON_DMAX;
    int h = (int)((d / MON_DMAX) * (MON_H));
    c_mon.fillRect(hpos, MON_H - h, band_pad, h, WHITE);
  }
}

void event_btn_note(Event &e)
{
  if (n_nt)
  {
    BLEMidiServer.noteOn(0, n_nt, 100);
    d_nt = n_nt;
  }
}

void event_btn_note_r(Event &e)
{
  if (d_nt)
  {
    BLEMidiServer.noteOff(0, d_nt, 100);
    d_nt = 0;
  }
}

// 初期実行
void setup()
{
  M5.begin();
  disp.begin();
  c_hz.setColorDepth(1);
  c_hz.createSprite(245, 50);
  c_hz.setFont(&fonts::Font7);
  c_hz.setTextWrap(false);
  c_hz.setTextSize(1);
  c_hz.printf("%05d", 0);
  c_mon.setColorDepth(1);
  c_mon.createSprite(320, 70);

  Serial.begin(115200);
  while (!Serial)
    ;
  M5.lcd.setBrightness(20);
  pinMode(MIC, INPUT);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  BLEMidiServer.begin("M5 Hamminger");
  BLEMidiServer.setOnConnectCallback(onConnected);
  BLEMidiServer.setOnDisconnectCallback(onDisconnected);

  btn_note.addHandler(event_btn_note, E_TOUCH);
  btn_note.addHandler(event_btn_note_r, E_RELEASE);
  btn_note.draw({BLACK, 0xC618, 0xC618});
}

// 直流成分除去
void DCRemoval(double *vData, uint16_t samples)
{
  double mean = 0;
  for (uint16_t i = 1; i < samples; i++)
  {
    mean += vData[i];
  }
  mean /= samples;
  for (uint16_t i = 1; i < samples; i++)
  {
    vData[i] -= mean;
  }
}

// RC計算
double calc_rc(double val)
{
  rc_prev = RC_AFF * rc_prev + (1 - RC_AFF) * val;
  return rc_prev;
}

void calc_note(int r_mp)
{
  // 計算
  int not_w = NOT_T - NOT_B;
  int rhz_w = RHZ_T - RHZ_B;
  // 範囲差分
  int def_v = rhz_w / not_w / 2 + RHZ_D;
  n_nt = (r_mp - RHZ_B + def_v) * not_w / rhz_w + NOT_B;
}

// クロック
void loop()
{
  M5.update();
  sample(FFTsamples);
  DCRemoval(vReal, FFTsamples);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  double r_mp = FFT.MajorPeak();
  drawChart(FFTsamples / 2);

  c_hz.setCursor(0, 0);
  if ((int)r_mp < 3000)
  {
    if (!peak_on)
    {
      rc_prev = r_mp;
      peak_on = true;
    }
    rchz = calc_rc(r_mp);
    c_hz.printf("%05d", rchz);
    calc_note(r_mp);
  }
  else
  {
    if (peak_on)
    {
      peak_on = false;
    }
    rchz = 0;
    n_nt = 0;
    c_hz.printf("%05d", 0);
  }

  c_mon.setCursor(0, 50);
  c_mon.printf("Peak: %.0f", r_mp);
  c_mon.printf(" NOTE: %0d", n_nt);

  if (is_ble)
  {
    c_mon.print(" [BLE]");
  }

  c_hz.pushSprite(80, 0);
  c_mon.pushSprite(0, 170);

  delay(10);
}