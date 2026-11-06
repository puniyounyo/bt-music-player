// ============================================================
// Bluetooth ミュージックプレイヤー (ESP32 A2DPシンク)
//   + ST7789 76x284 TFT (曲情報 + 90年代風イコライザ)
//   + シリアルデバッグ / シリアルコマンド
//
// ・ESP32単体（TFT・PCM5102・ボタン未接続）でも動作します
// ・シリアル(115200bps)に状況を出力し、コマンドで操作できます（'h'で一覧）
// ・ツール > Core Debug Level は「None」を推奨（BTのログで埋まるため）
// ・必要ライブラリ: ESP32-A2DP, arduino-audio-tools, LovyanGFX
// ============================================================
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

#define USE_TFT            1      // 0にするとTFT関連を完全に無効化
#define BT_NAME            "MyCarPlayer"
#define BT_AUTO_RECONNECT  1      // 0: 起動時に前回の機器へ自動接続しない
#define DEMO_AT_BOOT       0      // 1: 起動直後からデモ表示（スマホ不要で画面確認）

#if USE_TFT
#include <LovyanGFX.hpp>
#endif

// ---------------- ピン ----------------
constexpr int MY_I2S_BCK  = 26;   // PCM5102 BCK
constexpr int MY_I2S_WS   = 25;   // PCM5102 LCK
constexpr int MY_I2S_DATA = 22;   // PCM5102 DIN

constexpr int PIN_TFT_SCLK = 18;   // 基板: SCL
constexpr int PIN_TFT_MOSI = 23;   // 基板: SDA
constexpr int PIN_TFT_CS   = 5;
constexpr int PIN_TFT_DC   = 19;
constexpr int PIN_TFT_RST  = 4;
constexpr int PIN_TFT_BL   = 27;

// ボタン（GNDに落とす。内部プルアップ。未接続でも誤動作しません）
struct Btn { int pin; const char* name; bool stable; bool lastRaw; uint32_t changedAt; };
Btn btns[] = {
  {13, "PLAY/PAUSE", true, true, 0},
  {14, "NEXT",       true, true, 0},
  {16, "PREV",       true, true, 0},
  {17, "VOL+",       true, true, 0},
};

// ---------------- TFT調整値 ----------------
constexpr int  TFT_OFFSET_X = 82;
constexpr int  TFT_OFFSET_Y = 18;    // 右端がノイズなら増やす / 左端が欠けるなら減らす（'c'キーで確認）
constexpr int  TFT_ROT      = 1;      // 向きが逆なら3
constexpr bool TFT_INVERT   = false;  // 色が反転して見えたら切り替える（今回の基板はfalseが正）
constexpr bool TFT_BL_ACTIVE_LOW = true;   // BLがLowで点灯する基板ならtrue（Highで点灯ならfalse）

// ---------------- イコライザ調整値 ----------------
constexpr float EQ_DB_MIN  = -60.0f;  // これ以下は0
constexpr float EQ_DB_MAX  = -15.0f;  // これ以上は満タン
constexpr float EQ_TILT_DB = 1.5f;    // 高域ほど持ち上げる量(dB/バンド)
constexpr float SAMPLE_RATE = 44100.0f;

// ============================================================
// グローバル状態
// ============================================================
I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// BTスタック側のコールバックが更新する値
volatile bool btConnected   = false;
volatile bool avrcConnected = false;
volatile bool audioStarted  = false;

// 受信した曲情報（BTタスク -> loop への受け渡し用）
char pTitle[128] = "", pArtist[128] = "", pAlbum[128] = "";
uint32_t pDur = 0;
volatile bool metaDirty = false;

// 現在表示している曲情報（loopだけが触る）
char curTitle[128] = "", curArtist[128] = "", curAlbum[128] = "";
uint32_t curDur = 0;
bool textDirty = true;

// 再生位置（自前で時間を進める）
uint32_t posBaseMs = 0, posBaseAt = 0;
bool posRunning = false;

bool demoMode = false;
bool eqPrint  = false;

// 音声受信の統計
static uint32_t rxBytes = 0;
volatile uint32_t lastAudioMs = 0;

// ============================================================
// ユーティリティ
// ============================================================
void copyUtf8(char* dst, size_t cap, const char* src) {
  snprintf(dst, cap, "%s", src);
  if (strlen(src) >= cap) {              // 途中で切れたとき、UTF-8を壊さない
    size_t n = strlen(dst);
    while (n > 0 && (((uint8_t)dst[n - 1]) & 0xC0) == 0x80) n--;
    if (n > 0) n--;
    dst[n] = 0;
  }
}

void fmtTime(char* buf, size_t cap, uint32_t ms) {
  snprintf(buf, cap, "%d:%02d", (int)(ms / 60000), (int)((ms / 1000) % 60));
}

// ============================================================
// スペクトラム解析（Goertzel法、12バンド）
// ============================================================
constexpr int BANDS = 12;
constexpr int FFT_N = 1024;
static float monoBuf[FFT_N];
static float winTab[FFT_N];
static float coefTab[BANDS];
static float bandHz[BANDS];
static int   monoFill = 0;
volatile float bandLvl[BANDS] = {0};

void spectrumInit() {
  for (int i = 0; i < FFT_N; i++) winTab[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_N - 1)));
  for (int b = 0; b < BANDS; b++) {
    bandHz[b] = 60.0f * powf(14000.0f / 60.0f, (float)b / (BANDS - 1));   // 60Hz$301C14kHz(対数)
    coefTab[b] = 2.0f * cosf(2.0f * PI * bandHz[b] / SAMPLE_RATE);
  }
}

void spectrumCompute() {
  for (int i = 0; i < FFT_N; i++) monoBuf[i] *= winTab[i];
  for (int b = 0; b < BANDS; b++) {
    float c = coefTab[b], s1 = 0, s2 = 0;
    for (int i = 0; i < FFT_N; i++) {
      float s0 = monoBuf[i] + c * s1 - s2;
      s2 = s1;
      s1 = s0;
    }
    float power = s1 * s1 + s2 * s2 - c * s1 * s2;
    if (power < 0) power = 0;
    float amp = sqrtf(power) / (FFT_N / 4.0f);   // ハン窓の補正込み。満振幅=1.0
    float db = 20.0f * log10f(amp + 1e-9f) + b * EQ_TILT_DB;
    float v = (db - EQ_DB_MIN) / (EQ_DB_MAX - EQ_DB_MIN);
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    bandLvl[b] = v;
  }
}

// 音声データ(16bit/ステレオ)が来るたびに呼ばれる。BTタスク上なので軽く保つ
void onAudioData(const uint8_t* data, uint32_t len) {
  rxBytes += len;
  lastAudioMs = millis();
  const int16_t* s = (const int16_t*)data;
  uint32_t frames = len / 4;
  for (uint32_t i = 0; i < frames; i++) {
    monoBuf[monoFill++] = 0.5f * ((float)s[2 * i] + (float)s[2 * i + 1]) / 32768.0f;
    if (monoFill >= FFT_N) {
      spectrumCompute();
      monoFill = 0;
    }
  }
}

// ============================================================
// BTコールバック
// ============================================================
void onMetadata(uint8_t id, const uint8_t* text) {
  const char* s = (const char*)text;
  Serial.printf("[%lu] [AVRC] meta id=0x%02x : %s\n", millis(), id, s);
  portENTER_CRITICAL(&mux);
  switch (id) {
    case ESP_AVRC_MD_ATTR_TITLE:        // 曲名が来たら他の項目は一旦クリア
      copyUtf8(pTitle, sizeof(pTitle), s);
      pArtist[0] = 0; pAlbum[0] = 0; pDur = 0;
      break;
    case ESP_AVRC_MD_ATTR_ARTIST:       copyUtf8(pArtist, sizeof(pArtist), s); break;
    case ESP_AVRC_MD_ATTR_ALBUM:        copyUtf8(pAlbum, sizeof(pAlbum), s); break;
    case ESP_AVRC_MD_ATTR_PLAYING_TIME: pDur = strtoul(s, nullptr, 10); break;
    default: break;
  }
  metaDirty = true;
  portEXIT_CRITICAL(&mux);
}

void onConnectionState(esp_a2d_connection_state_t state, void*) {
  btConnected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
  Serial.printf("[%lu] [A2DP] connection: %s\n", millis(), a2dp_sink.to_str(state));
}

void onAudioState(esp_a2d_audio_state_t state, void*) {
  audioStarted = (state == ESP_A2D_AUDIO_STATE_STARTED);
  Serial.printf("[%lu] [A2DP] audio: %s\n", millis(), a2dp_sink.to_str(state));
}

void onAvrcConnection(bool connected) {
  avrcConnected = connected;
  Serial.printf("[%lu] [AVRC] connection: %s\n", millis(), connected ? "Connected" : "Disconnected");
}

// ============================================================
// 曲情報・再生位置の管理（loop側）
// ============================================================
void posStart(uint32_t now) { if (!posRunning) { posBaseAt = now; posRunning = true; } }
void posStop(uint32_t now)  { if (posRunning) { posBaseMs += now - posBaseAt; posRunning = false; } }
void posReset(uint32_t now) { posBaseMs = 0; posBaseAt = now; }
uint32_t currentPos(uint32_t now) { return posRunning ? posBaseMs + (now - posBaseAt) : posBaseMs; }

bool shownPlaying() { return demoMode ? true : audioStarted; }

void clearTrack(uint32_t now) {
  portENTER_CRITICAL(&mux);
  pTitle[0] = 0; pArtist[0] = 0; pAlbum[0] = 0; pDur = 0; metaDirty = false;
  portEXIT_CRITICAL(&mux);
  curTitle[0] = 0; curArtist[0] = 0; curAlbum[0] = 0; curDur = 0;
  posBaseMs = 0; posBaseAt = now;
  textDirty = true;
}

void applyMetadata(uint32_t now) {
  if (demoMode) return;
  char t[128], a[128], b[128];
  uint32_t d;
  portENTER_CRITICAL(&mux);
  if (!metaDirty) { portEXIT_CRITICAL(&mux); return; }
  metaDirty = false;
  strcpy(t, pTitle); strcpy(a, pArtist); strcpy(b, pAlbum); d = pDur;
  portEXIT_CRITICAL(&mux);

  bool newTrack = strcmp(t, curTitle) != 0;
  bool changed  = newTrack || strcmp(a, curArtist) || strcmp(b, curAlbum) || d != curDur;
  if (!changed) return;     // 再生再開時などの再送は無視（位置をリセットしない）

  strcpy(curTitle, t); strcpy(curArtist, a); strcpy(curAlbum, b); curDur = d;
  if (newTrack) {
    posReset(now);
    Serial.printf("[%lu] [TRACK] \"%s\" / %s / %s (%lu ms)\n", now, t, a, b, (unsigned long)d);
  }
  textDirty = true;
}

void trackConnectionChanges(uint32_t now) {
  static bool prevBt = false, prevAvrc = false;
  if (btConnected != prevBt) {
    prevBt = btConnected;
    if (!btConnected && !demoMode) clearTrack(now);
    textDirty = true;
  }
  if (avrcConnected != prevAvrc) {
    prevAvrc = avrcConnected;
    if (btConnected && !avrcConnected) Serial.println("[WARN] A2DPは接続中だがAVRCP未接続（曲情報・操作が効かない状態）");
  }
}

void syncPosition(uint32_t now) {
  bool sp = shownPlaying();
  if (sp && !posRunning) posStart(now);
  if (!sp && posRunning) posStop(now);
  if (demoMode && curDur > 0 && currentPos(now) >= curDur) posReset(now);   // デモは繰り返し
  if (!demoMode && curDur > 0 && currentPos(now) > curDur) {               // 範囲外は頭打ち
    posBaseMs = curDur; posBaseAt = now;
  }
}

// 画面に出す文字（曲情報が無いときは状態表示）
const char* titleText() {
  if (curTitle[0]) return curTitle;
  if (!btConnected) return "Bluetooth待機中  " BT_NAME;
  return "接続済み  再生待ち";
}
const char* artistText() {
  if (curTitle[0]) return curArtist;
  if (!btConnected) return "スマホから接続してください";
  return "";
}
const char* albumText() { return curTitle[0] ? curAlbum : ""; }

// ============================================================
// デモモード（スマホなしで表示を確認）
// ============================================================
float demoTarget(int b, uint32_t t) {
  float x = 0.5f + 0.5f * sinf(t * 0.0020f * (1.0f + b * 0.23f) + b * 1.7f);
  float y = 0.5f + 0.5f * sinf(t * 0.0071f + b * 0.9f);
  float v = (0.6f * x + 0.4f * y) * (1.05f - b * 0.035f);
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  return v;
}

void setDemo(bool on, uint32_t now) {
  demoMode = on;
  if (on) {
    copyUtf8(curTitle, sizeof(curTitle), "テスト曲 - 日本語の長いタイトルが横スクロールします / Long Title Scroll Test");
    copyUtf8(curArtist, sizeof(curArtist), "テストアーティスト");
    copyUtf8(curAlbum, sizeof(curAlbum), "テストアルバム");
    curDur = 225000;
    posReset(now);
    textDirty = true;
  } else {
    clearTrack(now);
    posStop(now);
  }
  Serial.printf("[DEMO] %s\n", on ? "ON (ダミーの曲情報とスペクトラム)" : "OFF");
}

// ============================================================
// TFT
// ============================================================
#if USE_TFT
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = SPI3_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = true;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_TFT_SCLK;
      cfg.pin_mosi    = PIN_TFT_MOSI;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = PIN_TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs          = PIN_TFT_CS;
      cfg.pin_rst         = PIN_TFT_RST;
      cfg.pin_busy        = -1;
      cfg.panel_width     = 76;
      cfg.panel_height    = 284;
      cfg.memory_width    = 240;
      cfg.memory_height   = 320;
      cfg.offset_x        = TFT_OFFSET_X;
      cfg.offset_y        = TFT_OFFSET_Y;
      cfg.offset_rotation = 0;
      cfg.readable        = false;     // 読み出さない＝未接続でも固まらない
      cfg.invert          = TFT_INVERT;
      cfg.rgb_order       = false;
      cfg.dlen_16bit      = false;
      cfg.bus_shared      = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

LGFX tft;
lgfx::LGFX_Sprite eqSpr(&tft);
lgfx::LGFX_Sprite titleSpr(&tft);

const uint16_t C_BG     = 0x0000;
const uint16_t C_GREEN  = lgfx::color565(0, 220, 60);
const uint16_t C_YELLOW = lgfx::color565(255, 200, 0);
const uint16_t C_RED    = lgfx::color565(255, 40, 40);
const uint16_t C_PEAK   = lgfx::color565(255, 255, 255);
const uint16_t C_TITLE  = lgfx::color565(255, 255, 255);
const uint16_t C_ARTIST = lgfx::color565(120, 200, 255);
const uint16_t C_ALBUM  = lgfx::color565(170, 170, 170);
const uint16_t C_BAR_BG = lgfx::color565(50, 50, 50);
const uint16_t C_BAR    = lgfx::color565(0, 200, 80);

constexpr int EQ_W = 76, EQ_H = 76;
constexpr int BAR_W = 5, BAR_GAP = 1, EQ_X0 = 2;
constexpr int SEG_H = 3, SEG_GAP = 1, SEGS = 19;   // 19 * 4 = 76

float    level[BANDS] = {0};
float    peakSeg[BANDS] = {0};
uint32_t peakHoldUntil[BANDS] = {0};

constexpr int TEXT_X = 80;
constexpr int TITLE_W = 204, TITLE_H = 22, TITLE_GAP = 40;
int  titleW = 0, scrollX = 0;
bool eqReady = false, titleReady = false, progressDirty = true;
uint32_t lastFrame = 0, frames = 0;
float fps = 0;

void updateEq(uint32_t now) {
  bool active = demoMode ? (((now / 1000) % 16) < 12)
                         : (audioStarted && (now - lastAudioMs < 400));
  for (int b = 0; b < BANDS; b++) {
    float target = active ? (demoMode ? demoTarget(b, now) : bandLvl[b]) : 0.0f;
    if (target > level[b]) level[b] += (target - level[b]) * 0.6f;   // 上昇は速く
    else                   level[b] -= 0.035f;                       // 下降はなめらか
    if (level[b] < 0) level[b] = 0;

    float segs = level[b] * SEGS;
    if (segs >= peakSeg[b]) {
      peakSeg[b] = segs;
      peakHoldUntil[b] = now + 400;
    } else if (now > peakHoldUntil[b]) {
      peakSeg[b] -= 0.12f;
      if (peakSeg[b] < 0) peakSeg[b] = 0;
    }
  }
}

uint16_t segColor(int s) { return s < 12 ? C_GREEN : (s < 16 ? C_YELLOW : C_RED); }

void drawEq() {
  if (!eqReady) return;
  eqSpr.fillScreen(C_BG);
  for (int b = 0; b < BANDS; b++) {
    int x = EQ_X0 + b * (BAR_W + BAR_GAP);
    int n = (int)(level[b] * SEGS + 0.5f);
    for (int s = 0; s < n && s < SEGS; s++) {
      eqSpr.fillRect(x, EQ_H - (s + 1) * (SEG_H + SEG_GAP), BAR_W, SEG_H, segColor(s));
    }
    int p = (int)peakSeg[b];
    if (p > 0 && p <= SEGS) eqSpr.fillRect(x, EQ_H - p * (SEG_H + SEG_GAP), BAR_W, SEG_H, C_PEAK);
  }
  eqSpr.pushSprite(0, 0);
}

void rebuildText() {
  textDirty = false;
  scrollX = 0;
  if (titleReady) {
    titleSpr.setFont(&lgfx::fonts::lgfxJapanGothic_20);
    titleW = titleSpr.textWidth(titleText());
  }
  tft.fillRect(TEXT_X, 24, 204, 52, C_BG);
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  tft.setTextColor(C_ARTIST, C_BG);
  tft.drawString(artistText(), TEXT_X, 25);
  tft.setTextColor(C_ALBUM, C_BG);
  tft.drawString(albumText(), TEXT_X, 43);
  progressDirty = true;
}

void updateTitle(uint32_t now) {
  if (!titleReady) return;
  static uint32_t holdUntil = 0;
  if (titleW > TITLE_W) {
    if (scrollX == 0 && holdUntil == 0) holdUntil = now + 1500;   // 先頭で一旦停止
    if (now >= holdUntil) {
      scrollX += 2;
      if (scrollX >= titleW + TITLE_GAP) { scrollX = 0; holdUntil = 0; }
    }
  } else {
    scrollX = 0; holdUntil = 0;
  }
  titleSpr.fillScreen(C_BG);
  titleSpr.setFont(&lgfx::fonts::lgfxJapanGothic_20);
  titleSpr.setTextColor(C_TITLE, C_BG);
  titleSpr.drawString(titleText(), -scrollX, 1);
  if (titleW > TITLE_W) titleSpr.drawString(titleText(), -scrollX + titleW + TITLE_GAP, 1);
  titleSpr.pushSprite(TEXT_X, 0);
}

void updateProgress(uint32_t now) {
  static int lastSec = -1;
  static bool lastPlaying = false;
  uint32_t pos = currentPos(now);
  int sec = (int)(pos / 1000);
  bool sp = shownPlaying();
  if (!progressDirty && sec == lastSec && sp == lastPlaying) return;   // 変化時のみ描画
  progressDirty = false; lastSec = sec; lastPlaying = sp;

  tft.fillRect(TEXT_X, 61, 204, 15, C_BG);
  if (sp) tft.fillTriangle(81, 64, 81, 74, 89, 69, C_GREEN);                      // 再生中 $25B6
  else { tft.fillRect(81, 64, 3, 10, C_ALBUM); tft.fillRect(86, 64, 3, 10, C_ALBUM); }  // 停止中 $2016

  const int bx = 96, by = 67, bw = 100, bh = 5;
  tft.fillRect(bx, by, bw, bh, C_BAR_BG);
  if (curDur > 0) {
    uint32_t p = pos > curDur ? curDur : pos;
    tft.fillRect(bx, by, (int)((uint64_t)bw * p / curDur), bh, C_BAR);
  }
  char a[12], b[12], buf[28];
  fmtTime(a, sizeof(a), pos);
  if (curDur > 0) { fmtTime(b, sizeof(b), curDur); snprintf(buf, sizeof(buf), "%s / %s", a, b); }
  else            { snprintf(buf, sizeof(buf), "%s", a); }
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_12);
  tft.setTextColor(C_ALBUM, C_BG);
  tft.drawString(buf, 200, 63);
}

void drawCalibration() {
  int w = tft.width(), h = tft.height();
  tft.fillScreen(C_BG);
  tft.drawRect(0, 0, w, h, C_RED);
  tft.fillRect(0, 0, 8, 8, lgfx::color565(255, 0, 0));
  tft.fillRect(w - 8, 0, 8, 8, lgfx::color565(0, 255, 0));
  tft.fillRect(0, h - 8, 8, 8, lgfx::color565(0, 0, 255));
  tft.fillRect(w - 8, h - 8, 8, 8, lgfx::color565(255, 255, 255));
  char buf[32];
  snprintf(buf, sizeof(buf), "%dx%d off %d,%d", w, h, TFT_OFFSET_X, TFT_OFFSET_Y);
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  tft.setTextColor(C_TITLE, C_BG);
  tft.drawString(buf, (w - tft.textWidth(buf)) / 2, h / 2 - 8);
}

void tftSetup() {
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, TFT_BL_ACTIVE_LOW ? LOW : HIGH);   // バックライト点灯
  Serial.printf("[TFT] backlight: GPIO%d = %s (active-%s)\n", PIN_TFT_BL,
                TFT_BL_ACTIVE_LOW ? "LOW" : "HIGH", TFT_BL_ACTIVE_LOW ? "low" : "high");
  tft.init();
  tft.setRotation(TFT_ROT);
  Serial.printf("[TFT] init done: %d x %d (offset %d,%d)  ※接続の有無は検出不可（未接続でもエラーにならない）\n",
                tft.width(), tft.height(), TFT_OFFSET_X, TFT_OFFSET_Y);
  drawCalibration();
  delay(1500);
  tft.fillScreen(C_BG);

  eqSpr.setColorDepth(16);
  eqReady = (eqSpr.createSprite(EQ_W, EQ_H) != nullptr);
  titleSpr.setColorDepth(16);
  titleReady = (titleSpr.createSprite(TITLE_W, TITLE_H) != nullptr);
  Serial.printf("[TFT] sprites: eq=%s title=%s\n", eqReady ? "OK" : "FAILED", titleReady ? "OK" : "FAILED");
}
#endif  // USE_TFT

// ============================================================
// 操作（ボタン / シリアルコマンド）
// ============================================================
void doPlayPause() {
  if (audioStarted) { Serial.println("[CMD] pause"); a2dp_sink.pause(); }
  else              { Serial.println("[CMD] play");  a2dp_sink.play();  }
}
void doNext()  { Serial.println("[CMD] next");     a2dp_sink.next(); }
void doPrev()  { Serial.println("[CMD] previous"); a2dp_sink.previous(); }
void doVolume(int delta) {
  int v = (int)a2dp_sink.get_volume() + delta;
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  a2dp_sink.set_volume((uint8_t)v);
  Serial.printf("[CMD] volume %d/127\n", v);
}

void printStatus(uint32_t now) {
  char a[12], b[12];
  fmtTime(a, sizeof(a), currentPos(now));
  fmtTime(b, sizeof(b), curDur);
  Serial.printf("[STAT] t=%lus BT=%s AVRC=%s audio=%s vol=%d pos=%s/%s rx=%lukB heap=%u(min %u) fps=%.1f%s\n",
                now / 1000,
                btConnected ? "conn" : "disc", avrcConnected ? "conn" : "disc",
                audioStarted ? "playing" : "stopped", (int)a2dp_sink.get_volume(),
                a, b, (unsigned long)(rxBytes / 1024),
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), fps,
                demoMode ? " [DEMO]" : "");
  Serial.printf("[STAT] title=\"%s\" artist=\"%s\" album=\"%s\"\n", curTitle, curArtist, curAlbum);
  if (eqPrint) {
    Serial.print("[EQ]  ");
    for (int i = 0; i < BANDS; i++) Serial.printf(" %3d", (int)(bandLvl[i] * 100));
    Serial.println();
  }
}

void printHelp() {
  Serial.println("---- シリアルコマンド ----");
  Serial.println(" space : 再生/一時停止   n : 次の曲   p : 前の曲");
  Serial.println(" + / - : 音量 上げ/下げ  s : 状態表示   e : EQ数値の表示 切替");
  Serial.println(" d     : デモ表示 切替（スマホ不要でTFTを確認）");
  Serial.println(" c     : TFTの表示位置確認パターンを4秒表示");
  Serial.println(" h     : このヘルプ");
  Serial.println("--------------------------");
}

void handleSerial(uint32_t now) {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case ' ': doPlayPause(); break;
      case 'n': doNext(); break;
      case 'p': doPrev(); break;
      case '+': doVolume(+8); break;
      case '-': doVolume(-8); break;
      case 's': printStatus(now); break;
      case 'e': eqPrint = !eqPrint; Serial.printf("[EQ] 数値表示 %s\n", eqPrint ? "ON" : "OFF"); break;
      case 'd': setDemo(!demoMode, now); break;
      case 'c':
#if USE_TFT
        Serial.println("[TFT] 表示位置確認パターンを4秒表示");
        drawCalibration();
        delay(4000);
        tft.fillScreen(C_BG);
        textDirty = true;
#endif
        break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

void handleButtons(uint32_t now) {
  for (int i = 0; i < 4; i++) {
    Btn& b = btns[i];
    bool raw = digitalRead(b.pin);
    if (raw != b.lastRaw) { b.lastRaw = raw; b.changedAt = now; }
    if (now - b.changedAt > 25 && raw != b.stable) {
      b.stable = raw;
      if (!raw) {
        Serial.printf("[BTN] %s\n", b.name);
        switch (i) {
          case 0: doPlayPause(); break;
          case 1: doNext(); break;
          case 2: doPrev(); break;
          case 3: doVolume(+8); break;
        }
      }
    }
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== BT Music Player (TFT) boot ===");

  for (auto& b : btns) pinMode(b.pin, INPUT_PULLUP);

  spectrumInit();
  Serial.print("[EQ] band Hz:");
  for (int i = 0; i < BANDS; i++) Serial.printf(" %d", (int)bandHz[i]);
  Serial.println();

#if USE_TFT
  tftSetup();   // BT開始の前に画面メモリを確保（ヒープの断片化を避ける）
#else
  Serial.println("[TFT] USE_TFT=0 : 表示は無効");
#endif

  auto cfg = i2s.defaultConfig();
  cfg.pin_bck  = MY_I2S_BCK;
  cfg.pin_ws   = MY_I2S_WS;
  cfg.pin_data = MY_I2S_DATA;
  i2s.begin(cfg);

  a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
      ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME);
  a2dp_sink.set_avrc_metadata_callback(onMetadata);
  a2dp_sink.set_on_connection_state_changed(onConnectionState);
  a2dp_sink.set_on_audio_state_changed(onAudioState);
  a2dp_sink.set_avrc_connection_state_callback(onAvrcConnection);
  a2dp_sink.set_stream_reader(onAudioData, true);   // 音は通常どおり出力しつつ、解析にも回す

  a2dp_sink.start(BT_NAME, BT_AUTO_RECONNECT);
  Serial.printf("[BT] started as \"%s\" (auto reconnect %s)\n", BT_NAME, BT_AUTO_RECONNECT ? "ON" : "OFF");
  Serial.printf("[SYS] free heap after start: %u bytes\n", (unsigned)ESP.getFreeHeap());
  printHelp();

#if DEMO_AT_BOOT
  setDemo(true, millis());
#endif
}

void loop() {
  uint32_t now = millis();
  handleSerial(now);
  handleButtons(now);
  applyMetadata(now);
  trackConnectionChanges(now);
  syncPosition(now);

#if USE_TFT
  if (now - lastFrame >= 33) {       // 約30fps
    static uint32_t fpsAt = 0;
    lastFrame = now;
    if (textDirty) rebuildText();
    updateEq(now);
    drawEq();
    updateTitle(now);
    updateProgress(now);
    frames++;
    if (now - fpsAt >= 2000) { fps = frames * 1000.0f / (now - fpsAt); frames = 0; fpsAt = now; }
  }
#else
  textDirty = false;
#endif

  static uint32_t lastStat = 0;
  if (now - lastStat >= 5000) { lastStat = now; printStatus(now); }

  delay(1);
}
