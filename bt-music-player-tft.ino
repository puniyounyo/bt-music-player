// ============================================================
// Bluetooth ミュージックプレイヤー (ESP32 A2DPシンク)
//   + ST7789 76x284 TFT (曲情報 + 90年代カーオーディオ風スペアナ)
//   + ロータリーエンコーダ(押し込み付き) + NEXT/PREVボタン
//   + シリアルデバッグ / シリアルコマンド
//
// ・ESP32単体（TFT・PCM5102・操作系 未接続）でも動作します
// ・シリアル(115200bps)に状況を出力し、コマンドで操作できます（'h'で一覧）
// ・ツール > Core Debug Level は「None」を推奨（BTのログで埋まるため）
// ・必要ライブラリ: ESP32-A2DP, arduino-audio-tools, LovyanGFX
//
// 【操作】
//   ノブを回す          : 音量（変更中は前面にボリュームゲージを表示）
//   ノブ短押し          : 再生 / 一時停止
//   ノブ押しながら回す  : 次の曲 / 前の曲
//   ノブ長押し(0.6秒$301C) : 表示形式の切替（1:標準 / 2:タイトル+再生位置 / 3:全面スペアナ）
//   ノブ超長押し(1.5秒$301C): スペアナのデザイン切替（離したときに切り替わる）
//   NEXT 短押し/長押し  : 次の曲 / 早送り
//   PREV 短押し/長押し  : 前の曲 / 巻き戻し
//   NEXT+PREV 同時押し  : 画面の明るさ切替（前面に明るさゲージを表示）
// ============================================================
#include "driver/gpio.h"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

struct Scroller {
  char last[128];
  int w;
  int x;
  uint32_t holdUntil;
};

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

constexpr int PIN_TFT_SCLK = 18;  // 基板: SCL
constexpr int PIN_TFT_MOSI = 23;  // 基板: SDA
constexpr int PIN_TFT_CS   = 5;
constexpr int PIN_TFT_DC   = 19;
constexpr int PIN_TFT_RST  = 4;
constexpr int PIN_TFT_BL   = 27;

// 操作系（すべてGNDに落とす方式。内部プルアップを使用）
constexpr int MY_ENC_A    = 13;   // エンコーダ CLK
constexpr int MY_ENC_B    = 14;   // エンコーダ DT
constexpr int MY_ENC_SW   = 16;   // エンコーダ 押し込みスイッチ
constexpr int MY_BTN_NEXT = 17;
constexpr int MY_BTN_PREV = 32;

// ---------------- 操作の調整値 ----------------
constexpr uint32_t LONG_PRESS_MS  = 600;   // 長押しと判定する時間
constexpr uint32_t SUPER_LONG_MS  = 1500;  // ノブの超長押し（デザイン切替）
constexpr uint32_t REPEAT_MS      = 450;   // 長押し中の早送り/巻き戻しの繰り返し間隔
constexpr uint32_t SKIP_INTERVAL_MS = 300; // 押しながら回す時の曲送り最短間隔
constexpr int  VOL_STEP = 4;               // ノブ1クリックあたりの音量変化(0-127)
constexpr int  ENC_STEPS_PER_DETENT = 4;   // 1クリックあたりの遷移数（KY-040は4。2のタイプもある）
constexpr bool ENC_REVERSE = false;        // 回す向きが逆ならtrue

// ---------------- TFT調整値 ----------------
constexpr int  TFT_OFFSET_X = 82;
constexpr int  TFT_OFFSET_Y = 18;    // 右端がノイズなら増やす / 左端が欠けるなら減らす（'c'キーで確認）
constexpr int  TFT_ROT      = 1;     // 向きが逆なら3
constexpr bool TFT_INVERT   = false; // 色が反転して見えたら切り替える
constexpr bool TFT_BL_ACTIVE_LOW = true;   // BLがLowで点灯する基板ならtrue（Highで点灯ならfalse）
const uint8_t BL_LEVELS[] = {255, 140, 60, 15};   // 明るさの段階（255=最大）
constexpr int BL_LEVEL_COUNT = 4;

constexpr int MODE2_BG_EXTRA = 170;   // 表示形式2の背面スペアナの明るさ(256=100%)
constexpr float OVL_DIM = 0.30f;      // オーバーレイ表示中、背景をどれだけ暗くするか
constexpr uint32_t OVL_MS = 1600;     // オーバーレイを出しておく時間

// ---------------- イコライザ調整値 ----------------
constexpr float EQ_DB_MIN  = -60.0f;  // これ以下は0
constexpr float EQ_DB_MAX  = -15.0f;  // これ以上は満タン
constexpr float EQ_TILT_DB = 1.5f;    // 高域ほど持ち上げる量(dB/バンド)
constexpr float SAMPLE_RATE = 44100.0f;

// スマホの再生位置を取り込む間隔(秒)。スマホ側でシークしても、この間隔以内に反映される
constexpr uint32_t PLAYPOS_INTERVAL_S = 2;

// ============================================================
// スペアナのデザイン定義（スタイル x 配色）
// ============================================================
enum { ST_LED = 0, ST_FINE, ST_MIRROR, ST_DOT };
enum { TH_CLASSIC = 0, TH_NEON, TH_AMBER };

struct Stop { float t; uint8_t r, g, b; };    // グラデーションの色の節目（t: 0=下 $301C 1=上）
const Stop PAL_CLASSIC[] = {{0.00f, 0, 215, 80}, {0.50f, 170, 230, 20}, {0.72f, 255, 190, 0}, {1.00f, 255, 45, 30}};
const Stop PAL_NEON[]    = {{0.00f, 0, 225, 255}, {0.45f, 50, 120, 255}, {0.75f, 170, 70, 255}, {1.00f, 255, 70, 190}};
const Stop PAL_AMBER[]   = {{0.00f, 255, 200, 30}, {0.55f, 255, 125, 0}, {1.00f, 255, 40, 25}};
struct Theme { const Stop* st; int n; };
const Theme THEMES[] = {{PAL_CLASSIC, 4}, {PAL_NEON, 4}, {PAL_AMBER, 3}};

struct Design { uint8_t style; uint8_t theme; const char* name; };
const Design DESIGNS[] = {
  {ST_LED,    TH_CLASSIC, "LED / クラシック"},
  {ST_LED,    TH_AMBER,   "LED / アンバー"},
  {ST_FINE,   TH_NEON,    "ファイン / ネオン"},
  {ST_MIRROR, TH_NEON,    "ミラー / ネオン"},
  {ST_DOT,    TH_AMBER,   "ドット / アンバー"},
  {ST_DOT,    TH_CLASSIC, "ドット / クラシック"},
};
constexpr int DESIGN_COUNT = sizeof(DESIGNS) / sizeof(DESIGNS[0]);
int designIdx = 0;

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

// 再生位置（自前で時間を進め、スマホの通知で補正する）
uint32_t posBaseMs = 0, posBaseAt = 0;
bool posRunning = false;
uint32_t pPlayPos = 0;                 // スマホから通知された再生位置（BTタスク -> loop）
volatile bool playPosDirty = false;
uint32_t lastPosSyncAt = 0;

bool demoMode = false;
bool eqPrint  = false;
bool posPrint = false;     // 再生位置の同期ログを毎回出す（'v'で切替）

// 表示形式・明るさ・オーバーレイ
int  displayMode = 0;      // 0..2（画面上の表示は+1した番号）
int  blIdx = 0;
constexpr int MODE_COUNT = 3;
const char* const MODE_NAMES[MODE_COUNT] = {"1:標準", "2:タイトル+再生位置(背面EQ)", "3:全面スペアナ"};
enum { OVL_NONE = 0, OVL_VOLUME = 1, OVL_BRIGHT = 2 };
int ovlType = OVL_NONE;
uint32_t ovlUntil = 0;
uint32_t volIgnoreUntil = 0;           // 接続直後の音量同期ではオーバーレイを出さない

// 音声受信の統計
static uint32_t rxBytes = 0;
volatile uint32_t lastAudioMs = 0;

void showOverlay(int type) {
  ovlType = type;
  ovlUntil = millis() + OVL_MS;
}

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

// スマホが再生位置(ms)を通知してくる。シーク操作やリピート再生でもここで補正される
void onPlayPos(uint32_t pos) {
  portENTER_CRITICAL(&mux);
  pPlayPos = pos;
  playPosDirty = true;
  portEXIT_CRITICAL(&mux);
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

// スマホの再生位置で、自前のタイマーを補正する
void applyPlayPos(uint32_t now) {
  uint32_t pos;
  portENTER_CRITICAL(&mux);
  if (!playPosDirty) { portEXIT_CRITICAL(&mux); return; }
  playPosDirty = false;
  pos = pPlayPos;
  portEXIT_CRITICAL(&mux);
  if (demoMode) return;
  if (pos == 0xFFFFFFFFu) return;            // 再生位置が不明
  long drift = (long)currentPos(now) - (long)pos;
  if (posPrint || labs(drift) > 1500) {
    Serial.printf("[%lu] [POS] phone=%lu ms local=%lu ms drift=%ld ms%s\n", now, (unsigned long)pos,
                  (unsigned long)currentPos(now), drift, labs(drift) > 1500 ? "  (補正: シーク等)" : "");
  }
  posBaseMs = pos;
  posBaseAt = now;
  lastPosSyncAt = now;
}

void trackConnectionChanges(uint32_t now) {
  static bool prevBt = false, prevAvrc = false;
  if (btConnected != prevBt) {
    prevBt = btConnected;
    if (btConnected) volIgnoreUntil = now + 2500;       // 接続直後の音量同期は無視
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

// スマホ側で音量が変えられたときも、オーバーレイを出す
void pollVolume(uint32_t now) {
  static int lastVol = -1;
  static uint32_t lastPoll = 0;
  if (now - lastPoll < 50) return;
  lastPoll = now;
  int v = (int)a2dp_sink.get_volume();
  if (lastVol < 0) { lastVol = v; return; }
  if (v != lastVol) {
    lastVol = v;
    if (btConnected && now > volIgnoreUntil) showOverlay(OVL_VOLUME);
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
// TFT（画面全体を1枚のフレームバッファに描いてから転送する）
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
constexpr int FB_W = 284, FB_H = 76;
lgfx::LGFX_Sprite fb(&tft);      // フレームバッファ（8bit色 約21KB）
bool fbReady = false;

// ---- 色（RGB565）----
const uint16_t C_BG      = 0x0000;
const uint16_t C_PEAK    = lgfx::color565(255, 255, 255);
const uint16_t C_TITLE   = lgfx::color565(255, 255, 255);
const uint16_t C_ARTIST  = lgfx::color565(120, 200, 255);
const uint16_t C_ALBUM   = lgfx::color565(170, 170, 170);
const uint16_t C_BAR_BG  = lgfx::color565(50, 50, 50);
const uint16_t C_BAR     = lgfx::color565(0, 200, 80);
const uint16_t C_GREEN   = lgfx::color565(0, 220, 60);
const uint16_t C_RED     = lgfx::color565(255, 0, 0);
const uint16_t C_OVL_BG     = lgfx::color565(36, 36, 85);
const uint16_t C_OVL_BORDER = lgfx::color565(110, 200, 255);
const uint16_t C_OVL_OFF    = lgfx::color565(73, 73, 85);
const uint16_t C_OVL_LABEL  = lgfx::color565(160, 210, 255);
const uint16_t C_OVL_TEXT   = lgfx::color565(255, 255, 255);

// 明るさを落とす（s256: 256=そのまま）
static inline uint16_t scaleColor(uint16_t c, int s256) {
  int r = (((c >> 11) & 31) * s256) >> 8;
  int g = (((c >> 5) & 63) * s256) >> 8;
  int b = ((c & 31) * s256) >> 8;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

// 画面全体の減光（オーバーレイ表示中は暗くする）
float dimLevel = 1.0f;
int sceneScale256 = 256;
static inline uint16_t sc(uint16_t c, int extra256 = 256) {
  return scaleColor(c, (sceneScale256 * extra256) >> 8);
}

// グラデーションの色を返す（t: 0=下 $301C 1=上）
uint16_t paletteColor(int theme, float t) {
  const Theme& th = THEMES[theme];
  if (t <= th.st[0].t) return lgfx::color565(th.st[0].r, th.st[0].g, th.st[0].b);
  for (int i = 1; i < th.n; i++) {
    if (t <= th.st[i].t) {
      float u = (t - th.st[i - 1].t) / (th.st[i].t - th.st[i - 1].t);
      int r = (int)(th.st[i - 1].r + (th.st[i].r - th.st[i - 1].r) * u);
      int g = (int)(th.st[i - 1].g + (th.st[i].g - th.st[i - 1].g) * u);
      int b = (int)(th.st[i - 1].b + (th.st[i].b - th.st[i - 1].b) * u);
      return lgfx::color565(r, g, b);
    }
  }
  const Stop& e = th.st[th.n - 1];
  return lgfx::color565(e.r, e.g, e.b);
}

// ---- レベル（滑らかな上昇・下降とピークホールド）----
float    level[BANDS] = {0};          // 0.0 - 1.0
float    peakFrac[BANDS] = {0};       // 0.0 - 1.0
uint32_t peakHoldUntil[BANDS] = {0};

constexpr int TEXT_X = 80;
constexpr int TITLE_W = 204, TITLE_GAP = 40;
uint32_t lastFrame = 0, frames = 0;
float fps = 0;

// バックライト（PWM調光）
void setBacklight(int idx) {
  int duty = BL_LEVELS[idx];
  if (TFT_BL_ACTIVE_LOW) duty = 255 - duty;
  ledcWrite(PIN_TFT_BL, duty);
}

void updateEq(uint32_t now) {
  bool active = demoMode ? (((now / 1000) % 16) < 12)
                         : (audioStarted && (now - lastAudioMs < 400));
  for (int b = 0; b < BANDS; b++) {
    float target = active ? (demoMode ? demoTarget(b, now) : bandLvl[b]) : 0.0f;
    if (target > level[b]) level[b] += (target - level[b]) * 0.6f;   // 上昇は速く
    else                   level[b] -= 0.035f;                       // 下降はなめらか
    if (level[b] < 0) level[b] = 0;

    if (level[b] >= peakFrac[b]) {
      peakFrac[b] = level[b];
      peakHoldUntil[b] = now + 400;
    } else if (now > peakHoldUntil[b]) {
      peakFrac[b] -= 0.0065f;
      if (peakFrac[b] < 0) peakFrac[b] = 0;
    }
  }
}

// ------------------------------------------------------------
// スペアナ描画（指定した矩形に、現在のデザインで描く）
//   extra256: 明るさ（背面に薄く出すときに下げる。256=100%）
// ------------------------------------------------------------
void drawSpectrum(int X, int Y, int W, int H, int extra256) {
  const Design& d = DESIGNS[designIdx];
  const int style = d.style;
  const int n = (style == ST_FINE) ? 24 : BANDS;

  // バンドのレベルを、描く本数に合わせる（ファインは24本に補間）
  float lv[24], pk[24];
  if (n == BANDS) {
    for (int b = 0; b < BANDS; b++) { lv[b] = level[b]; pk[b] = peakFrac[b]; }
  } else {
    for (int i = 0; i < n; i++) {
      float p = (float)i * (BANDS - 1) / (n - 1);
      int lo = (int)p;
      int hi = (lo + 1 < BANDS) ? lo + 1 : lo;
      float f = p - lo;
      lv[i] = level[lo] * (1.0f - f) + level[hi] * f;
      pk[i] = peakFrac[lo] * (1.0f - f) + peakFrac[hi] * f;
    }
  }

  int gap = (W >= 200) ? (style == ST_FINE ? 1 : 4) : 1;
  int bw = (W - (n - 1) * gap) / n;
  if (bw < 1) bw = 1;
  int x0 = X + (W - (n * bw + (n - 1) * gap)) / 2;

  // 段ごとの色（下から上へのグラデーション）
  int nseg;
  if (style == ST_MIRROR)    nseg = (H / 2 - 5) / 4 + 1;   // 中央から上下に9段
  else if (style == ST_FINE) nseg = H / 3;                 // 細かい段(25段)
  else                       nseg = H / 4;                 // 19段
  uint16_t rowC[32];
  for (int s = 0; s < nseg && s < 32; s++) {
    float t = (nseg > 1) ? (float)s / (nseg - 1) : 0.0f;
    rowC[s] = sc(paletteColor(d.theme, t), extra256);
  }
  uint16_t peakC = sc(C_PEAK, extra256);
  const int cY = Y + H / 2;

  for (int i = 0; i < n; i++) {
    int x = x0 + i * (bw + gap);
    int nl = (int)(lv[i] * nseg + 0.5f);
    if (nl > nseg) nl = nseg;
    int p = (int)(pk[i] * nseg + 0.5f);
    if (p > nseg) p = nseg;

    if (style == ST_MIRROR) {
      for (int s = 0; s < nl; s++) {
        fb.fillRect(x, cY - 5 - 4 * s, bw, 3, rowC[s]);
        fb.fillRect(x, cY + 2 + 4 * s, bw, 3, rowC[s]);
      }
      if (p > 0) {
        fb.fillRect(x, cY - 5 - 4 * (p - 1), bw, 3, peakC);
        fb.fillRect(x, cY + 2 + 4 * (p - 1), bw, 3, peakC);
      }
    } else if (style == ST_DOT) {
      int dpb = bw / 4;
      if (dpb < 1) dpb = 1;
      int dx0 = x + (bw - dpb * 4) / 2 + 2;           // 最初の点の中心
      for (int s = 0; s < nl; s++) {
        int cy = Y + H - 2 - 4 * s;
        for (int k = 0; k < dpb; k++) fb.fillCircle(dx0 + 4 * k, cy, 1, rowC[s]);
      }
      if (p > 0) {
        int cy = Y + H - 2 - 4 * (p - 1);
        for (int k = 0; k < dpb; k++) fb.fillCircle(dx0 + 4 * k, cy, 1, peakC);
      }
    } else {                                          // ST_LED / ST_FINE
      int sh = (style == ST_FINE) ? 2 : 3;
      int pitch = sh + 1;
      for (int s = 0; s < nl; s++) fb.fillRect(x, Y + H - (s + 1) * pitch, bw, sh, rowC[s]);
      if (p > 0) fb.fillRect(x, Y + H - p * pitch, bw, sh, peakC);
    }
  }
}

// ------------------------------------------------------------
// 文字まわりの補助
// ------------------------------------------------------------
Scroller scrStd = {"", 0, 0, 0}, scrTwo = {"", 0, 0, 0};

// 長い文字を横スクロールさせるためのオフセットを返す（先に setFont しておくこと）
int updateScroll(Scroller& s, const char* t, int viewW, uint32_t now) {
  if (strcmp(s.last, t) != 0) {
    snprintf(s.last, sizeof(s.last), "%s", t);
    s.w = fb.textWidth(t);
    s.x = 0; s.holdUntil = 0;
  }
  if (s.w > viewW) {
    if (s.x == 0 && s.holdUntil == 0) s.holdUntil = now + 1500;   // 先頭で一旦停止
    if (now >= s.holdUntil) {
      s.x += 2;
      if (s.x >= s.w + TITLE_GAP) { s.x = 0; s.holdUntil = 0; }
    }
  } else {
    s.x = 0; s.holdUntil = 0;
  }
  return s.x;
}

// 影付き文字（背景が動いても読みやすくする）
void drawTextShadow(const char* str, int x, int y, uint16_t fg) {
  fb.setTextColor(C_BG);
  fb.drawString(str, x + 1, y + 1);
  fb.setTextColor(fg);
  fb.drawString(str, x, y);
}

void drawPlayIcon(int x, int y, int h) {
  if (shownPlaying()) fb.fillTriangle(x, y, x, y + h, x + h * 3 / 4, y + h / 2, sc(C_GREEN));
  else {
    int w = h / 4 > 2 ? h / 4 : 2;
    fb.fillRect(x, y, w, h, sc(C_ALBUM));
    fb.fillRect(x + w + 3, y, w, h, sc(C_ALBUM));
  }
}

// ------------------------------------------------------------
// 表示形式1: 標準（左にスペアナ、右に曲情報）
// ------------------------------------------------------------
void renderStandard(uint32_t now) {
  drawSpectrum(0, 0, 76, 76, 256);

  const char* t = titleText();
  fb.setFont(&lgfx::fonts::lgfxJapanGothic_20);
  int off = updateScroll(scrStd, t, TITLE_W, now);
  fb.setClipRect(TEXT_X, 0, TITLE_W, 22);
  fb.setTextColor(sc(C_TITLE));
  fb.drawString(t, TEXT_X - off, 1);
  if (scrStd.w > TITLE_W) fb.drawString(t, TEXT_X - off + scrStd.w + TITLE_GAP, 1);
  fb.clearClipRect();

  fb.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  fb.setTextColor(sc(C_ARTIST));
  fb.drawString(artistText(), TEXT_X, 25);
  fb.setTextColor(sc(C_ALBUM));
  fb.drawString(albumText(), TEXT_X, 43);

  // 再生位置
  uint32_t pos = currentPos(now);
  uint32_t shown = (curDur > 0 && pos > curDur) ? curDur : pos;
  drawPlayIcon(81, 64, 10);
  const int bx = 96, by = 67, bw = 100, bh = 5;
  fb.fillRect(bx, by, bw, bh, sc(C_BAR_BG));
  if (curDur > 0) fb.fillRect(bx, by, (int)((uint64_t)bw * shown / curDur), bh, sc(C_BAR));
  char a[12], b[12], buf[28];
  fmtTime(a, sizeof(a), shown);
  if (curDur > 0) { fmtTime(b, sizeof(b), curDur); snprintf(buf, sizeof(buf), "%s / %s", a, b); }
  else            { snprintf(buf, sizeof(buf), "%s", a); }
  fb.setFont(&lgfx::fonts::lgfxJapanGothic_12);
  fb.setTextColor(sc(C_ALBUM));
  fb.drawString(buf, 200, 63);
}

// ------------------------------------------------------------
// 表示形式2: タイトルと再生位置の2段表示 + 背面でスペアナ
// ------------------------------------------------------------
void renderTwoLine(uint32_t now) {
  drawSpectrum(0, 0, FB_W, FB_H, MODE2_BG_EXTRA);

  const char* t = titleText();
  fb.setFont(&lgfx::fonts::lgfxJapanGothic_24);
  const int viewW = FB_W - 8;
  int off = updateScroll(scrTwo, t, viewW, now);
  drawTextShadow(t, 4 - off, 2, sc(C_TITLE));
  if (scrTwo.w > viewW) drawTextShadow(t, 4 - off + scrTwo.w + TITLE_GAP, 2, sc(C_TITLE));

  uint32_t pos = currentPos(now);
  uint32_t shown = (curDur > 0 && pos > curDur) ? curDur : pos;
  char a[12], b[12], buf[28];
  fmtTime(a, sizeof(a), shown);
  if (curDur > 0) { fmtTime(b, sizeof(b), curDur); snprintf(buf, sizeof(buf), "%s / %s", a, b); }
  else            { snprintf(buf, sizeof(buf), "%s", a); }

  drawPlayIcon(6, 38, 24);
  fb.setFont(&lgfx::fonts::lgfxJapanGothic_32);
  drawTextShadow(buf, 32, 34, sc(C_TITLE));

  const int bx = 4, by = 69, bw = FB_W - 8, bh = 5;
  fb.fillRect(bx, by, bw, bh, sc(C_BAR_BG));
  if (curDur > 0) fb.fillRect(bx, by, (int)((uint64_t)bw * shown / curDur), bh, sc(C_BAR));
}

// ------------------------------------------------------------
// 表示形式3: 全面スペアナ
// ------------------------------------------------------------
void renderFull(uint32_t now) {
  (void)now;
  drawSpectrum(0, 0, FB_W, FB_H, 256);
}

// ------------------------------------------------------------
// 前面のオーバーレイ（音量 / 明るさ）。背景は暗くなって動き続ける
// ------------------------------------------------------------
void drawOverlay() {
  const int px = 28, py = 8, pw = 228, ph = 60;
  fb.fillRoundRect(px, py, pw, ph, 6, C_OVL_BG);
  fb.drawRoundRect(px, py, pw, ph, 6, C_OVL_BORDER);

  const char* label;
  char val[16];
  int segs, lit, gap;
  if (ovlType == OVL_VOLUME) {
    int v = (int)a2dp_sink.get_volume();
    if (v < 0) v = 0;
    if (v > 127) v = 127;
    label = "VOLUME";
    snprintf(val, sizeof(val), "%d", v * 100 / 127);
    segs = 20;
    gap = 1;
    lit = (int)(v * segs / 127.0f + 0.5f);
  } else {
    label = "BRIGHTNESS";
    snprintf(val, sizeof(val), "%d%%", BL_LEVELS[blIdx] * 100 / 255);
    segs = BL_LEVEL_COUNT;
    gap = 2;
    lit = BL_LEVEL_COUNT - blIdx;
  }

  fb.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  fb.setTextColor(C_OVL_LABEL);
  fb.drawString(label, px + 12, py + 9);
  fb.setFont(&lgfx::fonts::lgfxJapanGothic_24);
  fb.setTextColor(C_OVL_TEXT);
  fb.drawString(val, px + pw - 12 - fb.textWidth(val), py + 5);

  const int gx = px + 12, gy = py + 38, gw = pw - 24, gh = 14;
  int sw = (gw - (segs - 1) * gap) / segs;
  for (int i = 0; i < segs; i++) {
    float t = (segs > 1) ? (float)i / (segs - 1) : 0.0f;
    uint16_t c = (i < lit) ? paletteColor(DESIGNS[designIdx].theme, t) : C_OVL_OFF;
    fb.fillRect(gx + i * (sw + gap), gy, sw, gh, c);
  }
}

// ------------------------------------------------------------
// 1フレーム描画して転送
// ------------------------------------------------------------
void renderFrame(uint32_t now) {
  if (!fbReady) return;

  bool ovl = (ovlType != OVL_NONE) && (now < ovlUntil);
  float target = ovl ? OVL_DIM : 1.0f;
  if (dimLevel < target)      { dimLevel += 0.15f; if (dimLevel > target) dimLevel = target; }
  else if (dimLevel > target) { dimLevel -= 0.15f; if (dimLevel < target) dimLevel = target; }
  sceneScale256 = (int)(dimLevel * 256.0f);

  fb.fillScreen(C_BG);
  if (displayMode == 0)      renderStandard(now);
  else if (displayMode == 1) renderTwoLine(now);
  else                       renderFull(now);
  if (ovl) drawOverlay();
  else if (ovlType != OVL_NONE) ovlType = OVL_NONE;

  fb.pushSprite(0, 0);
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
  ledcAttach(PIN_TFT_BL, 5000, 8);      // バックライトをPWMで制御
  setBacklight(blIdx);
  Serial.printf("[TFT] backlight: GPIO%d PWM (active-%s) level %d/%d\n", PIN_TFT_BL,
                TFT_BL_ACTIVE_LOW ? "low" : "high", blIdx + 1, BL_LEVEL_COUNT);
  tft.init();
  tft.setRotation(TFT_ROT);
  Serial.printf("[TFT] init done: %d x %d (offset %d,%d)  ※接続の有無は検出不可（未接続でもエラーにならない）\n",
                tft.width(), tft.height(), TFT_OFFSET_X, TFT_OFFSET_Y);
  drawCalibration();
  delay(1500);
  tft.fillScreen(C_BG);

  fb.setColorDepth(8);
  fbReady = (fb.createSprite(FB_W, FB_H) != nullptr);
  Serial.printf("[TFT] frame buffer: %s (%d x %d, 8bit)\n", fbReady ? "OK" : "FAILED (メモリ不足)", FB_W, FB_H);
}
#endif  // USE_TFT

// ============================================================
// ロータリーエンコーダ（割り込みで取りこぼしを防ぐ）
// ============================================================
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile int encAccum = 0;
volatile uint8_t encPrev = 0;

void IRAM_ATTR encISR() {
  uint8_t cur = (uint8_t)((gpio_get_level((gpio_num_t)MY_ENC_A) << 1) | gpio_get_level((gpio_num_t)MY_ENC_B));
  uint8_t idx = (uint8_t)(((encPrev << 2) | cur) & 0x0F);
  encAccum = encAccum + ENC_TABLE[idx];
  encPrev = cur;
}

// 溜まった回転量を「クリック数」として取り出す（+が時計回り）
int takeEncDetents() {
  int d = 0;
  portENTER_CRITICAL(&encMux);
  while (encAccum >= ENC_STEPS_PER_DETENT)  { encAccum = encAccum - ENC_STEPS_PER_DETENT; d++; }
  while (encAccum <= -ENC_STEPS_PER_DETENT) { encAccum = encAccum + ENC_STEPS_PER_DETENT; d--; }
  portEXIT_CRITICAL(&encMux);
  return ENC_REVERSE ? -d : d;
}

// ============================================================
// 操作（エンコーダ / ボタン / シリアルコマンド）
// ============================================================
void doPlayPause() {
  if (audioStarted) { Serial.println("[CMD] pause"); a2dp_sink.pause(); }
  else              { Serial.println("[CMD] play");  a2dp_sink.play();  }
}
void doNext() { Serial.println("[CMD] next");     a2dp_sink.next(); }
void doPrev() { Serial.println("[CMD] previous"); a2dp_sink.previous(); }
void doFF()   { Serial.println("[CMD] fast forward"); a2dp_sink.fast_forward(); }
void doRew()  { Serial.println("[CMD] rewind");       a2dp_sink.rewind(); }

void doVolume(int delta) {
  int v = (int)a2dp_sink.get_volume() + delta;
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  a2dp_sink.set_volume((uint8_t)v);
  Serial.printf("[CMD] volume %d/127\n", v);
  showOverlay(OVL_VOLUME);
}

void cycleBrightness() {
  blIdx = (blIdx + 1) % BL_LEVEL_COUNT;
#if USE_TFT
  setBacklight(blIdx);
#endif
  Serial.printf("[BL] level %d/%d (duty %d)\n", blIdx + 1, BL_LEVEL_COUNT, BL_LEVELS[blIdx]);
  showOverlay(OVL_BRIGHT);
}

void setMode(int m) {
  displayMode = m;
  Serial.printf("[MODE] 表示形式 %s\n", MODE_NAMES[displayMode]);
}
void cycleMode() { setMode((displayMode + 1) % MODE_COUNT); }

void cycleDesign() {
  designIdx = (designIdx + 1) % DESIGN_COUNT;
  Serial.printf("[DESIGN] %d/%d %s\n", designIdx + 1, DESIGN_COUNT, DESIGNS[designIdx].name);
}

struct Btn {
  int pin; const char* name;
  bool stable; bool lastRaw; uint32_t changedAt;
  uint32_t pressedAt; bool longFired; bool suppressed; uint32_t lastRepeat;
};
enum { B_SW = 0, B_NEXT = 1, B_PREV = 2 };
Btn btns[3] = {
  {MY_ENC_SW,   "ENC-SW", true, true, 0, 0, false, false, 0},
  {MY_BTN_NEXT, "NEXT",   true, true, 0, 0, false, false, 0},
  {MY_BTN_PREV, "PREV",   true, true, 0, 0, false, false, 0},
};
bool encTurnedWhilePressed = false;
uint32_t lastSkipAt = 0;

void handleButtons(uint32_t now) {
  // --- デバウンスと押下/離しの検出 ---
  for (int i = 0; i < 3; i++) {
    Btn& b = btns[i];
    bool raw = digitalRead(b.pin);
    if (raw != b.lastRaw) { b.lastRaw = raw; b.changedAt = now; }
    if (now - b.changedAt > 25 && raw != b.stable) {
      b.stable = raw;
      if (!raw) {                                   // 押した
        b.pressedAt = now; b.longFired = false; b.suppressed = false; b.lastRepeat = now;
        if (i == B_SW) encTurnedWhilePressed = false;
        Serial.printf("[BTN] %s down\n", b.name);
      } else {                                      // 離した
        uint32_t dur = now - b.pressedAt;
        Serial.printf("[BTN] %s up (%lu ms)\n", b.name, (unsigned long)dur);
        if (i == B_SW) {
          if (!encTurnedWhilePressed) {             // 押している時間で動作を分ける
            if (dur >= SUPER_LONG_MS)      { Serial.println("[BTN] ENC-SW 超長押し -> デザイン切替"); cycleDesign(); }
            else if (dur >= LONG_PRESS_MS) { Serial.println("[BTN] ENC-SW 長押し -> 表示形式切替"); cycleMode(); }
            else                           doPlayPause();
          }
        } else if (!b.suppressed && !b.longFired) { // NEXT / PREV の短押し
          if (i == B_NEXT) doNext(); else doPrev();
        }
      }
    }
  }

  // --- NEXT+PREV 同時押し（ほぼ同時に押された場合）---
  Btn& n = btns[B_NEXT];
  Btn& p = btns[B_PREV];
  if (!n.stable && !p.stable && !n.suppressed && !p.suppressed && !n.longFired && !p.longFired) {
    uint32_t dt = n.pressedAt > p.pressedAt ? n.pressedAt - p.pressedAt : p.pressedAt - n.pressedAt;
    if (dt < 350) {
      n.suppressed = true; p.suppressed = true;
      Serial.println("[BTN] NEXT+PREV");
      cycleBrightness();
    }
  }

  // --- NEXT / PREV の長押しと繰り返し（早送り・巻き戻し）---
  for (int i = B_NEXT; i <= B_PREV; i++) {
    Btn& b = btns[i];
    if (b.stable || b.suppressed) continue;
    if (now - b.pressedAt >= LONG_PRESS_MS) {
      if (!b.longFired) {
        b.longFired = true; b.lastRepeat = now;
        if (i == B_NEXT) doFF(); else doRew();
      } else if (now - b.lastRepeat >= REPEAT_MS) {
        b.lastRepeat = now;
        if (i == B_NEXT) doFF(); else doRew();
      }
    }
  }
}

void handleEncoder(uint32_t now) {
  int d = takeEncDetents();
  if (d == 0) return;
  bool swDown = !btns[B_SW].stable;
  Serial.printf("[ENC] %+d%s\n", d, swDown ? " (push)" : "");
  if (swDown) {                                      // 押しながら回す = 曲送り
    encTurnedWhilePressed = true;
    if (now - lastSkipAt >= SKIP_INTERVAL_MS) {
      lastSkipAt = now;
      if (d > 0) doNext(); else doPrev();
    }
  } else {
    doVolume(d * VOL_STEP);                          // 回す = 音量
  }
}

void printStatus(uint32_t now) {
  char a[12], b[12];
  fmtTime(a, sizeof(a), currentPos(now));
  fmtTime(b, sizeof(b), curDur);
  Serial.printf("[STAT] t=%lus BT=%s AVRC=%s audio=%s vol=%d pos=%s/%s rx=%lukB heap=%u(min %u) fps=%.1f sync=%ds%s\n",
                now / 1000,
                btConnected ? "conn" : "disc", avrcConnected ? "conn" : "disc",
                audioStarted ? "playing" : "stopped", (int)a2dp_sink.get_volume(),
                a, b, (unsigned long)(rxBytes / 1024),
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
#if USE_TFT
                fps,
#else
                0.0f,
#endif
                lastPosSyncAt ? (int)((now - lastPosSyncAt) / 1000) : -1,
                demoMode ? " [DEMO]" : "");
  Serial.printf("[STAT] 表示=%s デザイン=%s 明るさ=%d/%d\n",
                MODE_NAMES[displayMode], DESIGNS[designIdx].name, blIdx + 1, BL_LEVEL_COUNT);
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
  Serial.println(" f / r : 早送り / 巻き戻し");
  Serial.println(" + / - : 音量 上げ/下げ  s : 状態表示   e : EQ数値の表示 切替");
  Serial.println(" m     : 表示形式の切替  1/2/3 : 表示形式を選択");
  Serial.println(" y     : スペアナのデザイン切替   b : 明るさ切替");
  Serial.println(" d     : デモ表示 切替（スマホ不要でTFTを確認）");
  Serial.println(" v     : 再生位置の同期ログ 切替");
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
      case 'f': doFF(); break;
      case 'r': doRew(); break;
      case '+': doVolume(+VOL_STEP); break;
      case '-': doVolume(-VOL_STEP); break;
      case 's': printStatus(now); break;
      case 'e': eqPrint = !eqPrint; Serial.printf("[EQ] 数値表示 %s\n", eqPrint ? "ON" : "OFF"); break;
      case 'm': cycleMode(); break;
      case '1': case '2': case '3': setMode(c - '1'); break;
      case 'y': cycleDesign(); break;
      case 'b': cycleBrightness(); break;
      case 'v': posPrint = !posPrint; Serial.printf("[POS] 同期ログ %s\n", posPrint ? "ON" : "OFF"); break;
      case 'd': setDemo(!demoMode, now); break;
      case 'c':
#if USE_TFT
        Serial.println("[TFT] 表示位置確認パターンを4秒表示");
        drawCalibration();
        delay(4000);
#endif
        break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== BT Music Player (TFT + Encoder) boot ===");

  pinMode(MY_ENC_A, INPUT_PULLUP);
  pinMode(MY_ENC_B, INPUT_PULLUP);
  pinMode(MY_ENC_SW, INPUT_PULLUP);
  pinMode(MY_BTN_NEXT, INPUT_PULLUP);
  pinMode(MY_BTN_PREV, INPUT_PULLUP);
  encPrev = (uint8_t)((digitalRead(MY_ENC_A) << 1) | digitalRead(MY_ENC_B));
  attachInterrupt(digitalPinToInterrupt(MY_ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MY_ENC_B), encISR, CHANGE);
  Serial.printf("[CTRL] encoder A=%d B=%d SW=%d / NEXT=%d / PREV=%d\n",
                MY_ENC_A, MY_ENC_B, MY_ENC_SW, MY_BTN_NEXT, MY_BTN_PREV);

  spectrumInit();
  Serial.print("[EQ] band Hz:");
  for (int i = 0; i < BANDS; i++) Serial.printf(" %d", (int)bandHz[i]);
  Serial.println();
  Serial.printf("[DESIGN] %d/%d %s\n", designIdx + 1, DESIGN_COUNT, DESIGNS[designIdx].name);

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
  a2dp_sink.set_avrc_rn_play_pos_callback(onPlayPos, PLAYPOS_INTERVAL_S);   // スマホの再生位置を取り込む
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
  handleEncoder(now);
  handleButtons(now);
  applyMetadata(now);
  applyPlayPos(now);
  trackConnectionChanges(now);
  syncPosition(now);
  pollVolume(now);

#if USE_TFT
  if (now - lastFrame >= 33) {       // 約30fps
    static uint32_t fpsAt = 0;
    lastFrame = now;
    updateEq(now);
    renderFrame(now);
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
