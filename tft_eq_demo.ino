// ============================================================
// ST7789 76x284 TFT 表示テスト + 90年代風イコライザ デモ
//  - 音声はまだ使わず、ダミーデータで動作
//  - ライブラリ: LovyanGFX
//  - TFT未接続でもエラーにならず動作します（SPIは書き込み専用のため）
//    表示処理そのものを止めたい場合は USE_TFT を 0 にしてください
//  - シリアル(115200)に状況を出力します
// ============================================================
#define USE_TFT 1

#if USE_TFT
#include <LovyanGFX.hpp>

// ---- ピン割り当て（基板の端子名との対応）----
constexpr int PIN_TFT_SCLK = 18;  // SCL
constexpr int PIN_TFT_MOSI = 23;  // SDA
constexpr int PIN_TFT_CS   = 5;   // CS
constexpr int PIN_TFT_DC   = 19;  // DC
constexpr int PIN_TFT_RST  = 4;   // RST
constexpr int PIN_TFT_BL   = 27;  // BL

// ---- パネル調整値（表示がずれる場合はここを調整）----
constexpr int  TFT_OFFSET_X = 82;    // 76x284パネルの標準的な値
constexpr int  TFT_OFFSET_Y = 0;     // 端にノイズが出る場合は 18 なども試す
constexpr int  TFT_ROT      = 1;     // 向きが逆なら 3 にする
constexpr bool TFT_INVERT   = true;  // 色が反転して見えたら false

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
      cfg.readable        = false;   // 読み出さない＝未接続でも固まらない
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
lgfx::LGFX_Sprite eqSpr(&tft);     // イコライザ 76x76
lgfx::LGFX_Sprite titleSpr(&tft);  // 曲名スクロール用 204x22

// ---- 色 ----
const uint16_t C_BG      = 0x0000;
const uint16_t C_GREEN   = lgfx::color565(0, 220, 60);
const uint16_t C_YELLOW  = lgfx::color565(255, 200, 0);
const uint16_t C_RED     = lgfx::color565(255, 40, 40);
const uint16_t C_PEAK    = lgfx::color565(255, 255, 255);
const uint16_t C_TITLE   = lgfx::color565(255, 255, 255);
const uint16_t C_ARTIST  = lgfx::color565(120, 200, 255);
const uint16_t C_ALBUM   = lgfx::color565(170, 170, 170);
const uint16_t C_BAR_BG  = lgfx::color565(50, 50, 50);
const uint16_t C_BAR     = lgfx::color565(0, 200, 80);

// ---- イコライザ ----
constexpr int EQ_W = 76, EQ_H = 76;
constexpr int BANDS = 12, BAR_W = 5, BAR_GAP = 1, EQ_X0 = 2;
constexpr int SEG_H = 3, SEG_GAP = 1, SEGS = 19;   // 19 * (3+1) = 76

float    level[BANDS] = {0};          // 0.0 - 1.0
float    peakSeg[BANDS] = {0};        // セグメント単位 (0 - SEGS)
uint32_t peakHoldUntil[BANDS] = {0};

// ---- 曲情報（デモ用の固定テキスト）----
const char* TITLE  = "テスト曲 - 日本語の長いタイトルが横スクロールします / Long Title Scroll Test";
const char* ARTIST = "テストアーティスト";
const char* ALBUM  = "テストアルバム";
constexpr uint32_t DEMO_DURATION_MS = 225000;   // 3:45

constexpr int TEXT_X = 80;
constexpr int TITLE_W = 204, TITLE_H = 22, TITLE_GAP = 40;
int titleW = 0;
int scrollX = 0;

bool eqReady = false, titleReady = false;
uint32_t lastFrame = 0, lastLog = 0, frames = 0;

// ------------------------------------------------------------
// ダミーのスペクトラム（本番はここを音声解析の結果に差し替える）
// ------------------------------------------------------------
float demoTarget(int b, uint32_t t) {
  float x = 0.5f + 0.5f * sinf(t * 0.0020f * (1.0f + b * 0.23f) + b * 1.7f);
  float y = 0.5f + 0.5f * sinf(t * 0.0071f + b * 0.9f);
  float v = (0.6f * x + 0.4f * y) * (1.05f - b * 0.035f);
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  return v;
}

void updateEq(uint32_t now, bool active) {
  for (int b = 0; b < BANDS; b++) {
    float target = active ? demoTarget(b, now) : 0.0f;
    if (target > level[b]) level[b] += (target - level[b]) * 0.6f;  // 上昇は速く
    else                   level[b] -= 0.035f;                      // 下降はなめらか
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

uint16_t segColor(int s) {
  if (s < 12) return C_GREEN;
  if (s < 16) return C_YELLOW;
  return C_RED;
}

void drawEq() {
  if (!eqReady) return;
  eqSpr.fillScreen(C_BG);
  for (int b = 0; b < BANDS; b++) {
    int x = EQ_X0 + b * (BAR_W + BAR_GAP);
    int n = (int)(level[b] * SEGS + 0.5f);
    for (int s = 0; s < n && s < SEGS; s++) {
      int y = EQ_H - (s + 1) * (SEG_H + SEG_GAP);
      eqSpr.fillRect(x, y, BAR_W, SEG_H, segColor(s));
    }
    int p = (int)peakSeg[b];
    if (p > 0 && p <= SEGS) {
      int y = EQ_H - p * (SEG_H + SEG_GAP);
      eqSpr.fillRect(x, y, BAR_W, SEG_H, C_PEAK);
    }
  }
  eqSpr.pushSprite(0, 0);
}

// ------------------------------------------------------------
// 曲名（長いときは横スクロール）
// ------------------------------------------------------------
void updateTitle(uint32_t now) {
  if (!titleReady) return;
  static uint32_t holdUntil = 0;

  if (titleW > TITLE_W) {
    if (scrollX == 0 && holdUntil == 0) holdUntil = now + 1500;  // 先頭で一旦停止
    if (now >= holdUntil) {
      scrollX += 2;
      if (scrollX >= titleW + TITLE_GAP) { scrollX = 0; holdUntil = 0; }
    }
  }

  titleSpr.fillScreen(C_BG);
  titleSpr.setFont(&lgfx::fonts::lgfxJapanGothic_20);
  titleSpr.setTextColor(C_TITLE, C_BG);
  titleSpr.drawString(TITLE, -scrollX, 1);
  if (titleW > TITLE_W) titleSpr.drawString(TITLE, -scrollX + titleW + TITLE_GAP, 1);
  titleSpr.pushSprite(TEXT_X, 0);
}

void drawInfoStatic() {
  tft.fillRect(TEXT_X, 24, 204, 52, C_BG);
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  tft.setTextColor(C_ARTIST, C_BG);
  tft.drawString(ARTIST, TEXT_X, 25);
  tft.setTextColor(C_ALBUM, C_BG);
  tft.drawString(ALBUM, TEXT_X, 43);
}

// ------------------------------------------------------------
// 進捗バーと時間
// ------------------------------------------------------------
void drawProgress(uint32_t pos, uint32_t dur) {
  const int bx = TEXT_X, by = 67, bw = 110, bh = 5;
  tft.fillRect(TEXT_X, 61, 204, 15, C_BG);
  tft.fillRect(bx, by, bw, bh, C_BAR_BG);
  tft.fillRect(bx, by, (int)((uint64_t)bw * pos / dur), bh, C_BAR);

  char buf[24];
  snprintf(buf, sizeof(buf), "%d:%02d / %d:%02d",
           (int)(pos / 60000), (int)((pos / 1000) % 60),
           (int)(dur / 60000), (int)((dur / 1000) % 60));
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_12);
  tft.setTextColor(C_ALBUM, C_BG);
  tft.drawString(buf, 196, 63);
}

void updateProgress(uint32_t now) {
  static int lastSec = -1;
  uint32_t pos = now % DEMO_DURATION_MS;
  int sec = (int)(pos / 1000);
  if (sec == lastSec) return;   // 1秒に1回だけ更新（ちらつき防止）
  lastSec = sec;
  drawProgress(pos, DEMO_DURATION_MS);
}

// ------------------------------------------------------------
// 起動時の表示確認（四隅のマーカーと枠）
// 赤=左上 / 緑=右上 / 青=左下 / 白=右下 が四隅に出れば正常
// ------------------------------------------------------------
void drawCalibration() {
  int w = tft.width(), h = tft.height();
  tft.fillScreen(C_BG);
  tft.drawRect(0, 0, w, h, C_RED);
  tft.fillRect(0, 0, 8, 8, lgfx::color565(255, 0, 0));
  tft.fillRect(w - 8, 0, 8, 8, lgfx::color565(0, 255, 0));
  tft.fillRect(0, h - 8, 8, 8, lgfx::color565(0, 0, 255));
  tft.fillRect(w - 8, h - 8, 8, 8, lgfx::color565(255, 255, 255));

  char buf[32];
  snprintf(buf, sizeof(buf), "%d x %d  ST7789", w, h);
  tft.setFont(&lgfx::fonts::lgfxJapanGothic_16);
  tft.setTextColor(C_TITLE, C_BG);
  int tw = tft.textWidth(buf);
  tft.drawString(buf, (w - tw) / 2, h / 2 - 8);
}
#endif  // USE_TFT

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TFT EQ demo boot ===");

#if USE_TFT
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);   // バックライト点灯（PWM調光は後で）

  tft.init();
  tft.setRotation(TFT_ROT);
  Serial.printf("[TFT] init done: %d x %d (rotation %d, offset %d,%d)\n",
                tft.width(), tft.height(), TFT_ROT, TFT_OFFSET_X, TFT_OFFSET_Y);
  Serial.println("[TFT] note: 書き込み専用のため、TFTの接続有無は検出できません（未接続でもエラーにはなりません）");

  drawCalibration();
  delay(2500);
  tft.fillScreen(C_BG);

  eqSpr.setColorDepth(16);
  eqReady = (eqSpr.createSprite(EQ_W, EQ_H) != nullptr);
  Serial.printf("[TFT] eq sprite: %s\n", eqReady ? "OK" : "FAILED (メモリ不足)");

  titleSpr.setColorDepth(16);
  titleReady = (titleSpr.createSprite(TITLE_W, TITLE_H) != nullptr);
  Serial.printf("[TFT] title sprite: %s\n", titleReady ? "OK" : "FAILED (メモリ不足)");

  if (titleReady) {
    titleSpr.setFont(&lgfx::fonts::lgfxJapanGothic_20);
    titleW = titleSpr.textWidth(TITLE);
    Serial.printf("[TFT] title width: %d px (表示幅 %d px, %s)\n",
                  titleW, TITLE_W, titleW > TITLE_W ? "スクロールあり" : "スクロールなし");
  }

  drawInfoStatic();
  Serial.printf("[SYS] free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
#else
  Serial.println("[TFT] USE_TFT=0 : 表示処理は無効");
#endif
}

void loop() {
#if USE_TFT
  uint32_t now = millis();

  if (now - lastFrame >= 33) {          // 約30fps
    lastFrame = now;
    bool active = ((now / 1000) % 16) < 12;   // 12秒鳴って4秒無音（減衰の確認用）
    updateEq(now, active);
    drawEq();
    updateTitle(now);
    updateProgress(now);
    frames++;
  }

  if (now - lastLog >= 5000) {
    Serial.printf("[SYS] fps: %.1f, free heap: %u bytes\n",
                  frames * 1000.0f / (now - lastLog), (unsigned)ESP.getFreeHeap());
    frames = 0;
    lastLog = now;
  }
#endif
  delay(1);
}
