#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <U8g2lib.h>
#include <Wire.h>          // 追加（先頭のinclude群へ）

bool oledPresent = false;  // グローバルに追加

I2SStream i2s;                           // ← 追加
BluetoothA2DPSink a2dp_sink(i2s);        // ← 変更（i2sを渡す）
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 33 /*SCL*/, 32 /*SDA*/);

// ボタン
const int PIN_PLAY = 13, PIN_NEXT = 14, PIN_PREV = 16, PIN_VOLUP = 17;

// 曲情報（コールバックはBTタスクから呼ばれるので、保存だけして描画はloopで行う）
String title = "-", artist = "-", album = "-";
volatile bool needRedraw = true;
bool isPlaying = false;
bool isConnected = false;
uint8_t volume = 64;  // 0-127

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  Serial.printf("[AVRC] id=0x%x : %s\n", id, text);
  if (id == ESP_AVRC_MD_ATTR_TITLE)  title  = (const char*)text;
  if (id == ESP_AVRC_MD_ATTR_ARTIST) artist = (const char*)text;
  if (id == ESP_AVRC_MD_ATTR_ALBUM)  album  = (const char*)text;
  needRedraw = true;
}

void connection_callback(esp_a2d_connection_state_t state, void *) {
  isConnected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
  Serial.printf("[A2DP] connection: %s\n", a2dp_sink.to_str(state));
  needRedraw = true;
}

void audio_state_callback(esp_a2d_audio_state_t state, void *) {
  isPlaying = (state == ESP_A2D_AUDIO_STATE_STARTED);
  Serial.printf("[A2DP] audio: %s\n", a2dp_sink.to_str(state));
  needRedraw = true;
}

void draw() {
  if (!oledPresent) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_b12_t_japanese1);
  u8g2.setCursor(0, 12);  u8g2.print(isConnected ? (isPlaying ? "▶ 再生中" : "■ 停止") : "未接続");
  u8g2.setCursor(0, 30);  u8g2.print(title);
  u8g2.setCursor(0, 46);  u8g2.print(artist);
  u8g2.setCursor(0, 62);  u8g2.print(album);
  u8g2.sendBuffer();
}

bool pressed(int pin) {           // 簡易デバウンス
  if (digitalRead(pin) == LOW) {
    delay(30);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) delay(5);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== BT Music Player boot ===");

  pinMode(PIN_PLAY, INPUT_PULLUP);
  pinMode(PIN_NEXT, INPUT_PULLUP);
  pinMode(PIN_PREV, INPUT_PULLUP);
  pinMode(PIN_VOLUP, INPUT_PULLUP);

  Wire.begin(32, 33);
  Wire.beginTransmission(0x3C);
  oledPresent = (Wire.endTransmission() == 0);
  Serial.printf("OLED: %s\n", oledPresent ? "found" : "not found");
  if (oledPresent) {
    u8g2.begin();
    u8g2.enableUTF8Print();
  }

  auto cfg = i2s.defaultConfig();
  cfg.pin_bck  = 26;
  cfg.pin_ws   = 25;
  cfg.pin_data = 22;
  i2s.begin(cfg);

  a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
      ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_on_connection_state_changed(connection_callback);
  a2dp_sink.set_on_audio_state_changed(audio_state_callback);

  a2dp_sink.set_avrc_connection_state_callback([](bool connected) {
    Serial.printf("[AVRC] connection: %s\n", connected ? "Connected" : "Disconnected");
  });

  a2dp_sink.start("MyCarPlayer", false);   // 2つ目の引数: 自動再接続オフ
  Serial.println("A2DP started. スマホのBT設定から MyCarPlayer を選んでください");
}

void loop() {
  if (pressed(PIN_PLAY)) {
    Serial.println("[BTN] play/pause");
    isPlaying ? a2dp_sink.pause() : a2dp_sink.play();
  }
  if (pressed(PIN_NEXT)) { Serial.println("[BTN] next"); a2dp_sink.next(); }
  if (pressed(PIN_PREV)) { Serial.println("[BTN] prev"); a2dp_sink.previous(); }
  if (pressed(PIN_VOLUP)) {
    volume = min(127, volume + 8);
    Serial.printf("[BTN] volume %d\n", volume);
    a2dp_sink.set_volume(volume);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n') { Serial.println("[TEST] next");  a2dp_sink.next(); }
    if (c == 'p') { Serial.println("[TEST] prev");  a2dp_sink.previous(); }
    if (c == ' ') { Serial.println("[TEST] pause"); a2dp_sink.pause(); }
    if (c == 'r') { Serial.println("[TEST] play");  a2dp_sink.play(); }
  }

  if (needRedraw) { needRedraw = false; draw(); }
  delay(10);
}
