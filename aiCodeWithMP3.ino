/**
 * Revised: avoid pin conflicts and reduce race conditions
 * - Use HardwareSerial(2) (UART2) on pins TX=17 -> DF RX, RX=16 <- DF TX
 * - Initialize DFPlayer BEFORE camera begin()
 * - Keep heap checks and small delay before playback
 */

#include <Smart_Stick_v2_inferencing.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

using eloq::camera;
using eloq::ei::fomo;

// ===================== MP3 CONFIG (changed to UART2) =====================
HardwareSerial mp3Serial(2);              // use UART2
static const uint8_t PIN_MP3_TX = 4;     // ESP32 TX -> DFPlayer RX (change from 14)
static const uint8_t PIN_MP3_RX = 15;     // ESP32 RX <- DFPlayer TX (keep 16)
DFRobotDFPlayerMini player;

void safePrintLabel(const char *lbl) {
  if (lbl == nullptr) {
    Serial.print("<null>");
  } else {
    Serial.print(lbl);
  }
}

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("__EDGE IMPULSE FOMO (NO-PSRAM)__");

  // ---------- DFPLAYER INIT FIRST (avoid later pin/lock races) ----------
  mp3Serial.begin(9600, SERIAL_8N1, PIN_MP3_RX, PIN_MP3_TX);
  delay(50); // give UART some time

  if (!player.begin(mp3Serial)) {
    Serial.println("❌ Unable to begin DFPlayer:");
    Serial.println("1. Check wiring (TX->RX, RX->TX)");
    Serial.println("2. Insert SD card formatted FAT32 with files like 0001.mp3");
    while (true) { delay(1000); }
  }
  player.volume(20);
  Serial.println("✅ DFPlayer Mini online.");

  // ---------- CAMERA INIT (do this after DFPlayer) ----------
  camera.pinout.aithinker();
  camera.brownout.disable();
  camera.resolution.yolo();
  camera.pixformat.rgb565();

  while (!camera.begin().isOk()) {
    Serial.println(camera.exception.toString());
    delay(200);
  }

  Serial.println("Camera OK");
  Serial.println("Put object in front of camera");
}

void loop() {
  // capture picture
  if (!camera.capture().isOk()) {
    Serial.println(camera.exception.toString());
    return;
  }

  // run FOMO
  if (!fomo.run().isOk()) {
    Serial.println(fomo.exception.toString());
    return;
  }

  Serial.printf("Found %d object(s) in %dms\n",
                fomo.count(),
                fomo.benchmark.millis());

  if (!fomo.foundAnyObject()) return;

  // Print safely (don't dereference null pointers)
  Serial.print("Found ");
  safePrintLabel(fomo.first.label.c_str());
  Serial.printf(" at (x = %d, y = %d) (size %d x %d)\n",
                fomo.first.x,
                fomo.first.y,
                fomo.first.width,
                fomo.first.height);

  // ---------- SAFE AUDIO PLAYBACK ----------
  uint32_t freeHeap = ESP.getFreeHeap();
  Serial.printf("Free heap before playback: %u bytes\n", freeHeap);

  // small pause to help camera free resources / mutex
  delay(50);

  // if (freeHeap > 45000) {
  //   player.play(4);
  //   Serial.println("🎧 DFPlayer: Playing track 4");
  // } else {
  //   Serial.println("⚠️ Low heap detected — skipping audio playback");
  // }

  if (freeHeap > 45000) {
  if (strcmp(fomo.first.label.c_str(), "200takafront") == 0) {
    player.play(2);
    Serial.println("🎧 DFPlayer: Playing track 2 (200 Taka Front)");
  } 
  else if (strcmp(fomo.first.label.c_str(), "5taka_front") == 0) {
    player.play(1);
    Serial.println("🎧 DFPlayer: Playing track 1 (5 Taka Front)");
  } 
  else {
    Serial.println("ℹ️ Label not recognized — no audio played");
  }
  } else {
    Serial.println("⚠️ Low heap detected — skipping audio playback");
  }

  // allow DFPlayer to start without us immediately capturing again
  delay(4000);
}
