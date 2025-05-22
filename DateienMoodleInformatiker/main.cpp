#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "SD_MMC.h"

// ==== Kamera-Pins für AI-Thinker ESP32-CAM ====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==== WLAN-Zugang ====
const char* ssid = "";
const char* password = "";

// ==== Webserver auf Port 80 ====
WebServer server(80);

int image_counter = 0;

bool saveImageToSD(camera_fb_t * fb) {
  if (!fb || fb->len == 0) return false;

  String path = "/img_" + String(image_counter++) + ".jpg";
  fs::FS &fs = SD_MMC;
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("SD-Datei konnte nicht geöffnet werden");
    return false;
  }

  file.write(fb->buf, fb->len);
  file.close();
  Serial.println("Bild gespeichert: " + path);
  return true;
}

void startCameraServer() {
  server.on("/capture", HTTP_GET, []() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      server.send(500, "text/plain", "Kamerazugriff fehlgeschlagen");
      return;
    }

    // Bild auf SD speichern
    if (!saveImageToSD(fb)) {
      Serial.println("Bild konnte nicht gespeichert werden");
    }

    // Bild als HTTP-Antwort senden
    server.sendHeader("Content-Type", "image/jpeg");
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ==== Kamera-Konfiguration ====
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_SVGA; // 800x600
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // ==== Kamera starten ====
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamerafehler: 0x%x\n", err);
    return;
  }

  // ==== Kamera-Einstellungen ====
  sensor_t * s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_sharpness(s, 1);

  // ==== WLAN verbinden ====
  WiFi.begin(ssid, password);
  Serial.print("Verbinde mit WLAN");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nVerbunden!");
  Serial.println(WiFi.localIP());

  // ==== SD-Karte mounten ====
  if (!SD_MMC.begin()) {
    Serial.println("Fehler beim Mounten der SD-Karte");
    return;
  }
  Serial.println("SD-Karte bereit");

  // ==== Webserver starten ====
  startCameraServer();
  Serial.println("Webserver bereit: /capture");
}

void loop() {
  server.handleClient();
}
