// ============================================================================
// ESP32-CAM Live Stream + Snapshot + Motion Detection
// Routes:
//   /         -> HTML control page
//   /stream   -> MJPEG live video
//   /capture  -> Single JPEG snapshot
//   /status   -> JSON: { "motion": true/false, "uptime": ms }
// ============================================================================

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"

// ===== WiFi Credentials =====
const char* ssid     = "YourWiFiName";
const char* password = "YourWiFiPassword";

// ===== AI-Thinker Pin Map =====
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

// On-board flash LED (AI-Thinker). Used as motion indicator.
// WARNING: this LED is BRIGHT. Comment out FLASH_LED_PIN logic if unwanted.
#define FLASH_LED_PIN      4

// ===== MJPEG stream constants =====
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;

// ============================================================================
// Motion Detection State
// We sample a small grayscale "fingerprint" of each frame and compare it to
// the previous one. If enough pixels changed by more than a threshold, that's
// motion. This is crude but uses almost no memory and no extra libraries.
// ============================================================================
#define MOTION_GRID_W      16    // sample 16 columns
#define MOTION_GRID_H      12    // sample 12 rows  (16*12 = 192 samples)
#define MOTION_PIXEL_DIFF  25    // per-pixel brightness diff to count as "changed" (0-255)
#define MOTION_PIXEL_PCT   10    // % of sampled pixels that must change to trigger
#define MOTION_COOLDOWN_MS 3000  // don't re-trigger for this long after motion

uint8_t  prev_samples[MOTION_GRID_W * MOTION_GRID_H] = {0};
bool     prev_samples_valid = false;
volatile bool motion_detected = false;
unsigned long last_motion_ms = 0;

// ============================================================================
// HTML Landing Page
// Plain HTML/CSS/JS, served as one big string. Shows the live stream plus
// buttons for snapshot and a tiny motion indicator polled from /status.
// ============================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-CAM</title>
  <style>
    body { font-family: system-ui, sans-serif; background:#111; color:#eee;
           margin:0; padding:20px; text-align:center; }
    h1 { margin:0 0 16px; font-weight:500; }
    img { max-width:100%; border-radius:8px; box-shadow:0 4px 20px rgba(0,0,0,.5); }
    .controls { margin-top:16px; }
    button, a.btn {
      background:#2a7; color:#fff; border:none; padding:10px 18px;
      margin:4px; border-radius:6px; font-size:16px; cursor:pointer;
      text-decoration:none; display:inline-block;
    }
    button:hover, a.btn:hover { background:#3b8; }
    #motion {
      display:inline-block; width:12px; height:12px; border-radius:50%;
      background:#444; margin-right:6px; vertical-align:middle;
    }
    #motion.active { background:#f33; box-shadow:0 0 8px #f33; }
    .status { margin-top:12px; font-size:14px; color:#aaa; }
  </style>
</head>
<body>
  <h1>ESP32-CAM Live Feed</h1>
  <img id="stream" src="/stream" alt="Live stream">
  <div class="controls">
    <a class="btn" href="/capture" target="_blank">Snapshot</a>
    <button onclick="document.getElementById('stream').src='/stream?'+Date.now()">
      Reload Stream
    </button>
  </div>
  <div class="status">
    <span id="motion"></span><span id="motionText">No motion</span>
  </div>
  <script>
    // Poll /status once a second for motion state.
    async function poll() {
      try {
        const r = await fetch('/status');
        const j = await r.json();
        const dot = document.getElementById('motion');
        const txt = document.getElementById('motionText');
        if (j.motion) { dot.classList.add('active'); txt.textContent = 'Motion!'; }
        else          { dot.classList.remove('active'); txt.textContent = 'No motion'; }
      } catch(e) {}
    }
    setInterval(poll, 1000);
  </script>
</body>
</html>
)rawliteral";

// ============================================================================
// Motion Detection Helper
// Decodes the JPEG to grayscale samples, compares with previous frame.
// Called on a low-rate timer (not every frame -- too expensive).
// ============================================================================
void check_motion() {
  // Grab a frame just for analysis
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  // Decode JPEG -> RGB888 buffer (small allocation since we'll downsample)
  // We use the camera's reported width/height to find sampling positions.
  uint8_t *rgb_buf = (uint8_t *)malloc(fb->width * fb->height * 3);
  if (!rgb_buf) {
    esp_camera_fb_return(fb);
    return;
  }

  bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf);
  esp_camera_fb_return(fb);   // Return frame buffer ASAP

  if (!ok) {
    free(rgb_buf);
    return;
  }

  // Sample on a grid: convert each sampled RGB pixel to luma (brightness)
  uint8_t samples[MOTION_GRID_W * MOTION_GRID_H];
  int step_x = fb->width  / MOTION_GRID_W;
  int step_y = fb->height / MOTION_GRID_H;

  for (int gy = 0; gy < MOTION_GRID_H; gy++) {
    for (int gx = 0; gx < MOTION_GRID_W; gx++) {
      int px = gx * step_x;
      int py = gy * step_y;
      int idx = (py * fb->width + px) * 3;
      uint8_t r = rgb_buf[idx];
      uint8_t g = rgb_buf[idx + 1];
      uint8_t b = rgb_buf[idx + 2];
      // Quick luma approximation: (R + 2G + B) / 4
      samples[gy * MOTION_GRID_W + gx] = (r + (g << 1) + b) >> 2;
    }
  }
  free(rgb_buf);

  // Compare with previous fingerprint
  if (prev_samples_valid) {
    int changed = 0;
    int total   = MOTION_GRID_W * MOTION_GRID_H;
    for (int i = 0; i < total; i++) {
      int diff = (int)samples[i] - (int)prev_samples[i];
      if (diff < 0) diff = -diff;
      if (diff > MOTION_PIXEL_DIFF) changed++;
    }
    int pct = (changed * 100) / total;

    unsigned long now = millis();
    if (pct >= MOTION_PIXEL_PCT && (now - last_motion_ms) > MOTION_COOLDOWN_MS) {
      motion_detected = true;
      last_motion_ms  = now;
      Serial.printf("MOTION! %d%% of pixels changed\n", pct);
      digitalWrite(FLASH_LED_PIN, HIGH);   // Flash on
    } else if (motion_detected && (now - last_motion_ms) > MOTION_COOLDOWN_MS) {
      motion_detected = false;
      digitalWrite(FLASH_LED_PIN, LOW);    // Flash off
    }
  }

  // Save current samples as the new baseline
  memcpy(prev_samples, samples, sizeof(samples));
  prev_samples_valid = true;
}

// ============================================================================
// Handler: GET / -- serves the HTML control page
// ============================================================================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
// Handler: GET /capture -- single JPEG snapshot
// ============================================================================
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ============================================================================
// Handler: GET /status -- tiny JSON for the page's motion indicator
// ============================================================================
static esp_err_t status_handler(httpd_req_t *req) {
  char buf[96];
  snprintf(buf, sizeof(buf),
           "{\"motion\":%s,\"uptime\":%lu}",
           motion_detected ? "true" : "false",
           (unsigned long)millis());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
// Handler: GET /stream -- MJPEG live video
// (Same logic as before, just moved to its own URI)
// ============================================================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t  _jpg_buf_len = 0;
  uint8_t *_jpg_buf    = NULL;
  char    part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) res = ESP_FAIL;
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf     = fb->buf;
      }
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    if (fb)            { esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL; }
    else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }
    if (res != ESP_OK) break;
  }
  return res;
}

// ============================================================================
// Register all four URIs
// ============================================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port    = 80;
  config.max_uri_handlers = 8;   // we have 4, default of 8 is plenty

  httpd_uri_t index_uri   = { "/",        HTTP_GET, index_handler,   NULL };
  httpd_uri_t stream_uri  = { "/stream",  HTTP_GET, stream_handler,  NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };
  httpd_uri_t status_uri  = { "/status",  HTTP_GET, status_handler,  NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
  }
}

// ============================================================================
// setup() -- camera + WiFi + server bring-up
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Camera config (same as before)
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  // WiFi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("Camera Ready! Visit: http://");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

// ============================================================================
// loop() -- runs motion detection every ~500 ms.
// We do this here (rather than per-frame in the stream) so it doesn't slow
// down the live feed for connected viewers.
// ============================================================================
void loop() {
  static unsigned long last_check = 0;
  unsigned long now = millis();
  if (now - last_check >= 500) {
    last_check = now;
    check_motion();
  }
  delay(10);
}