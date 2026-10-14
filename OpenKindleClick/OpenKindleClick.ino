// Open Kindle Click: Seeed Studio XIAO ESP32C6 firmware that a Kindle accepts
// as a Kindle Click page turner. PROTOCOL.md in Open-Kindle-Click-nRF52840
// describes where each requirement comes from.
//
// Buttons (to GND, internal pull-ups), laid out like the Kindle Click that the
// Kindle's own UI describes (pairing dialog; Settings > Accessories > remote):
//   D0 (GPIO0)  = Button 1, top face      -> page forward   (KEY_PAGEDOWN)
//   D1 (GPIO1)  = Button 2, bottom face   -> page backward  (KEY_PAGEUP)
//   D2 (GPIO2)  = Button 3, upper side    -> brightness up  (held: repeats)
//   D3 (GPIO21) = Button 4, lower side    -> brightness down (held: repeats)
//   D2 + D3 held 1 s                      -> Kindle sleeps (KEY_POWER), the
//                                            Kindle's documented "long press
//                                            both side buttons"
//   D0 + D1 held 10 s                     -> forget bonds, pairing mode. Also
//                                            Forget the remote on the Kindle
//                                            (Bluetooth devices).
//
// USB serial (115200) accepts test commands; send "h" for the list.

#include <stdarg.h>
#include <string.h>

#include <esp_bt.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_store.h>
#include <host/ble_uuid.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

extern "C" void ble_store_config_init(void);

#define FW_VERSION   "0.3.1"
#define DEVICE_NAME  "Open Kindle Click"

// ---- Identity the Kindle checks -------------------------------------------
// ADV_IND raw payload:
//   0x01 Flags = 0x06 (LE General Discoverable | BR/EDR Not Supported)
//   0xFF Manufacturer Specific = company 0x0171 (LE) + PID 0x0443 (BE)
//   0x03 Complete 16-bit Service UUIDs = 0x1812 (HID)
// Flags and 71 01 04 43 must sit in the ADV_IND itself (not only the scan
// response), and 71 01 04 43 must be the first 0xFF AD.
static const uint8_t ADV_DATA[] = {
  0x02, 0x01, 0x06,
  0x05, 0xFF, 0x71, 0x01, 0x04, 0x43,
  0x03, 0x03, 0x12, 0x18,
};

// DIS PnP ID: source 0x01 (Bluetooth SIG), VID 0x0171, PID 0x0443, version
// 0x0100. HOGP hands VID/PID to uhid; input_manager matches on them.
static const uint8_t PNP_ID[7] = { 0x01, 0x71, 0x01, 0x43, 0x04, 0x00, 0x01 };

// HID Information: bcdHID 0x0111, bCountryCode 0x00, Flags 0x03 (RemoteWake |
// NormallyConnectable).
static const uint8_t HID_INFO[4] = { 0x11, 0x01, 0x00, 0x03 };

// Report Reference descriptor for Input Report 1: Report ID 1, type Input (1).
static const uint8_t REPORT_REF_IN1[2] = { 0x01, 0x01 };

static const char DIS_MFG[]   = "Open Kindle Click project";
static const char DIS_MODEL[] = "XIAO ESP32C6";
static const char DIS_FW[]    = "ESP32-C6 " FW_VERSION;

// ---- Pins ------------------------------------------------------------------
enum ButtonId { BTN_NEXT, BTN_PREV, BTN_BRI_UP, BTN_BRI_DOWN, BTN_COUNT };
static const uint8_t BUTTON_PINS[BTN_COUNT] = { D0, D1, D2, D3 };

// XIAO ESP32C6 orange user LED on GPIO15 (LED_BUILTIN) is active-low.
#define LED_ON  LOW
#define LED_OFF HIGH

// ---- Timing ----------------------------------------------------------------
static const uint32_t DEBOUNCE_MS         = 15;
static const uint32_t SIDE_CHORD_MS       = 150;    // 2nd side button within this: no brightness
static const uint32_t SLEEP_HOLD_MS       = 1000;   // both side buttons -> KEY_POWER
static const uint32_t PAIR_HOLD_MS        = 10000;  // both face buttons -> forget bonds
static const uint32_t MIN_PRESS_MS        = 30;
static const uint32_t PENDING_TTL_MS      = 10000;
static const uint32_t READY_QUIET_MS      = 800;
static const uint32_t READY_FALLBACK_MS   = 5000;
static const uint32_t CONN_PARAM_DELAY_MS = 1500;   // after ready
static const uint16_t ADV_FAST_INTERVAL   = 32;     // 20 ms (in 0.625 ms units)
static const uint16_t ADV_SLOW_INTERVAL   = 244;    // 152.5 ms (in 0.625 ms units)
static const uint32_t ADV_FAST_TIMEOUT_MS = 30000;  // pairing mode slows down after 30 s
static const uint32_t BATTERY_PERIOD_MS   = 60000;
static const uint32_t BATTERY_RENOTIFY_MS = 15UL * 60 * 1000;
static const uint8_t  BATTERY_LOW_PCT     = 20;
static const uint32_t DORMANT_AFTER_MS    = 5UL * 60 * 1000;  // bonded, on battery

// ---- HID -------------------------------------------------------------------
// One 10-byte input report (ID 1): the boot-keyboard layout, then one consumer
// usage. One application collection so Linux hid-generic makes one evdev node
// with EV_REP (from the keyboard usages), allowing held brightness to repeat.
static const uint8_t REPORT_MAP[] = {
  0x05, 0x01,                   // Usage Page (Generic Desktop)
  0x09, 0x06,                   // Usage (Keyboard)
  0xA1, 0x01,                   // Collection (Application)
  0x85, 0x01,                   //   Report ID (1)
  0x05, 0x07,                   //   Usage Page (Keyboard/Keypad)
  0x19, 0xE0, 0x29, 0xE7,       //   Usage Min/Max (modifiers)
  0x15, 0x00, 0x25, 0x01,       //   Logical Min/Max (0..1)
  0x75, 0x01, 0x95, 0x08,       //   Report Size 1, Count 8
  0x81, 0x02,                   //   Input (Data, Var, Abs)
  0x75, 0x08, 0x95, 0x01,       //   Report Size 8, Count 1
  0x81, 0x01,                   //   Input (Const): reserved byte
  0x19, 0x00, 0x29, 0x65,       //   Usage Min/Max (0..0x65)
  0x15, 0x00, 0x25, 0x65,       //   Logical Min/Max (0..0x65)
  0x75, 0x08, 0x95, 0x06,       //   Report Size 8, Count 6
  0x81, 0x00,                   //   Input (Data, Array, Abs)
  0x05, 0x0C,                   //   Usage Page (Consumer)
  0x19, 0x00, 0x2A, 0xFF, 0x03, //   Usage Min/Max (0..0x3FF)
  0x15, 0x00, 0x26, 0xFF, 0x03, //   Logical Min/Max (0..0x3FF)
  0x75, 0x10, 0x95, 0x01,       //   Report Size 16, Count 1
  0x81, 0x00,                   //   Input (Data, Array, Abs)
  0xC0                          // End Collection
};

enum KeyId { KEY_NEXT, KEY_PREV, KEY_BRI_UP, KEY_BRI_DOWN, KEY_SLEEP, KEY_COUNT, KEY_NONE = 0xFF };

struct KeyDef {
  const char* name;
  bool        consumer;   // false = keyboard page
  uint16_t    usage;
};

static const KeyDef KEYS[KEY_COUNT] = {
  { "page forward (KEY_PAGEDOWN)",          false, 0x4E },
  { "page backward (KEY_PAGEUP)",           false, 0x4B },
  { "brightness up (KEY_BRIGHTNESSUP)",     true,  0x006F },
  { "brightness down (KEY_BRIGHTNESSDOWN)", true,  0x0070 },
  { "sleep (KEY_POWER)",                    true,  0x0030 },
};

// ---- Button pairs ----------------------------------------------------------
enum ChordState { CH_IDLE, CH_WAIT, CH_SINGLE, CH_CHORD, CH_DONE };

struct ChordPair {
  ButtonId   a, b;
  KeyId      key_a, key_b;
  uint32_t   window;    // chord window before a lone press counts
  bool       rollover;  // releasing one of a chord restarts the other as a press
  ChordState st;
  ButtonId   first;
  uint32_t   since;     // CH_WAIT: first press; CH_CHORD: both down
};

struct Button {
  bool     raw;
  bool     pressed;
  uint32_t raw_since;
};

// ---- State -----------------------------------------------------------------
static volatile bool     g_ble_synced = false;
static uint8_t           g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint16_t          g_report_handle = 0;
static uint16_t          g_battery_handle = 0;
static uint8_t           g_last_report[10] = { 0 };
static uint8_t           g_hid_ctrl_point = 0;

static volatile uint16_t g_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool     g_secured = false;
static volatile uint32_t g_connected_at = 0;
static volatile uint32_t g_secured_at = 0;
static volatile bool     g_hid_notify_en = false;
static volatile bool     g_bat_notify_en = false;
static volatile uint16_t g_adv_interval = ADV_FAST_INTERVAL;
static volatile bool     g_auto_adv = true;

// Readiness evidence, written in the NimBLE host task (GAP event callback).
static volatile uint32_t g_cccd_at = 0;        // last notification-enabling CCCD write
static volatile uint8_t  g_cccd_writes = 0;
static volatile uint32_t g_disc_done_at = 0;   // last link change: left 7.5 ms, or 20 ms with no latency
static volatile bool     g_fast_interval = false;
static bool              g_ready = false;      // latched per connection in loop()
static uint32_t          g_ready_at = 0;
static bool              g_params_requested = false;

static volatile bool     g_dormant = false;
static volatile uint32_t g_last_activity = 0;
static uint8_t  g_battery = 100;
static uint16_t g_battery_mv = 4200;
static bool     g_battery_sent = false;        // notified on this connection
static uint32_t g_battery_sent_cccd = 0;       // g_cccd_at when we notified
static uint32_t g_battery_sent_at = 0;

static bool     g_down[KEY_COUNT];             // key state the host should see
static uint32_t g_down_at[KEY_COUNT];
static bool     g_release_wanted[KEY_COUNT];
static bool     g_dirty = false;               // report not yet accepted by the stack

static KeyId    g_pending = KEY_NONE;          // page turn queued while disconnected
static uint32_t g_pending_at = 0;

static bool     g_bonds_cached = false;
static uint32_t g_flash_at = 0;                // last key feedback flash

static Button    g_btn[BTN_COUNT];
static ChordPair g_face = { BTN_NEXT,   BTN_PREV,     KEY_NEXT,   KEY_PREV,     0,             true,  CH_IDLE, BTN_NEXT,   0 };
static ChordPair g_side = { BTN_BRI_UP, BTN_BRI_DOWN, KEY_BRI_UP, KEY_BRI_DOWN, SIDE_CHORD_MS, false, CH_IDLE, BTN_BRI_UP, 0 };

// ---- Small helpers ---------------------------------------------------------
#define LOG_RING_SIZE 16
static char     g_log_ring[LOG_RING_SIZE][128];
static uint32_t g_log_time[LOG_RING_SIZE];
static uint32_t g_log_count = 0;

static void logmsg(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof buf - 1) n = sizeof buf - 1;
  if (buf[0] == '[') {
    uint32_t idx = g_log_count++ % LOG_RING_SIZE;
    g_log_time[idx] = millis();
    strncpy(g_log_ring[idx], buf, sizeof(g_log_ring[0]) - 1);
    g_log_ring[idx][sizeof(g_log_ring[0]) - 1] = '\0';
  }
  Serial.write((const uint8_t*)buf, n);
}

static void printLogRing() {
  uint32_t total = g_log_count;
  uint32_t start = total > LOG_RING_SIZE ? total - LOG_RING_SIZE : 0;
  logmsg("---- recent events (%lu) ----\n", (unsigned long)total);
  for (uint32_t i = start; i < total; i++) {
    uint32_t idx = i % LOG_RING_SIZE;
    logmsg("  +%6lu ms  %s", (unsigned long)g_log_time[idx], g_log_ring[idx]);
  }
}

static bool usbPowered() {
  static bool seen_usb = false;
  if (Serial.isPlugged()) seen_usb = true;
  return seen_usb;
}

static bool connected() {
  return g_conn != BLE_HS_CONN_HANDLE_NONE;
}

static void setLed(bool on) {
  digitalWrite(LED_BUILTIN, on ? LED_ON : LED_OFF);
}

static bool hasBonds() {
  if (!g_ble_synced) return false;
  int count = 0;
  if (ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &count) == 0 && count > 0) return true;
  if (ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count) == 0 && count > 0) return true;
  return false;
}

// ---- GATT server definitions -----------------------------------------------
static const ble_uuid16_t UUID_SVC_HID       = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t UUID_CHR_REPORT    = BLE_UUID16_INIT(0x2A4D);
static const ble_uuid16_t UUID_DSC_RPT_REF   = BLE_UUID16_INIT(0x2908);
static const ble_uuid16_t UUID_CHR_RPT_MAP   = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t UUID_CHR_HID_INFO  = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t UUID_CHR_HID_CTRL  = BLE_UUID16_INIT(0x2A4C);

static const ble_uuid16_t UUID_SVC_BAS       = BLE_UUID16_INIT(0x180F);
static const ble_uuid16_t UUID_CHR_BAT_LEVEL = BLE_UUID16_INIT(0x2A19);

static const ble_uuid16_t UUID_SVC_DIS       = BLE_UUID16_INIT(0x180A);
static const ble_uuid16_t UUID_CHR_DIS_MODEL = BLE_UUID16_INIT(0x2A24);
static const ble_uuid16_t UUID_CHR_DIS_FW    = BLE_UUID16_INIT(0x2A26);
static const ble_uuid16_t UUID_CHR_DIS_MFG   = BLE_UUID16_INIT(0x2A29);
static const ble_uuid16_t UUID_CHR_DIS_PNPID = BLE_UUID16_INIT(0x2A50);

static int onGattAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
  uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
    uuid16 = ble_uuid_u16(ctxt->dsc->uuid);
    if (uuid16 == 0x2908) {
      return os_mbuf_append(ctxt->om, REPORT_REF_IN1, sizeof REPORT_REF_IN1) == 0
                 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (uuid16 == 0x2A4C && OS_MBUF_PKTLEN(ctxt->om) >= 1) {
      os_mbuf_copydata(ctxt->om, 0, 1, &g_hid_ctrl_point);
      return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

  const void* data = NULL;
  size_t len = 0;
  switch (uuid16) {
    case 0x2A4D: data = g_last_report; len = sizeof g_last_report; break;
    case 0x2A4B: data = REPORT_MAP;    len = sizeof REPORT_MAP;    break;
    case 0x2A4A: data = HID_INFO;      len = sizeof HID_INFO;      break;
    case 0x2A19: data = &g_battery;    len = sizeof g_battery;     break;
    case 0x2A24: data = DIS_MODEL;     len = strlen(DIS_MODEL);    break;
    case 0x2A26: data = DIS_FW;        len = strlen(DIS_FW);       break;
    case 0x2A29: data = DIS_MFG;       len = strlen(DIS_MFG);      break;
    case 0x2A50: data = PNP_ID;        len = sizeof PNP_ID;        break;
    default:     return BLE_ATT_ERR_UNLIKELY;
  }
  return os_mbuf_append(ctxt->om, data, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static struct ble_gatt_dsc_def HID_REPORT_DSCS[] = {
  {
    .uuid = &UUID_DSC_RPT_REF.u,
    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
    .min_key_size = 0,
    .access_cb = onGattAccess,
    .arg = NULL,
  },
  { 0 }
};

static const struct ble_gatt_chr_def HID_CHRS[] = {
  {
    .uuid = &UUID_CHR_REPORT.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = HID_REPORT_DSCS,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
    .min_key_size = 0,
    .val_handle = &g_report_handle,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_RPT_MAP.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_HID_INFO.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_HID_CTRL.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  { 0 }
};

static const struct ble_gatt_chr_def BAS_CHRS[] = {
  {
    .uuid = &UUID_CHR_BAT_LEVEL.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    .min_key_size = 0,
    .val_handle = &g_battery_handle,
    .cpfd = NULL,
  },
  { 0 }
};

static const struct ble_gatt_chr_def DIS_CHRS[] = {
  {
    .uuid = &UUID_CHR_DIS_MFG.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_DIS_MODEL.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_DIS_FW.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  {
    .uuid = &UUID_CHR_DIS_PNPID.u,
    .access_cb = onGattAccess,
    .arg = NULL,
    .descriptors = NULL,
    .flags = BLE_GATT_CHR_F_READ,
    .min_key_size = 0,
    .val_handle = NULL,
    .cpfd = NULL,
  },
  { 0 }
};

// HID first, so later DIS/BAS changes never move the HID handles the Kindle
// keeps for a session.
static const struct ble_gatt_svc_def GATT_SVCS[] = {
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &UUID_SVC_HID.u,
    .includes = NULL,
    .characteristics = HID_CHRS,
  },
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &UUID_SVC_BAS.u,
    .includes = NULL,
    .characteristics = BAS_CHRS,
  },
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &UUID_SVC_DIS.u,
    .includes = NULL,
    .characteristics = DIS_CHRS,
  },
  { 0 }
};

// ---- Link readiness --------------------------------------------------------
static bool linkReady() {
  if (!connected() || !g_secured) return false;
  if (g_ready) return true;
  uint32_t now = millis();
  uint32_t c = g_cccd_at, d = g_disc_done_at;
  uint32_t last = !c ? d : !d ? c : (int32_t)(c - d) > 0 ? c : d;
  bool quiet = last && !g_fast_interval && (int32_t)(now - last) >= (int32_t)READY_QUIET_MS;
  if (quiet || now - g_secured_at >= READY_FALLBACK_MS) {
    g_ready = true;
    g_ready_at = now;
    uint32_t t0 = g_connected_at;
    char disc[24] = "no link change", cccd[32] = "no CCCD writes";
    if (d) snprintf(disc, sizeof disc, "link change +%lu", (unsigned long)(d - t0));
    if (c) snprintf(cccd, sizeof cccd, "%u CCCD writes, last +%lu", g_cccd_writes, (unsigned long)(c - t0));
    logmsg("[link] +%lu ms ready for keys (%s): encrypted +%lu, %s, %s\n",
           (unsigned long)(now - t0), quiet ? "host settled" : "fallback",
           (unsigned long)(g_secured_at - t0), disc, cccd);
  }
  return g_ready;
}

// ---- HID reports -----------------------------------------------------------
static bool sendReport() {
  uint16_t conn = g_conn;
  if (conn == BLE_HS_CONN_HANDLE_NONE || !g_hid_notify_en) return false;
  uint8_t report[10] = { 0 };     // modifiers, reserved, 6 keys, consumer usage
  uint8_t n = 2;
  uint16_t usage = 0;
  for (int k = 0; k < KEY_COUNT; k++) {
    if (!g_down[k]) continue;
    if (KEYS[k].consumer) usage = KEYS[k].usage;  // last one wins
    else if (n < 8)       report[n++] = (uint8_t)KEYS[k].usage;
  }
  report[8] = usage & 0xFF;
  report[9] = usage >> 8;
  memcpy(g_last_report, report, sizeof report);
  struct os_mbuf* om = ble_hs_mbuf_from_flat(report, sizeof report);
  if (!om) return false;
  return ble_gatts_notify_custom(conn, g_report_handle, om) == 0;
}

static void flushReports() {
  if (!connected()) { g_dirty = false; return; }
  if (g_dirty && sendReport()) g_dirty = false;
}

static void startAdvertisingWithInterval(uint16_t interval, int32_t duration_ms);
static void startAdvertising();

static void ensureFastAdvertising() {
  if (connected() || !g_ble_synced) return;
  if (ble_gap_adv_active() && g_adv_interval != ADV_FAST_INTERVAL) {
    ble_gap_adv_stop();
  }
  startAdvertising();
}

static void keyDown(KeyId k) {
  g_last_activity = millis();
  if (!linkReady()) {
    ensureFastAdvertising();
    if ((k == KEY_NEXT || k == KEY_PREV) && g_bonds_cached) {
      g_pending = k;
      g_pending_at = millis();
      logmsg("[key] %s queued until the Kindle is ready\n", KEYS[k].name);
    } else {
      logmsg("[key] %s ignored: not connected\n", KEYS[k].name);
    }
    return;
  }
  if (g_down[k]) {
    g_release_wanted[k] = false;
    return;
  }
  g_down[k] = true;
  g_down_at[k] = millis();
  g_release_wanted[k] = false;
  g_dirty = true;
  flushReports();
  g_flash_at = millis();
  logmsg("[key] %s down\n", KEYS[k].name);
}

static void keyUp(KeyId k) {
  if (g_down[k]) g_release_wanted[k] = true;
}

static void serviceReleases() {
  for (int k = 0; k < KEY_COUNT; k++) {
    if (g_down[k] && g_release_wanted[k] && millis() - g_down_at[k] >= MIN_PRESS_MS) {
      g_down[k] = false;
      g_release_wanted[k] = false;
      g_dirty = true;
      logmsg("[key] %s up\n", KEYS[k].name);
    }
  }
  flushReports();
}

static void tap(KeyId k) {
  keyDown(k);
  keyUp(k);
}

static void releaseAll() {
  bool any = false;
  for (int k = 0; k < KEY_COUNT; k++) {
    if (g_down[k]) any = true;
    g_down[k] = false;
    g_release_wanted[k] = false;
  }
  if (any) {
    g_dirty = true;
    flushReports();
  }
}

static void servicePending() {
  if (g_pending == KEY_NONE) return;
  if (millis() - g_pending_at > PENDING_TTL_MS) {
    logmsg("[key] dropped queued %s (the Kindle didn't reconnect: asleep?)\n", KEYS[g_pending].name);
    g_pending = KEY_NONE;
    return;
  }
  if (linkReady()) {
    KeyId k = g_pending;
    g_pending = KEY_NONE;
    tap(k);
  }
}

// ---- Battery ---------------------------------------------------------------
static uint8_t measureBattery() {
  // XIAO ESP32C6 has a built-in LiPo charger (SGM40567) on the BAT pads, without
  // an internal ADC divider wired to a GPIO. Report 100%.
  g_battery_mv = 4200;
  return 100;
}

static void notifyBattery(const char* why) {
  uint16_t conn = g_conn;
  if (conn == BLE_HS_CONN_HANDLE_NONE || !g_bat_notify_en) return;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(&g_battery, sizeof g_battery);
  if (!om) return;
  if (ble_gatts_notify_custom(conn, g_battery_handle, om) != 0) return;
  g_battery_sent = true;
  g_battery_sent_cccd = g_cccd_at;
  g_battery_sent_at = millis();
  logmsg("[bat] %u%% notified (%s)\n", g_battery, why);
}

static void serviceBattery() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (last == 0 || now - last >= BATTERY_PERIOD_MS) {
    last = now;
    uint8_t pct = measureBattery();
    bool changed = pct != g_battery;
    g_battery = pct;
    if (changed && linkReady()) notifyBattery("changed");
  }
  if (!linkReady()) return;
  uint32_t c = g_cccd_at;
  const char* why = NULL;
  if (!g_battery_sent)                                              why = "connected";
  else if (c != g_battery_sent_cccd && (int32_t)(now - c) >= 1000)  why = "subscribed";
  else if (now - g_battery_sent_at >= BATTERY_RENOTIFY_MS)          why = "periodic";
  if (why) notifyBattery(why);
}

// ---- BLE -------------------------------------------------------------------
static int onGapEvent(struct ble_gap_event* event, void* arg);

static void startAdvertisingWithInterval(uint16_t interval, int32_t duration_ms) {
  if (!g_ble_synced || connected()) return;
  g_dormant = false;
  g_last_activity = millis();
  if (ble_gap_adv_active()) return;

  int rc = ble_gap_adv_set_data(ADV_DATA, sizeof ADV_DATA);
  if (rc != 0) {
    logmsg("[ble] adv data set failed (rc=%d)\n", rc);
    return;
  }

  struct ble_hs_adv_fields rsp = {};
  rsp.name = (const uint8_t*)DEVICE_NAME;
  rsp.name_len = strlen(DEVICE_NAME);
  rsp.name_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&rsp);
  if (rc != 0) {
    logmsg("[ble] scan rsp set failed (rc=%d)\n", rc);
    return;
  }

  struct ble_gap_adv_params adv_params = {};
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = interval;
  adv_params.itvl_max = interval;

  g_adv_interval = interval;
  rc = ble_gap_adv_start(g_own_addr_type, NULL, duration_ms, &adv_params, onGapEvent, NULL);
  if (rc != 0) {
    logmsg("[ble] advertising failed to start (rc=%d)\n", rc);
    return;
  }
  if (interval == ADV_FAST_INTERVAL) {
    logmsg(g_bonds_cached ? "[ble] advertising (waiting for bonded Kindle)\n"
                          : "[ble] advertising in pairing mode\n");
  }
}

static void startAdvertising() {
  bool bonded = hasBonds();
  g_bonds_cached = bonded;
  startAdvertisingWithInterval(ADV_FAST_INTERVAL,
                               (bonded || usbPowered()) ? BLE_HS_FOREVER : ADV_FAST_TIMEOUT_MS);
}

static void enterDormant() {
  g_auto_adv = false;
  if (ble_gap_adv_active()) ble_gap_adv_stop();
  g_dormant = true;
  setLed(false);
  logmsg("[pwr] dormant: advertising stopped, any button wakes\n");
}

static void dropLink() {
  if (!connected()) return;
  releaseAll();
  delay(100);
  uint16_t conn = g_conn;
  if (conn != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
  }
  uint32_t t0 = millis();
  while (connected() && millis() - t0 < 3000) delay(10);
}

static void forgetBonds() {
  logmsg("[ble] forgetting all bonds (also Forget the remote on the Kindle)\n");
  g_pending = KEY_NONE;
  g_auto_adv = false;
  if (ble_gap_adv_active()) ble_gap_adv_stop();
  dropLink();
  ble_store_clear();
  g_bonds_cached = false;
  g_auto_adv = true;
  startAdvertising();
}

static int onGapEvent(struct ble_gap_event* event, void* arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
      if (event->connect.status != 0) {
        if (g_auto_adv) startAdvertising();
        return 0;
      }
      g_connected_at = millis();
      g_cccd_at = 0;
      g_cccd_writes = 0;
      g_disc_done_at = 0;
      g_secured = false;
      g_ready = false;
      g_params_requested = false;
      g_battery_sent = false;
      g_hid_notify_en = false;
      g_bat_notify_en = false;
      g_last_activity = millis();
      g_conn = event->connect.conn_handle;

      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
        g_fast_interval = (desc.conn_itvl == 6);
        const uint8_t* a = desc.peer_ota_addr.val;
        logmsg("[ble] connected to %02X:%02X:%02X:%02X:%02X:%02X\n",
               a[5], a[4], a[3], a[2], a[1], a[0]);
      } else {
        g_fast_interval = false;
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
      g_conn = BLE_HS_CONN_HANDLE_NONE;
      g_secured = false;
      g_ready = false;
      g_hid_notify_en = false;
      g_bat_notify_en = false;
      for (int k = 0; k < KEY_COUNT; k++) {
        g_down[k] = false;
        g_release_wanted[k] = false;
      }
      g_dirty = false;
      g_last_activity = millis();
      uint8_t hci_reason = (uint8_t)(event->disconnect.reason & 0xFF);
      logmsg("[ble] disconnected (reason 0x%02X)\n", hci_reason);
      if (g_auto_adv && !g_dormant) {
        startAdvertising();
      }
      return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE: {
      if (event->conn_update.status == 0) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
          bool fast = (desc.conn_itvl == 6);
          if ((g_fast_interval && !fast) || (desc.conn_itvl == 16 && desc.conn_latency == 0)) {
            g_disc_done_at = millis() | 1;
          }
          g_fast_interval = fast;
        }
      }
      return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
      struct ble_gap_conn_desc desc;
      if (event->enc_change.status == 0 &&
          ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0 &&
          desc.sec_state.encrypted) {
        g_secured_at = millis();
        g_secured = true;
        g_bonds_cached = hasBonds() || desc.sec_state.bonded;
        logmsg("[ble] link encrypted (bonded=%d)\n", (int)desc.sec_state.bonded);
      }
      return 0;
    }

    case BLE_GAP_EVENT_PARING_COMPLETE: {
      uint8_t st = event->pairing_complete.status;
      if (st == 0) g_bonds_cached = true;
      logmsg("[ble] pairing %s (status 0x%02X)\n", st == 0 ? "succeeded" : "FAILED", st);
      return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE: {
      if (event->subscribe.attr_handle == g_report_handle) {
        g_hid_notify_en = event->subscribe.cur_notify;
      } else if (event->subscribe.attr_handle == g_battery_handle) {
        g_bat_notify_en = event->subscribe.cur_notify;
      }
      if (event->subscribe.reason == BLE_GAP_SUBSCRIBE_REASON_WRITE &&
          event->subscribe.cur_notify) {
        g_cccd_at = millis() | 1;
        g_cccd_writes++;
      }
      return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE: {
      if (event->adv_complete.reason == BLE_HS_ETIMEOUT && g_auto_adv && !g_dormant && !connected()) {
        startAdvertisingWithInterval(ADV_SLOW_INTERVAL, BLE_HS_FOREVER);
      }
      return 0;
    }

    default:
      return 0;
  }
}

static void onBleReset(int reason) {
  g_ble_synced = false;
  logmsg("[ble] host reset (reason=%d)\n", reason);
}

static void onBleSync() {
  uint8_t rnd_addr[6] = { 0 };
  esp_read_mac(rnd_addr, ESP_MAC_BT);
  // esp_read_mac returns big-endian; ble_hs_id_set_rnd takes little-endian with
  // the two MSBs of byte[5] set to 11 (Static Random address, matching nRF52840).
  uint8_t le_addr[6] = {
    rnd_addr[5], rnd_addr[4], rnd_addr[3],
    rnd_addr[2], rnd_addr[1], (uint8_t)(rnd_addr[0] | 0xC0)
  };
  if (ble_hs_id_set_rnd(le_addr) == 0) {
    g_own_addr_type = BLE_OWN_ADDR_RANDOM;
  } else {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &g_own_addr_type);
  }
  g_ble_synced = true;
  g_bonds_cached = hasBonds();
  startAdvertising();
}

static void bleHostTask(void* param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void setupBle() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(nimble_port_init());

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P6);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P6);

  ble_hs_cfg.reset_cb = onBleReset;
  ble_hs_cfg.sync_cb = onBleSync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  // Match Open-Kindle-Click-nRF52840 (Bluefruit LESC_SUPPORTED=0):
  // Just Works (NoInputNoOutput), bonding enabled, Legacy Pairing so the
  // peripheral distributes its LTK (PENC) without distributing an IRK.
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 0;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_gap_device_name_set(DEVICE_NAME);

  ESP_ERROR_CHECK(ble_gatts_count_cfg(GATT_SVCS));
  ESP_ERROR_CHECK(ble_gatts_add_svcs(GATT_SVCS));

  ble_store_config_init();
  nimble_port_freertos_init(bleHostTask);

  uint32_t t0 = millis();
  while (!g_ble_synced && millis() - t0 < 3000) {
    delay(5);
  }
}

static void serviceConnection() {
  if (!linkReady() || g_params_requested) return;
  if (millis() - g_ready_at < CONN_PARAM_DELAY_MS) return;
  g_params_requested = true;
  // 20 ms (what the Kindle uses), latency 4, 3 s supervision timeout instead
  // of the Kindle's 20 s, so a silently lost link is noticed quickly.
  struct ble_gap_upd_params params = {};
  params.itvl_min = 16;
  params.itvl_max = 16;
  params.latency = 4;
  params.supervision_timeout = 300;
  ble_gap_update_params(g_conn, &params);
}

// ---- Buttons ---------------------------------------------------------------
static ChordPair& pairOf(ButtonId b) { return (b == BTN_NEXT || b == BTN_PREV) ? g_face : g_side; }
static KeyId soloKey(const ChordPair& p, ButtonId b) { return b == p.a ? p.key_a : p.key_b; }

static void sleepAction() {
  if (!linkReady()) { logmsg("[key] sleep ignored: not connected\n"); return; }
  tap(KEY_SLEEP);
}

static void forgetAction() {
  releaseAll();
  forgetBonds();
}

static void startSingle(ChordPair& p, ButtonId b) {
  p.st = CH_SINGLE;
  p.first = b;
  keyDown(soloKey(p, b));
}

static void onPress(ButtonId b) {
  g_auto_adv = true;
  ChordPair& p = pairOf(b);
  switch (p.st) {
    case CH_IDLE:
      if (p.window == 0) startSingle(p, b);
      else { p.st = CH_WAIT; p.first = b; p.since = millis(); }
      break;
    case CH_SINGLE:
      keyUp(soloKey(p, p.first));
      // fall through
    case CH_WAIT:
    case CH_CHORD:
      p.st = CH_CHORD;
      p.since = millis();
      break;
    case CH_DONE:
      break;
  }
}

static void onRelease(ButtonId b) {
  ChordPair& p = pairOf(b);
  switch (p.st) {
    case CH_WAIT:
      if (b == p.first) { tap(soloKey(p, b)); p.st = CH_IDLE; }
      break;
    case CH_SINGLE:
      if (b == p.first) { keyUp(soloKey(p, b)); p.st = CH_IDLE; }
      break;
    case CH_CHORD:
    case CH_DONE:
      if (!g_btn[p.a].pressed && !g_btn[p.b].pressed) {
        p.st = CH_IDLE;
      } else if (p.st == CH_CHORD && p.rollover) {
        ButtonId still = (b == p.a) ? p.b : p.a;
        if (p.window == 0) startSingle(p, still);
        else { p.st = CH_WAIT; p.first = still; p.since = millis(); }
      }
      break;
    case CH_IDLE:
      break;
  }
}

static void pollPair(ChordPair& p, uint32_t hold_ms, void (*action)()) {
  uint32_t now = millis();
  if (p.st == CH_WAIT && now - p.since >= p.window) {
    startSingle(p, p.first);
  } else if (p.st == CH_CHORD && g_btn[p.a].pressed && g_btn[p.b].pressed &&
             now - p.since >= hold_ms) {
    p.st = CH_DONE;
    action();
  }
}

static void pollButtons() {
  uint32_t now = millis();
  for (int i = 0; i < BTN_COUNT; i++) {
    bool raw = digitalRead(BUTTON_PINS[i]) == LOW;
    if (raw != g_btn[i].raw) {
      g_btn[i].raw = raw;
      g_btn[i].raw_since = now;
    } else if (raw != g_btn[i].pressed && now - g_btn[i].raw_since >= DEBOUNCE_MS) {
      g_btn[i].pressed = raw;
      if (raw) onPress((ButtonId)i);
      else     onRelease((ButtonId)i);
    }
  }
  pollPair(g_side, SLEEP_HOLD_MS, sleepAction);
  pollPair(g_face, PAIR_HOLD_MS, forgetAction);
}

// ---- LED -------------------------------------------------------------------
static void updateLed() {
  uint32_t now = millis();
  bool low = g_battery < BATTERY_LOW_PCT;
  if (now - g_flash_at < 40) { setLed(true); return; }
  if (g_dormant)             { setLed(false); return; }

  if (connected()) {
    setLed(low && now % 5000 < 30);
  } else if (!g_bonds_cached) {
    setLed(now % 500 < 100);     // pairing mode: fast blink
  } else {
    setLed(now % 2000 < 30);     // waiting for bonded Kindle: short blip every 2 s
  }
}

// ---- Serial console --------------------------------------------------------
static void printStatus() {
  logmsg("---- Open Kindle Click " FW_VERSION " (ESP32-C6) ----\n");
  uint8_t a[6] = { 0 };
  ble_hs_id_copy_addr(g_own_addr_type, a, NULL);
  logmsg("address     %02X:%02X:%02X:%02X:%02X:%02X\n",
         a[5], a[4], a[3], a[2], a[1], a[0]);
  logmsg("identity    VID 0x0171 / PID 0x0443 (mfg data 71 01 04 43)\n");
  logmsg("connected   %s%s%s\n", connected() ? "yes" : "no",
         g_secured ? ", encrypted" : "", g_ready ? ", ready" : "");
  struct ble_gap_conn_desc desc;
  if (connected() && ble_gap_conn_find(g_conn, &desc) == 0) {
    logmsg("link        interval %u x1.25 ms, latency %u, timeout %u0 ms\n",
           desc.conn_itvl, desc.conn_latency, desc.supervision_timeout);
  }
  logmsg("advertising %s%s\n", ble_gap_adv_active() ? "yes" : "no",
         g_dormant ? " (dormant)" : "");
  logmsg("bonds       %s\n", hasBonds() ? "yes" : "none (pairing mode)");
  logmsg("battery     %u%% (%u mV)  usb=%s\n", g_battery, g_battery_mv, usbPowered() ? "yes" : "no");
  logmsg("reset       0x%X\n", (unsigned)esp_reset_reason());
  printLogRing();
}

static void printHelp() {
  logmsg("commands: n=page forward  p=page backward  +=brightness up  -=brightness down\n");
  logmsg("          s=sleep  i=status+log  l=log  c=forget bonds/pair  a=advertise  z=dormant  x=disconnect\n");
  logmsg("          d=reboot  h=help\n");
}

static void handleSerial() {
  if (!Serial.available()) return;
  // Wait briefly so a full USB packet from a terminal, ModemManager probe, or
  // Linux TTY ECHO burst (on a freshly re-enumerated /dev/ttyACM0) is in the
  // RX queue together. Only accept a lone 1-character command (optionally
  // followed by \r and/or \n); discard multi-character bursts.
  delay(2);
  int avail = Serial.available();
  if (avail > 3) {
    while (Serial.available()) {
      Serial.read();
      if (!Serial.available()) delay(1);
    }
    return;
  }
  char c = Serial.read();
  if (c == '\r' || c == '\n') return;
  while (Serial.available()) {
    char next = Serial.read();
    if (next != '\r' && next != '\n') {
      while (Serial.available()) {
        Serial.read();
        if (!Serial.available()) delay(1);
      }
      return;
    }
  }
  switch (c) {
    case 'n': case 'f': case ']': tap(KEY_NEXT); break;
    case 'p': case 'b': case '[': tap(KEY_PREV); break;
    case '+': case '=':           tap(KEY_BRI_UP); break;
    case '-': case '_':           tap(KEY_BRI_DOWN); break;
    case 's':                     sleepAction(); break;
    case 'i':                     printStatus(); break;
    case 'l':                     printLogRing(); break;
    case 'c':                     forgetBonds(); break;
    case 'a':                     g_auto_adv = true; ensureFastAdvertising(); break;
    case 'z':                     enterDormant(); break;
    case 'x':                     dropLink(); break;
    case 'd':
      logmsg("rebooting...\n");
      g_auto_adv = false;
      if (ble_gap_adv_active()) ble_gap_adv_stop();
      dropLink();
      Serial.flush();
      delay(100);
      esp_restart();
      break;
    case 'h': case '?':           printHelp(); break;
    default: break;
  }
}

// ---- Arduino ---------------------------------------------------------------
void setup() {
  // Enable the XIAO ESP32C6 RF switch (FM8625H on GPIO3, active-low) and select
  // the onboard ceramic antenna (GPIO14 = LOW).
  pinMode(WIFI_ENABLE, OUTPUT);
  digitalWrite(WIFI_ENABLE, LOW);
  pinMode(WIFI_ANT_CONFIG, OUTPUT);
  digitalWrite(WIFI_ANT_CONFIG, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  setLed(false);

  for (int i = 0; i < BTN_COUNT; i++) pinMode(BUTTON_PINS[i], INPUT_PULLUP);

  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);

  g_battery = measureBattery();
  setupBle();

  logmsg("Open Kindle Click " FW_VERSION " (ESP32-C6) ready. Send 'h' for commands.\n");
}

void loop() {
  pollButtons();
  servicePending();
  serviceReleases();
  handleSerial();
  serviceBattery();
  serviceConnection();

  static uint32_t last_bond_check = 0;
  if (!g_dormant && millis() - last_bond_check > 1000) {
    last_bond_check = millis();
    g_bonds_cached = hasBonds();
  }
  updateLed();

  if (!g_dormant && !connected() && g_bonds_cached && !usbPowered() &&
      (int32_t)(millis() - g_last_activity) > (int32_t)DORMANT_AFTER_MS) {
    enterDormant();
  }

  delay(g_dormant ? 50 : 5);
}
