# H9 Design Plan — Standby, Wake, Wi-Fi, BLE, OTA

**Status:** DESIGN ONLY. No code changes at the time of writing. Current
stable tag `stable-h8.1-filter-guard-v2` (commit `27653c3`) is the
baseline this plan extends.

---

## 1. Goal in one paragraph

Turn `BARINV_SCALE_CLEAN` from "always-on weighing app" into a product
that (1) goes to standby when idle, (2) wakes instantly on touch, (3)
optionally wakes when the load changes, and lays the foundation for
(4) Wi-Fi connectivity, (5) BLE pairing, and (6) safe over-the-air
firmware updates — **without breaking weighing accuracy, HX711 timing,
the single-owner architecture, or the calibration-persistence
contract.** Sequenced so each step is independently shippable and
revertable.

---

## 2. Current baseline (what we're protecting)

```
HX711 owner task (CPU0) — sole hardware reader, 10 Hz, bit-banged with
critical-section around the 24-bit burst.  Computes (raw, net,
grams_filtered, stable).

LVGL port task (CPU1) — pinned to CPU1 since the flush_ready patch
(commit 56e1618).

UI task (LVGL timers) — reads atomic snapshots, never touches HX711 or
GPIO.

NVS — namespace "barinv_scale", keys cal_ver / cal_valid / cal_factor /
cal_known (v1).

Boot order — NVS → I2C → TCA9554 → backlight off → ST7701 → LVGL →
CST820 → UI → backlight on → HX711 init (loads NVS cal) → HX711 owner
task → boot auto-tare.

Pins: HX711 SCK = 44, DOUT = 43; touch INT = GPIO16 (RTC-capable);
backlight = GPIO6.
```

**Hard invariants (do not break in any later phase):**
- HX711 owner task is the **only** code that touches GPIO 43/44 or
  runs the bit-bang protocol.
- HX711 critical section duration (~60 µs) must not grow.
- NVS cal schema version 1 must continue to load successfully even
  after future writes (schema must be additive, never destructive).
- Display + touch + LVGL glue (current patched `esp_lvgl_port.c`) —
  the screen-touch-fix patches in `components/` are load-bearing and
  must travel with every future build.

---

## 3. ESP32-S3 sleep & wake landscape (technical context)

| Mode | Current draw | RAM | Wake sources | Wake latency |
|---|---|---|---|---|
| **Active** | 30–100 mA (WiFi off) | full | n/a | n/a |
| **Modem sleep** | 12–15 mA | full | n/a (radio idle only) | n/a |
| **Light sleep** | 0.8–2 mA | retained | any GPIO IRQ, UART, timer, touch sensor | ~ms — resumes mid-function |
| **Deep sleep** | ~7 µA | RTC slow only (8 KB) | RTC GPIO EXT0/EXT1, RTC timer, ULP coproc, touch sensor | full reboot — ~4 s with our boot sequence |

**Pin RTC capability on ESP32-S3:** GPIOs 0–21 are RTC-capable.
Therefore:

| Pin | Role | RTC? | Can wake from deep sleep? |
|---|---|---|---|
| 16 | CST820 touch INT (active-LOW pulse) | ✅ | ✅ via EXT0 / EXT1 |
| 43 | HX711 SCK | ❌ | ❌ (not RTC) |
| 44 | HX711 DOUT | ❌ | ❌ (not RTC) |
| 6 | backlight LEDC | ❌ | n/a (output only) |

**Key consequence:** wake from deep sleep on touch is natively
supported; wake from deep sleep on load-change is NOT — GPIO 43/44
aren't RTC pins, so the HX711 DOUT IRQ can't reach the wake circuit.
Load-wake from deep sleep requires either periodic timer wake (poll the
HX711, decide whether to fully wake), or a hardware change (route DOUT
to an RTC pin, or add an analog comparator). Recommendation: use
timer-poll for H9 and treat hardware re-routing as a future board
revision.

**Backlight & display:** the ST7701 has a sleep-in command (0x10) and
the backlight is LEDC PWM on GPIO 6 — both can be turned off cleanly
without tearing down the LVGL/display stack, giving us a "soft
standby" that's much faster to wake than deep sleep.

---

## 4. Phase H9 — Standby / sleep / wake

### 4.1 Recommended architecture: a three-tier power state machine

```
   ┌────────────────────────────────┐
   │  ACTIVE                        │  ~50-80 mA   full UI, 10 Hz weighing
   │  user actively interacting     │
   └──┬────────────────────────┬────┘
      │  idle 30 s             │  user taps screen
      ▼                        ▲
   ┌────────────────────────────────┐
   │  SOFT STANDBY (display off)    │  ~30-40 mA   HX711 still running at
   │                                │              ~1 Hz, backlight 0,
   │                                │              ST7701 sleep-in
   └──┬────────────────────────┬────┘
      │  idle 5 min            │  user taps screen
      ▼                        ▲                  (touch INT GPIO16)
   ┌────────────────────────────────┐
   │  DEEP SLEEP                    │  ~10 µA      everything off,
   │                                │              wake on EXT0(GPIO16)
   └────────────────────────────────┘
```

**Why three tiers, not just "active vs deep sleep":**
- Soft standby gives sub-200 ms wake from any tap — feels instant.
  Display is dark but the rest of the system is alive.
- Deep sleep saves 1000× the power but pays a ~4 s wake cost (full
  reboot, boot auto-tare).
- A kitchen scale typically toggles between soft standby and active
  many times per session; deep sleep is for end-of-day.

### 4.2 Soft-standby design

| Subsystem | Action entering standby | Action leaving standby |
|---|---|---|
| LCD backlight (GPIO 6 LEDC) | duty → 0 over 200 ms fade | duty → 800 over 200 ms fade |
| ST7701 panel | optional: `esp_lcd_panel_disp_on_off(false)` | `esp_lcd_panel_disp_on_off(true)` |
| LVGL task | leave running (cheap) | unchanged |
| HX711 owner task | reduce poll rate from 10 Hz → 1 Hz (still detects load changes for soft auto-wake on load) | back to 10 Hz |
| Touch | LVGL already gates by INT — taps still trigger UI events that the new power_mgr listens for | unchanged |

The HX711 chip itself can be **power-cycled** by holding SCK high
> 60 µs (datasheet) — this drops the chip to ~1 µA. Worth doing in
deep sleep, optional in soft standby.

### 4.3 Deep sleep design

Enter:
1. Save any session state to NVS (none in H9 baseline).
2. `esp_lcd_panel_disp_on_off(false)`; LEDC duty 0; ST7701 sleep-in.
3. **Park HX711:** hold SCK high (chip goes to power-down — saves
   ~1.5 mA continuous).
4. `esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 0)` — wake on touch INT
   falling edge.
5. *(optional)* `esp_sleep_enable_timer_wakeup(30 * 1000 * 1000ULL)` —
   periodic 30 s "background poll" wake to check load.
6. `esp_deep_sleep_start()`.

Wake (full reboot):
- Boot path runs normally — NVS calibration loads, boot auto-tare runs
  as today.
- An early check of `esp_sleep_get_wakeup_cause()` lets `main.c`
  distinguish:
  - `ESP_SLEEP_WAKEUP_EXT0` → user touched → straight to ACTIVE
  - `ESP_SLEEP_WAKEUP_TIMER` → load-check wake → take one HX711
    sample, if |Δ vs sleep-time baseline| > threshold, fully wake;
    else sleep again
  - `ESP_SLEEP_WAKEUP_UNDEFINED` → cold boot → normal flow

### 4.4 Load-wake decision

Recommend **timer-poll load wake from deep sleep** rather than
light-sleep-with-IRQ:
- Costs: at 30 s poll interval, ~50 ms awake per cycle = 0.17 % duty
  = ~150 µA average. Light sleep would be ~1 mA average — 7× worse.
- Behavior: if you put a pot on a sleeping scale, it wakes within
  30 s. Acceptable for most uses.

If load-wake-within-1 s is a hard requirement, we'd need a hardware
revision (route DOUT to GPIO 0–21).

### 4.5 New files (skeleton, no code yet)

```
main/power_mgr.h   — public API: enum, request transitions, query state
main/power_mgr.c   — state machine, sleep_in/wake hooks, activity timer
                     reads from ui activity events + hx711 cache
                     (read-only); calls into hx711 module for "park
                     chip" via a NEW hx711 API:
                     hx711_park() / hx711_resume()
```

### 4.6 New `hx711.h` surface (additive, no existing API breaks)

```c
// H9: power management hooks.  Called only by power_mgr on transitions.
esp_err_t hx711_park(void);     // stop owner task wake-ups, hold SCK high
esp_err_t hx711_resume(void);   // resume 10 Hz reads, reset filter
esp_err_t hx711_set_rate(uint32_t period_ms);  // 100→1000 for soft standby
```

`hx711_park` does NOT terminate the owner task — it sets a flag that
the task checks and switches its `vTaskDelay` to the long period plus
holds SCK high. This preserves the single-owner contract.

### 4.7 UI changes (minimal)

`ui.c` exports an "activity ping" function that gets called from any
tap event handler. `power_mgr` listens for the ping and resets the
idle timer.

```c
void ui_register_activity(void);   // call from tap callbacks
```

The existing TARE / CAL handlers call this before doing their work.
The poll callback does NOT call it (would defeat idle detection).

### 4.8 H9 constants (proposed, in `board_config.h`)

```c
#define BSP_IDLE_TO_STANDBY_MS        30000    // 30 s
#define BSP_IDLE_TO_DEEP_SLEEP_MS    300000    // 5 min
#define BSP_STANDBY_BACKLIGHT_FADE_MS   200
#define BSP_STANDBY_HX711_PERIOD_MS    1000    // 1 Hz in standby
#define BSP_DEEP_SLEEP_LOAD_CHECK_S      30    // timer-wake interval
#define BSP_DEEP_SLEEP_LOAD_DELTA_G      20    // wake if |Δ| ≥ this
```

### 4.9 Risks (H9-specific)

| Risk | Mitigation |
|---|---|
| Deep-sleep wake re-runs boot auto-tare with weight on the platform → that weight becomes new zero | On `ESP_SLEEP_WAKEUP_EXT0` (touch wake), boot auto-tare can be **skipped** — we already had a tare offset before sleep; restore it from RTC slow memory (8 KB, persists across deep sleep). |
| Soft standby fade looks weird if LVGL stops updating | Keep LVGL running; only stop pushing samples to the label and fade the backlight. |
| Touch INT (GPIO16) not actually pulling low fast enough to wake | Already proven by the H8 INT-gate fix — chip drives INT low on touch reliably. |
| Holding SCK high in standby breaks the next read | `hx711_resume` waits 1 ms after releasing SCK before next read — well within HX711 wake spec. |
| Power_mgr races HX711 owner task | All state transitions go through atomic flags + the existing single-owner contract — no new mutex needed. |
| Idle timer is shared between UI and HX711 → can fire mid-CAL | Power_mgr checks busy guards (`s_tare_in_progress`, `s_cal_in_progress`, `s_boot_autotare_in_progress`) before entering standby. |

### 4.10 H9 test plan

1. Bench multimeter: confirm ACTIVE ~50-80 mA, SOFT STANDBY ~30 mA,
   DEEP SLEEP < 20 µA.
2. UX: tap screen in soft standby → display fully visible in < 300 ms
   with last weight value.
3. UX: tap screen in deep sleep → full boot in < 5 s, calibration
   loads, auto-tare may run.
4. Stress: 30 sleep/wake cycles in a row, no crash, no NVS corruption.
5. Regression: TARE/CAL/STABLE all still work after a wake.
6. Load-wake (if implemented): place 200 g on sleeping scale; scale
   wakes within 30 s, displays ≈ 200 g STABLE.

---

## 5. Phase H10 — Wi-Fi + BLE bring-up (connectivity only)

### Goal of this phase: get radio working, **don't use it for OTA yet**.

This phase splits cleanly into H10a (Wi-Fi) and H10b (BLE). They can
ship independently. Either can be dropped if you decide it's not
needed.

### 5.1 H10a — Wi-Fi

**Design choice: opt-in connect, not always-on.**

```
WIFI_OFF (default)        ←  radio fully off, no current cost
   │
   │ user enables Wi-Fi from UI (Settings screen — new H10b widget)
   ▼
WIFI_CONNECTING           ←  STA mode, attempt saved AP for 20 s
   │           │
   │ success   │ failure → enter SoftAP provisioning mode
   ▼
WIFI_CONNECTED            ←  STA, modem-sleep (DTIM=10), ~1-3 mA avg
```

Credentials live in NVS namespace `barinv_wifi` (separate from cal).
Reset via long-press on a future Settings UI element.

**Power impact:**
- WiFi OFF: 0 mA extra
- WiFi associated + modem sleep: 1-3 mA avg + bursts on Rx
- WiFi associated, active: 12-20 mA continuous

**HX711 timing risk:** WiFi RX/TX bursts can preempt CPU0 momentarily.
Our HX711 task is on CPU0 (LVGL on CPU1). Mitigations:
1. Move HX711 task to CPU1 if WiFi-induced jitter shows up (test
   first — likely not needed).
2. WiFi interrupt priority is already higher than user tasks; the
   critical-section around the 60 µs bit-bang burst is safe because
   portENTER_CRITICAL disables CPU0 interrupts entirely.
3. Worst-case: a delayed sample is just a timeout (handled by existing
   `ESP_ERR_TIMEOUT` path).

**sdkconfig changes required:**
```
CONFIG_ESP32_WIFI_ENABLED=y           (was n)
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y (already y in our defaults)
```

**Plus partition change?** No — WiFi credential blob fits in our
existing 16 KB NVS partition. No partition layout change for H10a.

### 5.2 H10b — BLE

**Design choice: BLE peripheral mode only.**

Advertises "BARINV Scale" with a custom service exposing:
- Weight characteristic (read + notify) — current filtered grams
- Stable characteristic (read + notify) — bool
- TARE characteristic (write) — triggers `hx711_request_tare()`
- CAL characteristic (write) — known weight in grams as 4-byte float →
  triggers `hx711_request_calibrate()`

**Use case justification:** lets BARINV phone app receive live weight
without WiFi setup. Useful for setup workflows (commissioning at a
customer site without their WiFi creds).

**Power:** BLE advertising at 1 Hz = ~5 mA avg. Lower with longer adv
interval.

**sdkconfig changes:**
```
CONFIG_BT_ENABLED=y                   (was n)
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y         (or NimBLE if smaller footprint)
```

NimBLE is smaller (~25 KB) than Bluedroid (~75 KB). Prefer NimBLE for
our footprint budget.

### 5.3 New files

```
main/wifi_mgr.h/c       — state machine, NVS cred store, public connect/disconnect
main/ble_mgr.h/c        — GATT server, characteristics, request bridging to hx711
main/ui_settings.h/c    — small Settings screen accessible via a future gear icon
```

### 5.4 Risks (H10-specific)

| Risk | Mitigation |
|---|---|
| WiFi blocks boot if AP not present | Async connect with 20 s timeout, never block app_main. |
| BLE conflicts with WiFi on the radio | ESP-IDF schedules them via the coexistence layer. Use NimBLE with low-duty advertising; expect ~5 % WiFi throughput loss when both active. |
| sdkconfig bump may invalidate NVS cal | Verify NVS partition offset/size remains unchanged. If we move it for OTA in H11, run a migration on first boot. |
| Footprint grows past current 4 MB OTA partition projection | Run `idf.py size` at end of H10 — should land ≤ 1.5 MB. |
| WiFi RX preempts HX711 bit-bang | Watch for serial `read error: ESP_ERR_TIMEOUT` after H10 enables WiFi. If observed, move HX711 task to CPU1 or raise its priority above WiFi (priority 23). |
| Insecure BLE pairing | Use BLE Just-Works initially, add LE Secure Connections in H11 if compromising. |

### 5.5 H10 test plan

- WiFi off → no behavior change vs H9.
- WiFi connect/disconnect → no impact on STABLE flag, no impact on
  weighing accuracy (compare 500 g reading WiFi-off vs WiFi-on, should
  match within 1 g).
- BLE advertising → phone app sees scale, can read weight, can issue
  TARE/CAL.
- Combined WiFi + BLE → 30-minute soak, no weighing glitches.
- Power: confirm WIFI_OFF stays at H9's standby/sleep numbers.

---

## 6. Phase H11 — OTA

### 6.1 Critical: partition table change

Current:
```
nvs       0x9000   16 KB
phy_init  0xd000   4 KB
factory   0x10000  15 MB
```

Proposed OTA layout (16 MB flash):
```
nvs            0x9000     20 KB
otadata        0xe000      8 KB
phy_init       0x10000     4 KB
ota_0          0x20000   ~6.5 MB
ota_1          0x6c0000  ~6.5 MB
nvs_cal_v2     0xd60000   1 MB    (future cal backup / extended schema)
spiffs         0xe60000   1.5 MB  (web assets if needed)
```

**This change is irreversible without flash-erase.** Need a migration
plan: ship a **migration build** at the boundary that reads NVS v1 from
the old offset and writes it back to the new offset on first run. Or:
accept that users must re-calibrate after upgrading to H11. The latter
is simpler — just one tap of CAL.

**Recommended migration: "soft re-cal." Show a one-time banner: "After
this update, please re-calibrate the scale with 500 g."** Avoid clever
NVS-migration code that might brick devices.

### 6.2 OTA transport choices (pick one for H11, others can be added later)

| Transport | Pros | Cons | Recommended? |
|---|---|---|---|
| **HTTPS pull** from a known server | Standard, secure, simple | Requires hosting infra | ✅ first |
| Local web upload (ESP hosts captive portal) | No server needed | Slower, harder UX | second |
| BLE OTA | Works without WiFi | Very slow (~10 min for 1 MB), needs BARINV app | nice-to-have |
| Phone-relay via BARINV app | Best UX | Highest implementation cost | future |

**H11 ships HTTPS pull only.** Other transports are H12+.

### 6.3 Security baseline for H11

| Feature | Phase |
|---|---|
| HTTPS with **CA bundle** validation | H11 must-have |
| Cert pinning on the OTA endpoint | H11 must-have |
| Signed app images (`CONFIG_SECURE_SIGNED_APPS_ECDSA`) | H11 must-have |
| Secure boot v2 (`CONFIG_SECURE_BOOT`) | H12+ — fuse-burning is irreversible, defer |
| Flash encryption | H12+ — also irreversible fuse-burning |
| Rollback on boot failure (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`) | H11 must-have |

### 6.4 Rollback strategy (built into ESP-IDF)

ESP-IDF supports `esp_ota_mark_app_valid_cancel_rollback()`. The app
must call this within N boots of a new OTA, or the bootloader
auto-rolls back to the previous slot.

```c
// Suggested call site after H11 main reaches "[final step] ready":
esp_ota_mark_app_valid_cancel_rollback();
```

We also keep a stale-but-known-good image in the other OTA slot for
emergency.

### 6.5 New files

```
main/ota_mgr.h/c        — HTTPS pull, progress callbacks, rollback marking
main/ui_ota.h/c         — minimal "Updating…" overlay + progress bar
                          (only widget added in H11)
```

### 6.6 Risks (H11-specific)

| Risk | Mitigation |
|---|---|
| Repartition wipes NVS cal | Document "re-cal after update"; ship a known-good factor in code as a sensible default to avoid bricked UX. |
| OTA download interrupted mid-flash | Bootloader rollback (ESP-IDF native). |
| Bad image bricks device | Rollback after 3 failed boots (configurable). Worst case: USB-C reflash. |
| OTA download saturates WiFi → HX711 timeouts | Pause HX711 owner task or downgrade to 1 Hz during OTA (similar to soft-standby). |
| Anyone can push firmware → security incident | HTTPS + cert pin + signed images. |
| Partition fragmentation, sector wear | OTA partitions only wear when an update lands; expected lifetime 10 000+ erases per sector. |

### 6.7 H11 test plan

- Cold flash with new partition table; cal lost; re-cal works.
- HTTPS pull from known URL succeeds; new app boots; rollback flag
  cleared.
- Force-fail OTA mid-download → device recovers to previous slot.
- Force-fail by uploading a bricked image → device rolls back after N
  boots.
- 24-hour soak with periodic OTA checks.
- Cert pin: try connecting to OTA endpoint with self-signed cert →
  fails cleanly.

---

## 7. Files affected / preserved per phase

### 7.1 Files that may change (allowed)

| File | H9 | H10 | H11 |
|---|---|---|---|
| `main/main.c` | + power_mgr init + wake-cause branch | + wifi/ble init (opt-in) | + ota_mgr init |
| `main/board_config.h` | + sleep/standby constants | + wifi/ble constants | + ota constants |
| `main/hx711.h` | + park/resume/set_rate prototypes | — | — |
| `main/hx711.c` | + park/resume/set_rate impl | — | + pause hook during OTA |
| `main/ui.c` | + ui_register_activity export | + Settings entry icon | + OTA progress overlay hook |
| `main/CMakeLists.txt` | + power_mgr.c | + wifi_mgr.c, ble_mgr.c, ui_settings.c | + ota_mgr.c, ui_ota.c |
| `sdkconfig.defaults` | + power-mgmt options | + WiFi + BT enable | + OTA + signing config |
| `partitions.csv` | — | — | **rewritten** |

### 7.2 Files that must NOT change in H9–H11

- `main/hx711.c` protocol code: `hx711_read_raw_blocking`,
  `filter_step`, `filter_reset`, `ema_and_push`, the TARE/CAL/auto-tare
  handlers
- `main/cst820_touch.c`, `main/st7701_panel.c`, `main/i2c_bus.c`,
  `main/tca9554.c`, `main/backlight.c` — driver layer is stable
- `main/lvgl_glue.c` — touch INT gate stays as-is
- `components/espressif__esp_lvgl_port/esp_lvgl_port.c` — the three
  BARINV patches (flush_ready, delay clamp, touch INT gate) stay
- Pin map in `board_config.h` (LCD lanes, I2C, TCA9554, CST820, HX711)
  — fixed by hardware

### 7.3 NVS schema (must remain backward-readable)

- `barinv_scale` namespace cal v1 keys must continue to load
  successfully in H9, H10, H11.
- Any new keys are added with new names; never re-use existing names
  with new types.
- If we ever need a schema migration, write a v2 keyset and leave v1
  readable for one major version.

---

## 8. Cross-phase risk matrix (top 5)

| Rank | Risk | Phase | Severity | Mitigation |
|---|---|---|---|---|
| 1 | Partition change in H11 wipes NVS calibration | H11 | high (UX) | Document "re-cal after update"; ship sane default factor; one-time on-screen banner |
| 2 | OTA bricks device on bad image | H11 | high (support) | ESP-IDF native rollback + signed images + cert pin |
| 3 | WiFi RX bursts jitter HX711 timing | H10 | medium (accuracy) | Test thoroughly; move HX711 to CPU1 if observed; existing critical-section is already interrupt-safe |
| 4 | Deep-sleep wake re-tares with weight on platform | H9 | medium (accuracy) | Save `tare_offset` to RTC slow memory before sleep; restore on EXT0 wake; skip auto-tare |
| 5 | sdkconfig changes invalidate NVS cal partition layout | H10 | medium (UX) | Verify NVS offset/size hasn't moved in `idf.py partition-table`; keep nvs at 0x9000 |

---

## 9. Rollback plan (per phase)

Each phase ships as:
1. **Restore point** under `_restore_points/` before any edits (we
   already follow this pattern).
2. **Commits** on master, individually revertable via `git revert`.
3. **Tag** at the end of each phase: `stable-h9-sleep-v1`,
   `stable-h10-radio-v1`, `stable-h11-ota-v1`.
4. **NVS namespace separation** — each new feature uses its own
   namespace. Rollback never touches `barinv_scale` cal data.

Emergency rollback procedure:
```
git checkout stable-h8.1-filter-guard-v2
idf.py erase-flash
idf.py -p /dev/cu.usbmodemXXXX flash monitor
# scale recovers to known-good state; user re-cals with 500 g
```

For H11 specifically, after partition table change the rollback also
wipes NVS. That's why H11 is sequenced last and ships only when the
team is ready to handle support calls about "re-calibrate please."

---

## 10. Final recommendation

**Ship H9 next. Defer H10 + H11 until H9 is on real customer hardware
and proven.**

Why:
- H9 has the **highest UX value-per-line-of-code** (commercial kitchen
  scales auto-sleep — this is a "must" for product-feel).
- H9 has **zero partition or NVS schema changes** — fully reversible
  at any point.
- H9 doesn't add radio cost, so it stays inside the current 16 MB
  factory partition with room to spare.
- H10's value depends on whether BARINV's product roadmap actually
  needs network/BLE for this device; if not, skip.
- H11 is the highest-risk single change (partition rewrite + signed
  images + rollback wiring). It earns its place only when there's a
  real over-the-air update need.

If the product roadmap requires phone-app pairing without a customer's
WiFi credentials → H10b (BLE only) becomes the next must-have, ahead
of H10a.

**Recommended next concrete step (when leaving design mode):**

1. Confirm or reject this staging.
2. If staging accepted → start H9 with: `power_mgr.h/c` scaffolding +
   idle timer + soft-standby transition. No deep sleep yet. Tag
   `stable-h9.0-softstandby-v1`. Then add deep sleep as H9.1 and
   `stable-h9.1-deepsleep-v1`.
3. Each H9 sub-step independently flashable and revertable, following
   the same restore-point + tag pattern used since H1.
