// buzzer.c — H9.4 non-blocking buzzer feedback.
//
// The active buzzer hangs off TCA9554 expander EXIO8.  We drive it by
// posting pattern IDs to a small FreeRTOS queue.  A dedicated
// low-priority task drains the queue and plays each pattern with
// `bsp_tca9554_set` calls separated by `vTaskDelay` — so the HX711
// owner task (which calls into us) never blocks on beep timing.

#include "buzzer.h"
#include "board_config.h"
#include "tca9554.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "buzzer";

#if BSP_BUZZER_ENABLE

typedef enum {
    PAT_NONE = 0,
    PAT_TAP,
    PAT_TARE_OK,
    PAT_CAL_OK,
    PAT_ERROR,
    PAT_BUSY,
} pattern_t;

typedef struct {
    int          on1_ms;     /* 0 = skip */
    int          off1_ms;    /* 0 = skip */
    int          on2_ms;     /* 0 = skip */
    const char  *name;
} seq_t;

static QueueHandle_t s_q       = NULL;
static bool          s_inited  = false;

static void play_seq(const seq_t *s) {
    ESP_LOGI(TAG, "pattern %s", s->name);
    if (s->on1_ms > 0) {
        (void)bsp_tca9554_set(BSP_BUZZER_VIA_EXIO_BIT, true);
        vTaskDelay(pdMS_TO_TICKS(s->on1_ms));
        (void)bsp_tca9554_set(BSP_BUZZER_VIA_EXIO_BIT, false);
    }
    if (s->off1_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(s->off1_ms));
    }
    if (s->on2_ms > 0) {
        (void)bsp_tca9554_set(BSP_BUZZER_VIA_EXIO_BIT, true);
        vTaskDelay(pdMS_TO_TICKS(s->on2_ms));
        (void)bsp_tca9554_set(BSP_BUZZER_VIA_EXIO_BIT, false);
    }
}

static void buzzer_task(void *arg) {
    (void)arg;
    pattern_t pat;
    for (;;) {
        if (xQueueReceive(s_q, &pat, portMAX_DELAY) != pdTRUE) continue;
        switch (pat) {
            case PAT_TAP:      { seq_t s = {  30,  0,   0, "tap"          }; play_seq(&s); } break;
            case PAT_TARE_OK:  { seq_t s = { 120,  0,   0, "tare_success" }; play_seq(&s); } break;
            case PAT_CAL_OK:   { seq_t s = { 100, 80, 100, "cal_success"  }; play_seq(&s); } break;
            case PAT_ERROR:    { seq_t s = { 400,  0,   0, "error"        }; play_seq(&s); } break;
            case PAT_BUSY:     { seq_t s = {  40,  0,   0, "busy"         }; play_seq(&s); } break;
            default:           break;
        }
    }
}

esp_err_t buzzer_init(void) {
    if (s_inited) return ESP_OK;
    /* Ensure buzzer is silent at boot. */
    (void)bsp_tca9554_set(BSP_BUZZER_VIA_EXIO_BIT, false);

    s_q = xQueueCreate(8, sizeof(pattern_t));
    if (s_q == NULL) {
        ESP_LOGE(TAG, "queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    /* Low priority so beep timing never preempts HX711 (3) or LVGL (4). */
    BaseType_t ok = xTaskCreate(buzzer_task, "buzzer", 2048, NULL,
                                /*priority*/ 2, NULL);
    if (ok != pdPASS) {
        vQueueDelete(s_q);
        s_q = NULL;
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    s_inited = true;
    ESP_LOGI(TAG, "init EXIO%d active buzzer (queue depth 8)",
             BSP_BUZZER_VIA_EXIO_BIT);
    return ESP_OK;
}

static void post(pattern_t p) {
    if (!s_inited || s_q == NULL) return;
    /* xQueueSend with zero timeout: if the queue is full (very fast
     * triple-tap), drop silently rather than blocking the caller. */
    (void)xQueueSend(s_q, &p, 0);
}

void buzzer_tap(void)          { post(PAT_TAP); }
void buzzer_success_tare(void) { post(PAT_TARE_OK); }
void buzzer_success_cal(void)  { post(PAT_CAL_OK); }
void buzzer_error(void)        { post(PAT_ERROR); }
void buzzer_busy(void)         { post(PAT_BUSY); }

#else  /* BSP_BUZZER_ENABLE == 0 */

esp_err_t buzzer_init(void) {
    ESP_LOGI(TAG, "init disabled (BSP_BUZZER_ENABLE=0) — no audio feedback");
    return ESP_OK;
}
void buzzer_tap(void)          { /* no-op */ }
void buzzer_success_tare(void) { /* no-op */ }
void buzzer_success_cal(void)  { /* no-op */ }
void buzzer_error(void)        { /* no-op */ }
void buzzer_busy(void)         { /* no-op */ }

#endif  /* BSP_BUZZER_ENABLE */
