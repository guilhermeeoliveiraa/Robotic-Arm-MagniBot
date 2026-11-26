
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "led_strip.h"
#include "led_strip_rmt.h"

#include "vl53l0x.h"

// ============================================================
// LOG
// ============================================================

static const char *TAG = "ROBOT";
static const char *TOF_TAG = "VL53L0X";

// ============================================================
// CONFIGURACOES PWM
// ============================================================

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          50

#define PWM_PERIOD_US           20000
#define PWM_MAX_DUTY            8192

// ============================================================
// LED RGB
// ============================================================

#define GPIO_RGB_LED            GPIO_NUM_48
#define RGB_LED_COUNT           1
#define RGB_LED_BRIGHTNESS      40

static led_strip_handle_t g_rgb_led = NULL;

// ============================================================
// GPIO DOS SERVOS
// ============================================================

#define GPIO_BASE               GPIO_NUM_15
#define GPIO_OMBRO              GPIO_NUM_16
#define GPIO_COTOVELO           GPIO_NUM_17
#define GPIO_PUNHO_ROTACAO      GPIO_NUM_18
#define GPIO_PUNHO_FLEXAO       GPIO_NUM_8
#define GPIO_GARRA              GPIO_NUM_9

// ============================================================
// SENSOR VL53L0X
// ============================================================

#define GPIO_VL53L0X_SDA        GPIO_NUM_11
#define GPIO_VL53L0X_SCL        GPIO_NUM_12

#define I2C_PORT                I2C_NUM_0
#define VL53L0X_ADDRESS         0x29

#define TOF_MAX_AGE_MS          500

// ============================================================
// TELEMETRIA
// ============================================================

#define TELEMETRY_INTERVAL_MS  50

// ============================================================
// VARREDURA
// ============================================================

#define SCAN_DISTANCE_THRESHOLD_MM  150
#define SCAN_CHECK_INTERVAL_MS      20

// ============================================================
// BOTAO
// ============================================================

#define GPIO_BUTTON             GPIO_NUM_7

#define BUTTON_POLL_MS          10
#define BUTTON_DEBOUNCE_MS      30

#define SINGLE_TAP_MAX_MS       1000
#define DOUBLE_TAP_INTERVAL_MS  1000
#define LONG_PRESS_MIN_MS       2000

// ============================================================
// LIMITES DOS SERVOS
// ============================================================

#define OMBRO_MIN               80
#define OMBRO_MAX               150

#define COTOVELO_MIN            0
#define COTOVELO_MAX            79

#define PUNHO_ROTACAO_MIN       15
#define PUNHO_ROTACAO_MAX       112

#define PUNHO_FLEXAO_MIN        18
#define PUNHO_FLEXAO_MAX        144

#define GARRA_MIN               0
#define GARRA_MAX               75

// ============================================================
// BASE
// ============================================================

#define BASE_REFERENCE_ANGLE    90
#define BASE_STOP_PULSE_US      1500

// ============================================================
// MAQUINA DE ESTADOS
// ============================================================

typedef enum {
    IDLE,
    RUN_VARREDURA,
    VARREDURA_CONCLUIDA,
    AGARRAR_OBJETO,
    RUN_DESLOCAMENTO
} robot_state_t;

typedef enum {
    EVENT_SINGLE_TAP,
    EVENT_DOUBLE_TAP,
    EVENT_LONG_PRESS,
    EVENT_SCAN_FINISHED,
    EVENT_OBJECT_FOUND,
    EVENT_OPERATION_FINISHED
} robot_event_t;

static robot_state_t g_robot_state = IDLE;

static QueueHandle_t g_event_queue = NULL;

// Contador de varreduras realizadas.
// 0 = primeira varredura.
// 1 ou mais = segunda varredura em diante.
static uint32_t g_scan_count = 0;

// ============================================================
// ESTRUTURAS DOS SERVOS
// ============================================================

typedef struct {
    int gpio_num;
    ledc_channel_t channel;
    int current_angle;
    int min_angle;
    int max_angle;
} servo_180_t;

typedef struct {
    int gpio_num;
    ledc_channel_t channel;
    int current_pulse_us;
} servo_360_t;

// ============================================================
// SERVOS
// ============================================================

static servo_360_t g_base;

static servo_180_t g_ombro;
static servo_180_t g_cotovelo;
static servo_180_t g_punho_rotacao;
static servo_180_t g_punho_flexao;
static servo_180_t g_garra;

// ============================================================
// SENSOR E DADOS COMPARTILHADOS
// ============================================================

static i2c_master_bus_handle_t g_i2c_bus = NULL;

// API correta do componente pkolt__vl53l0x.
static vl53l0x_handle_t g_tof_sensor = NULL;

static bool g_tof_ready = false;

static int g_distance_mm = -1;
static int64_t g_distance_timestamp_us = 0;

static portMUX_TYPE g_data_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// PROTOTIPOS
// ============================================================

static uint32_t us_to_duty(int pulse_us);

static void ledc_timer_init(void);

static void servo_180_init(
    servo_180_t *servo,
    gpio_num_t gpio,
    ledc_channel_t channel,
    int min_angle,
    int max_angle
);

static void servo_180_move(
    servo_180_t *servo,
    int target_angle,
    int speed_delay_ms
);

static void servo_360_init(
    servo_360_t *servo,
    gpio_num_t gpio,
    ledc_channel_t channel
);

static void servo_360_move(
    servo_360_t *servo,
    int target_us,
    int step_delay_ms
);

static void rgb_led_init(void);
static void rgb_led_set_state(robot_state_t state);

static void i2c_master_init(void);
static esp_err_t vl53l0x_check_i2c(void);
static esp_err_t vl53l0x_init_sensor(void);

static bool robot_object_detected(void);

static bool robot_scan_wait_or_detect(
    uint32_t duration_ms,
    bool detection_enabled
);

static void robot_run_scan(void);
static void robot_idle(void);
static void robot_scan_finished(void);
static void robot_grab_object(void);
static void robot_run_displacement(void);

static void robot_post_event(robot_event_t event);

static void state_machine_handle_event(robot_event_t event);

static void robot_state_task(void *pvParameters);
static void button_task(void *pvParameters);
static void tof_telemetry_task(void *pvParameters);

static void button_init(void);
static void send_angles_to_python(void);

// ============================================================
// CONVERSAO DE MICROSEGUNDOS PARA DUTY
// ============================================================

static uint32_t us_to_duty(int pulse_us)
{
    return ((uint32_t)pulse_us * PWM_MAX_DUTY) /
           PWM_PERIOD_US;
}

// ============================================================
// TIMER PWM
// ============================================================

static void ledc_timer_init(void)
{
    ledc_timer_config_t config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&config));
}

// ============================================================
// LED RGB - INICIALIZACAO
// ============================================================

static void rgb_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_RGB_LED,
        .max_leds = RGB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false
        }
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &g_rgb_led
        )
    );

    ESP_ERROR_CHECK(led_strip_clear(g_rgb_led));
}

// ============================================================
// LED RGB - ESTADOS
// ============================================================

static void rgb_led_set_state(robot_state_t state)
{
    if (g_rgb_led == NULL) {
        return;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    switch (state) {

        case IDLE:
            green = RGB_LED_BRIGHTNESS;
            break;

        case RUN_VARREDURA:
            blue = RGB_LED_BRIGHTNESS;
            break;

        case VARREDURA_CONCLUIDA:
            red = RGB_LED_BRIGHTNESS;
            green = RGB_LED_BRIGHTNESS;
            break;

        case AGARRAR_OBJETO:
            red = RGB_LED_BRIGHTNESS;
            break;

        case RUN_DESLOCAMENTO:
            red = RGB_LED_BRIGHTNESS;
            blue = RGB_LED_BRIGHTNESS;
            break;

        default:
            break;
    }

    ESP_ERROR_CHECK(
        led_strip_set_pixel(
            g_rgb_led,
            0,
            red,
            green,
            blue
        )
    );

    ESP_ERROR_CHECK(led_strip_refresh(g_rgb_led));
}

// ============================================================
// ENVIO DE EVENTOS
// ============================================================

static void robot_post_event(robot_event_t event)
{
    if (g_event_queue == NULL) {
        return;
    }

    xQueueSend(g_event_queue, &event, 0);
}

// ============================================================
// TRANSICOES DA MAQUINA DE ESTADOS
// ============================================================

static void state_machine_handle_event(robot_event_t event)
{
    robot_state_t previous_state = g_robot_state;

    switch (g_robot_state) {

        case IDLE:

            if (event == EVENT_SINGLE_TAP) {
                g_robot_state = RUN_VARREDURA;
            }

            break;

        case RUN_VARREDURA:

            if (event == EVENT_SCAN_FINISHED) {
                g_robot_state = VARREDURA_CONCLUIDA;
            }
            else if (event == EVENT_OBJECT_FOUND) {
                g_robot_state = AGARRAR_OBJETO;
            }

            break;

        case VARREDURA_CONCLUIDA:

            // Toque simples: inicia deslocamento.
            if (event == EVENT_SINGLE_TAP) {
                g_robot_state = AGARRAR_OBJETO;
            }

            // Toque longo: inicia outra varredura.
            else if (event == EVENT_LONG_PRESS) {
                g_robot_state = RUN_VARREDURA;
            }

            break;

        case AGARRAR_OBJETO:

            if (event == EVENT_OPERATION_FINISHED) {
                g_robot_state = RUN_DESLOCAMENTO;
            }

            break;
        
        case RUN_DESLOCAMENTO:

            if (event == EVENT_OPERATION_FINISHED) {
                g_robot_state = IDLE;
            }

            break;

        default:
            g_robot_state = IDLE;
            break;
    }

    if (previous_state != g_robot_state) {

        ESP_LOGI(
            TAG,
            "Estado: %d -> %d",
            previous_state,
            g_robot_state
        );

        rgb_led_set_state(g_robot_state);
    }
}

// ============================================================
// SERVO 180 GRAUS - INICIALIZACAO
// ============================================================

static void servo_180_init(
    servo_180_t *servo,
    gpio_num_t gpio,
    ledc_channel_t channel,
    int min_angle,
    int max_angle
)
{
    servo->gpio_num = gpio;
    servo->channel = channel;

    servo->min_angle = min_angle;
    servo->max_angle = max_angle;

    servo->current_angle = (min_angle + max_angle) / 2;

    int pulse_us =
        500 + (2000 * servo->current_angle) / 180;

    ledc_channel_config_t config = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = us_to_duty(pulse_us),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&config));

    ESP_ERROR_CHECK(
        ledc_update_duty(LEDC_MODE, channel)
    );
}

// ============================================================
// SERVO 180 GRAUS - MOVIMENTACAO
// ============================================================

static void servo_180_move(
    servo_180_t *servo,
    int target_angle,
    int speed_delay_ms
)
{
    if (target_angle < servo->min_angle) {
        target_angle = servo->min_angle;
    }

    if (target_angle > servo->max_angle) {
        target_angle = servo->max_angle;
    }

    int current_angle;

    portENTER_CRITICAL(&g_data_mux);
    current_angle = servo->current_angle;
    portEXIT_CRITICAL(&g_data_mux);

    int step = (current_angle < target_angle) ? 1 : -1;

    while (current_angle != target_angle) {

        int next_angle = current_angle + step;

        int pulse_us =
            500 + (2000 * next_angle) / 180;

        ESP_ERROR_CHECK(
            ledc_set_duty(
                LEDC_MODE,
                servo->channel,
                us_to_duty(pulse_us)
            )
        );

        ESP_ERROR_CHECK(
            ledc_update_duty(
                LEDC_MODE,
                servo->channel
            )
        );

        portENTER_CRITICAL(&g_data_mux);
        servo->current_angle = next_angle;
        portEXIT_CRITICAL(&g_data_mux);

        current_angle = next_angle;

        if (speed_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(speed_delay_ms));
        }
    }
}

// ============================================================
// SERVO CONTINUO - INICIALIZACAO
// ============================================================

static void servo_360_init(
    servo_360_t *servo,
    gpio_num_t gpio,
    ledc_channel_t channel
)
{
    servo->gpio_num = gpio;
    servo->channel = channel;
    servo->current_pulse_us = BASE_STOP_PULSE_US;

    ledc_channel_config_t config = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = us_to_duty(BASE_STOP_PULSE_US),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&config));

    ESP_ERROR_CHECK(
        ledc_update_duty(LEDC_MODE, channel)
    );
}

// ============================================================
// SERVO CONTINUO - MOVIMENTACAO
// ============================================================

static void servo_360_move(
    servo_360_t *servo,
    int target_us,
    int step_delay_ms
)
{
    (void)step_delay_ms;

    if (target_us < 1000) {
        target_us = 1000;
    }

    if (target_us > 2000) {
        target_us = 2000;
    }

    servo->current_pulse_us = target_us;

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_MODE,
            servo->channel,
            us_to_duty(target_us)
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_MODE,
            servo->channel
        )
    );
}

// ============================================================
// ESTADO IDLE
// ============================================================

static void robot_idle(void)
{
    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    servo_180_move(&g_ombro, 127, 90);
    servo_180_move(&g_cotovelo, 40, 90);
    servo_180_move(&g_punho_rotacao, 63, 90);
    servo_180_move(&g_punho_flexao, 81, 90);
    servo_180_move(&g_garra, 50, 90);
}

// ============================================================
// VERIFICACAO DE OBJETO
// ============================================================

static bool robot_object_detected(void)
{
    int distance;
    int64_t sample_time;

    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&g_data_mux);

    distance = g_distance_mm;
    sample_time = g_distance_timestamp_us;

    portEXIT_CRITICAL(&g_data_mux);

    if (distance < 0 || sample_time == 0) {
        return false;
    }

    if (
        (now - sample_time) >
        ((int64_t)TOF_MAX_AGE_MS * 1000)
    ) {
        return false;
    }

    return distance < SCAN_DISTANCE_THRESHOLD_MM;
}

// ============================================================
// ESPERA DA VARREDURA
//
// detection_enabled = false:
// Apenas aguarda o tempo definido.
//
// detection_enabled = true:
// Verifica o sensor durante a espera.
// Se encontrar objeto, para a base e retorna true.
// ============================================================

static bool robot_scan_wait_or_detect(
    uint32_t duration_ms,
    bool detection_enabled
)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms) {

        if (
            detection_enabled &&
            robot_object_detected()
        ) {

            servo_360_move(
                &g_base,
                BASE_STOP_PULSE_US,
                0
            );

            ESP_LOGI(
                TAG,
                "Objeto detectado durante a varredura."
            );

            return true;
        }

        vTaskDelay(
            pdMS_TO_TICKS(SCAN_CHECK_INTERVAL_MS)
        );

        elapsed_ms += SCAN_CHECK_INTERVAL_MS;
    }

    // Verificacao final para nao perder uma deteccao
    // ocorrida exatamente no final da espera.
    if (
        detection_enabled &&
        robot_object_detected()
    ) {

        servo_360_move(
            &g_base,
            BASE_STOP_PULSE_US,
            0
        );

        return true;
    }

    return false;
}

// ============================================================
// ESTADO RUN_VARREDURA
// ============================================================

static void robot_run_scan(void)
{
    bool first_scan = (g_scan_count == 0);

    // A primeira varredura nao interrompe os movimentos.
    // A partir da segunda, a deteccao fica habilitada.
    bool detection_enabled = !first_scan;

    ESP_LOGI(
        TAG,
        "Iniciando varredura numero %lu",
        (unsigned long)(g_scan_count + 1)
    );

    ESP_LOGI(
        TAG,
        "Deteccao durante varredura: %s",
        detection_enabled ? "ATIVADA" : "DESATIVADA"
    );

    // Garante que a base esteja parada.
    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    // Posicionamento inicial do braco.
    servo_180_move(&g_punho_flexao, 140, 90);
    servo_180_move(&g_cotovelo, 50, 90);
    servo_180_move(&g_ombro, 100, 90);

    // --------------------------------------------------------
    // MOVIMENTO PARA A DIREITA
    // --------------------------------------------------------

    ESP_LOGI(TAG, "Varredura: direita");

    servo_360_move(&g_base, 1600, 0);

    if (
        robot_scan_wait_or_detect(
            1000,
            detection_enabled
        )
    ) {
        goto object_found;
    }

    // --------------------------------------------------------
    // PAUSA
    // --------------------------------------------------------

    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    if (
        robot_scan_wait_or_detect(
            300,
            detection_enabled
        )
    ) {
        goto object_found;
    }

    // --------------------------------------------------------
    // MOVIMENTO PARA A ESQUERDA
    // --------------------------------------------------------

    ESP_LOGI(TAG, "Varredura: esquerda");

    servo_360_move(&g_base, 1400, 0);

    if (
        robot_scan_wait_or_detect(
            2000,
            detection_enabled
        )
    ) {
        goto object_found;
    }

    // --------------------------------------------------------
    // PAUSA
    // --------------------------------------------------------

    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    if (
        robot_scan_wait_or_detect(
            300,
            detection_enabled
        )
    ) {
        goto object_found;
    }

    // --------------------------------------------------------
    // RETORNO AO CENTRO
    // --------------------------------------------------------

    ESP_LOGI(TAG, "Varredura: retorno ao centro");

    servo_360_move(&g_base, 1600, 0);

    if (
        robot_scan_wait_or_detect(
            1000,
            detection_enabled
        )
    ) {
        goto object_found;
    }

    // --------------------------------------------------------
    // VARREDURA TERMINOU SEM DETECCAO
    // --------------------------------------------------------

    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    g_scan_count++;

    ESP_LOGI(
        TAG,
        "Varredura %lu concluida sem deteccao.",
        (unsigned long)g_scan_count
    );

    robot_post_event(EVENT_SCAN_FINISHED);

    return;

    // --------------------------------------------------------
    // OBJETO ENCONTRADO
    // --------------------------------------------------------

object_found:

    // Interrompe a base imediatamente.
    // Os outros servos permanecem na posicao atual.
    servo_360_move(
        &g_base,
        BASE_STOP_PULSE_US,
        0
    );

    g_scan_count++;

    ESP_LOGI(
        TAG,
        "Objeto encontrado na varredura %lu.",
        (unsigned long)g_scan_count
    );

    ESP_LOGI(
        TAG,
        "Base parada. Mantendo posicao atual do braco."
    );

    robot_post_event(EVENT_OBJECT_FOUND);
}

// ============================================================
// ESTADO VARREDURA_CONCLUIDA
// ============================================================

static void robot_scan_finished(void)
{
    ESP_LOGI(
        TAG,
        "Varredura concluida. Aguardando comando."
    );

    /*
     * Toque simples:
     * EVENT_SINGLE_TAP -> RUN_DESLOCAMENTO
     *
     * Toque longo:
     * EVENT_LONG_PRESS -> RUN_VARREDURA
     */
}

// ============================================================
// ESTADO AGARRAR_OBJETO
// ============================================================

static void robot_grab_object(void)
{
    ESP_LOGI(
        TAG,
        "Estado AGARRAR_OBJETO."
    );

    ESP_LOGI(
        TAG,
        "Mantendo os servos na posicao em que o objeto foi detectado."
    );

    // Sequencia anterior preservada como placeholder.
    servo_180_move(&g_punho_flexao, 144, 70);
    servo_180_move(&g_cotovelo, 0, 90);
    servo_180_move(&g_ombro, 90, 90);
    servo_180_move(&g_punho_rotacao, 63, 90);
    servo_180_move(&g_punho_flexao, 50, 70);
    servo_180_move(&g_garra, 0, 40);

    // Sinaliza que terminou.
    robot_post_event(EVENT_OPERATION_FINISHED);
}

// ============================================================
// ESTADO RUN_DESLOCAMENTO
// ============================================================

static void robot_run_displacement(void)
{
    ESP_LOGI(
        TAG,
        "Iniciando operacao de deslocamento."
    );

    servo_180_move(&g_punho_flexao, 144, 70);

    
    // Sinaliza que terminou.
    robot_post_event(EVENT_OPERATION_FINISHED);
}

// ============================================================
// TAREFA DA MAQUINA DE ESTADOS
// ============================================================

static void robot_state_task(void *pvParameters)
{
    (void)pvParameters;

    robot_state_t executed_state = (robot_state_t)-1;

    robot_event_t event;

    while (1) {

        // Executa a rotina somente quando o estado muda.
        if (executed_state != g_robot_state) {

            executed_state = g_robot_state;

            switch (g_robot_state) {

                case IDLE:

                    robot_idle();

                    break;

                case RUN_VARREDURA:

                    robot_run_scan();

                    break;

                case VARREDURA_CONCLUIDA:

                    robot_scan_finished();

                    break;

                case AGARRAR_OBJETO:

                    robot_grab_object();

                    break;

                case RUN_DESLOCAMENTO:

                    robot_run_displacement();

                    break;

                default:

                    break;
            }
        }

        if (
            xQueueReceive(
                g_event_queue,
                &event,
                pdMS_TO_TICKS(20)
            ) == pdTRUE
        ) {

            state_machine_handle_event(event);
        }
    }
}

// ============================================================
// TELEMETRIA SERIAL
// ============================================================

static void send_angles_to_python(void)
{
    int angle_ombro;
    int angle_cotovelo;
    int angle_punho_rotacao;
    int angle_punho_flexao;
    int angle_garra;
    int distance;

    portENTER_CRITICAL(&g_data_mux);

    angle_ombro = g_ombro.current_angle;
    angle_cotovelo = g_cotovelo.current_angle;
    angle_punho_rotacao = g_punho_rotacao.current_angle;
    angle_punho_flexao = g_punho_flexao.current_angle;
    angle_garra = g_garra.current_angle;

    distance = g_distance_mm;

    portEXIT_CRITICAL(&g_data_mux);

    printf(
        "%d,%d,%d,%d,%d,%d,%d\n",
        BASE_REFERENCE_ANGLE,
        angle_ombro,
        angle_cotovelo,
        angle_punho_rotacao,
        angle_punho_flexao,
        angle_garra,
        distance
    );

    fflush(stdout);
}

// ============================================================
// I2C - INICIALIZACAO
// ============================================================

static void i2c_master_init(void)
{
    i2c_master_bus_config_t config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = GPIO_VL53L0X_SDA,
        .scl_io_num = GPIO_VL53L0X_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &config,
            &g_i2c_bus
        )
    );

    ESP_LOGI(
        TOF_TAG,
        "I2C inicializado. SDA=%d SCL=%d",
        GPIO_VL53L0X_SDA,
        GPIO_VL53L0X_SCL
    );
}

// ============================================================
// TESTE DE COMUNICACAO I2C
// ============================================================

static esp_err_t vl53l0x_check_i2c(void)
{
    return i2c_master_probe(
        g_i2c_bus,
        VL53L0X_ADDRESS,
        1000
    );
}

// ============================================================
// VL53L0X - INICIALIZACAO
//
// API do componente pkolt__vl53l0x:
//   vl53l0x_create(&sensor, bus)
//   vl53l0x_init(sensor)
//   vl53l0x_set_profile(sensor, profile)
//   vl53l0x_set_mode(sensor, mode)
//   vl53l0x_single_measure(sensor, &data)
// ============================================================

static esp_err_t vl53l0x_init_sensor(void)
{
    esp_err_t ret;

    ret = vl53l0x_check_i2c();

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Sensor nao encontrado no endereco 0x%02X.",
            VL53L0X_ADDRESS
        );

        return ret;
    }

    // Cria o handle associado ao barramento I2C.
    ret = vl53l0x_create(
        &g_tof_sensor,
        g_i2c_bus
    );

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro ao criar sensor: %s",
            esp_err_to_name(ret)
        );

        g_tof_sensor = NULL;

        return ret;
    }

    // Inicializa o sensor.
    ret = vl53l0x_init(g_tof_sensor);

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro na inicializacao: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    // Configura o perfil de medicao.
    ret = vl53l0x_set_profile(
        g_tof_sensor,
        VL53L0X_PROFILE_HIGH_SPEED
    );

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro ao configurar perfil: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    // Configura o modo de medicao unica.
    ret = vl53l0x_set_mode(
        g_tof_sensor,
        VL53L0X_MODE_SINGLE
    );

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro ao configurar modo SINGLE: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    g_tof_ready = true;

    ESP_LOGI(
        TOF_TAG,
        "VL53L0X inicializado com sucesso."
    );

    return ESP_OK;
}

// ============================================================
// TAREFA DO SENSOR E TELEMETRIA
// ============================================================

static void tof_telemetry_task(void *pvParameters)
{
    (void)pvParameters;

    vl53l0x_data_t data = {0};

    while (1) {

        if (
            g_tof_ready &&
            g_tof_sensor != NULL
        ) {

            data = (vl53l0x_data_t){0};

            esp_err_t ret = vl53l0x_single_measure(
                g_tof_sensor,
                &data
            );

            portENTER_CRITICAL(&g_data_mux);

            if (
                ret == ESP_OK &&
                data.valid &&
                data.distance_mm < 8190
            ) {

                g_distance_mm = (int)data.distance_mm;

                g_distance_timestamp_us =
                    esp_timer_get_time();

            } else {

                g_distance_mm = -1;
                g_distance_timestamp_us = 0;
            }

            portEXIT_CRITICAL(&g_data_mux);
        }

        // A telemetria permanece no formato CSV.
        send_angles_to_python();

        vTaskDelay(
            pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS)
        );
    }
}

// ============================================================
// BOTAO - INICIALIZACAO
// ============================================================

static void button_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << GPIO_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&config));
}

// ============================================================
// BOTAO - TAREFA
// ============================================================

static void button_task(void *pvParameters)
{
    (void)pvParameters;

    int raw_level;

    int stable_level = 1;
    int previous_level = 1;

    int64_t press_start = 0;
    int64_t first_release = 0;
    int64_t debounce_start = 0;

    bool press_active = false;
    bool long_press_sent = false;

    bool tap_pending = false;
    bool second_tap = false;

    while (1) {

        raw_level = gpio_get_level(GPIO_BUTTON);

        int64_t now = esp_timer_get_time() / 1000;

        // ----------------------------------------------------
        // DEBOUNCE
        // ----------------------------------------------------

        if (raw_level != stable_level) {

            if (debounce_start == 0) {
                debounce_start = now;
            }

            if (
                (now - debounce_start) >=
                BUTTON_DEBOUNCE_MS
            ) {

                stable_level = raw_level;
                debounce_start = 0;
            }

        } else {

            debounce_start = 0;
        }

        // ----------------------------------------------------
        // INICIO DO PRESSIONAMENTO
        // ----------------------------------------------------

        if (
            previous_level == 1 &&
            stable_level == 0
        ) {

            press_start = now;
            press_active = true;
            long_press_sent = false;

            if (tap_pending) {

                if (
                    (now - first_release) <
                    DOUBLE_TAP_INTERVAL_MS
                ) {

                    second_tap = true;

                } else {

                    robot_post_event(EVENT_SINGLE_TAP);

                    tap_pending = false;
                    second_tap = false;
                }
            }
        }

        // ----------------------------------------------------
        // TOQUE LONGO
        // ----------------------------------------------------

        if (
            stable_level == 0 &&
            press_active &&
            !long_press_sent
        ) {

            if (
                (now - press_start) >=
                LONG_PRESS_MIN_MS
            ) {

                robot_post_event(EVENT_LONG_PRESS);

                long_press_sent = true;
                tap_pending = false;
                second_tap = false;
            }
        }

        // ----------------------------------------------------
        // LIBERACAO DO BOTAO
        // ----------------------------------------------------

        if (
            previous_level == 0 &&
            stable_level == 1 &&
            press_active
        ) {

            int64_t press_duration = now - press_start;

            press_active = false;

            if (
                !long_press_sent &&
                press_duration < SINGLE_TAP_MAX_MS
            ) {

                if (
                    second_tap &&
                    (now - first_release) <
                    DOUBLE_TAP_INTERVAL_MS
                ) {

                    robot_post_event(EVENT_DOUBLE_TAP);

                    tap_pending = false;
                    second_tap = false;

                } else {

                    if (second_tap) {
                        robot_post_event(EVENT_SINGLE_TAP);
                    }

                    tap_pending = true;
                    second_tap = false;

                    first_release = now;
                }

            } else if (!long_press_sent) {

                tap_pending = false;
                second_tap = false;
            }
        }

        // ----------------------------------------------------
        // EXPIRACAO DO TOQUE SIMPLES
        // ----------------------------------------------------

        if (
            tap_pending &&
            !press_active &&
            (now - first_release) >=
            DOUBLE_TAP_INTERVAL_MS
        ) {

            robot_post_event(EVENT_SINGLE_TAP);

            tap_pending = false;
            second_tap = false;
        }

        previous_level = stable_level;

        vTaskDelay(
            pdMS_TO_TICKS(BUTTON_POLL_MS)
        );
    }
}

// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    // Desativa logs para manter a UART exclusivamente com CSV.
    esp_log_level_set("*", ESP_LOG_NONE);

    // --------------------------------------------------------
    // LED RGB
    // --------------------------------------------------------

    rgb_led_init();
    rgb_led_set_state(IDLE);

    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledc_timer_init();

    // --------------------------------------------------------
    // BASE
    // --------------------------------------------------------

    servo_360_init(
        &g_base,
        GPIO_BASE,
        LEDC_CHANNEL_0
    );

    // --------------------------------------------------------
    // OMBRO
    // --------------------------------------------------------

    servo_180_init(
        &g_ombro,
        GPIO_OMBRO,
        LEDC_CHANNEL_1,
        OMBRO_MIN,
        OMBRO_MAX
    );

    // --------------------------------------------------------
    // COTOVELO
    // --------------------------------------------------------

    servo_180_init(
        &g_cotovelo,
        GPIO_COTOVELO,
        LEDC_CHANNEL_2,
        COTOVELO_MIN,
        COTOVELO_MAX
    );

    // --------------------------------------------------------
    // PUNHO ROTACAO
    // --------------------------------------------------------

    servo_180_init(
        &g_punho_rotacao,
        GPIO_PUNHO_ROTACAO,
        LEDC_CHANNEL_3,
        PUNHO_ROTACAO_MIN,
        PUNHO_ROTACAO_MAX
    );

    // --------------------------------------------------------
    // PUNHO FLEXAO
    // --------------------------------------------------------

    servo_180_init(
        &g_punho_flexao,
        GPIO_PUNHO_FLEXAO,
        LEDC_CHANNEL_4,
        PUNHO_FLEXAO_MIN,
        PUNHO_FLEXAO_MAX
    );

    // --------------------------------------------------------
    // GARRA
    // --------------------------------------------------------

    servo_180_init(
        &g_garra,
        GPIO_GARRA,
        LEDC_CHANNEL_5,
        GARRA_MIN,
        GARRA_MAX
    );

    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    i2c_master_init();

    // --------------------------------------------------------
    // VL53L0X
    // --------------------------------------------------------

    esp_err_t tof_ret = vl53l0x_init_sensor();

    if (tof_ret != ESP_OK) {
        g_tof_ready = false;
    }

    // --------------------------------------------------------
    // BOTAO
    // --------------------------------------------------------

    button_init();

    // --------------------------------------------------------
    // FILA DE EVENTOS
    // --------------------------------------------------------

    g_event_queue = xQueueCreate(
        10,
        sizeof(robot_event_t)
    );

    if (g_event_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // --------------------------------------------------------
    // TAREFA DE TELEMETRIA E SENSOR
    // --------------------------------------------------------

    BaseType_t result;

    result = xTaskCreate(
        tof_telemetry_task,
        "tof_telemetry",
        4096,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // --------------------------------------------------------
    // TAREFA DO BOTAO
    // --------------------------------------------------------

    result = xTaskCreate(
        button_task,
        "button_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // --------------------------------------------------------
    // TAREFA DA MAQUINA DE ESTADOS
    // --------------------------------------------------------

    result = xTaskCreate(
        robot_state_task,
        "robot_state_task",
        4096,
        NULL,
        6,
        NULL
    );

    if (result != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // --------------------------------------------------------
    // LOOP PRINCIPAL
    // --------------------------------------------------------

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
