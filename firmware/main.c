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
// CONFIGURAÇÕES GLOBAIS DO PWM
// ============================================================

#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY         50

// ============================================================
// LED RGB INTERNO DO ESP32-S3
// ============================================================

// GPIO 48: referência comum em placas ESP32-S3 DevKitC-1.
// Confira o GPIO correspondente à sua placa.

#define GPIO_RGB_LED           GPIO_NUM_48
#define RGB_LED_COUNT          1
#define RGB_LED_BRIGHTNESS     40

static led_strip_handle_t g_rgb_led = NULL;

// ============================================================
// GPIOs DOS SERVOS
// ============================================================

#define GPIO_BASE              GPIO_NUM_15
#define GPIO_OMBRO             GPIO_NUM_16
#define GPIO_COTOVELO          GPIO_NUM_17
#define GPIO_PUNHO_ROTACAO     GPIO_NUM_18
#define GPIO_PUNHO_FLEXAO      GPIO_NUM_8
#define GPIO_GARRA             GPIO_NUM_9

// ============================================================
// GPIOs DO SENSOR VL53L0X
// ============================================================

#define GPIO_VL53L0X_SDA       GPIO_NUM_11
#define GPIO_VL53L0X_SCL       GPIO_NUM_12

#define I2C_PORT               I2C_NUM_0
#define I2C_FREQUENCY          400000
#define VL53L0X_ADDRESS        0x29

#define TELEMETRY_INTERVAL_MS  50
#define TOF_LOG_INTERVAL       20

// Critério para considerar que o objeto foi encontrado.
#define SCAN_DISTANCE_THRESHOLD_MM  150
#define SCAN_CHECK_INTERVAL_MS      20
#define TOF_MAX_AGE_MS              500

// ============================================================
// CONFIGURAÇÃO DO BOTÃO
// ============================================================

#define GPIO_BUTTON             GPIO_NUM_7

#define BUTTON_POLL_MS          10
#define BUTTON_DEBOUNCE_MS      30

#define SINGLE_TAP_MAX_MS       1000
#define DOUBLE_TAP_INTERVAL_MS  1000
#define LONG_PRESS_MIN_MS       2000

// ============================================================
// LIMITES DE ÂNGULO DOS SERVOS
// ============================================================

#define OMBRO_MIN              80
#define OMBRO_MAX              150

#define COTOVELO_MIN           0
#define COTOVELO_MAX           79

#define PUNHO_ROTACAO_MIN      15
#define PUNHO_ROTACAO_MAX      112

#define PUNHO_FLEXAO_MIN       18
#define PUNHO_FLEXAO_MAX       144

#define GARRA_MIN              0
#define GARRA_MAX              75

// ============================================================
// REFERÊNCIA DA BASE
// ============================================================

#define BASE_REFERENCE_ANGLE   90
#define BASE_STOP_PULSE_US     1500

// ============================================================
// MÁQUINA DE ESTADOS
// ============================================================

typedef enum {
    IDLE,
    RUN_VARREDURA,
    VARREDURA_CONCLUIDA,
    RUN_DESLOCAMENTO
} robot_state_t;

typedef enum {
    EVENT_SINGLE_TAP,
    EVENT_DOUBLE_TAP,
    EVENT_LONG_PRESS,
    EVENT_SCAN_FINISHED,
    EVENT_OPERATION_FINISHED
} robot_event_t;

static robot_state_t g_robot_state = IDLE;
static QueueHandle_t g_event_queue = NULL;

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
// INSTÂNCIAS GLOBAIS DOS SERVOS
// ============================================================

static servo_360_t g_base;

static servo_180_t g_ombro;
static servo_180_t g_cotovelo;
static servo_180_t g_punho_rotacao;
static servo_180_t g_punho_flexao;
static servo_180_t g_garra;

// ============================================================
// REFERÊNCIAS GLOBAIS DO SENSOR
// ============================================================

static i2c_master_bus_handle_t g_i2c_bus = NULL;
static vl53l0x_handle_t g_tof_sensor = NULL;

static bool g_tof_ready = false;

static volatile int g_distance_mm = -1;
static int64_t g_distance_timestamp_us = 0;

// ============================================================
// PROTEÇÃO DOS DADOS COMPARTILHADOS
// ============================================================

static portMUX_TYPE g_data_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// PROTÓTIPOS
// ============================================================

static void servo_180_move(
    servo_180_t *servo,
    int target_angle,
    int speed_delay_ms
);

static void servo_360_move(
    servo_360_t *servo,
    int target_pulse_us,
    int speed_delay_ms
);

static void robot_idle(void);
static void robot_run_scan(void);
static bool robot_scan_wait_or_detect(uint32_t duration_ms);
static bool robot_object_detected(void);
static void robot_scan_finished(void);
static void robot_run_displacement(void);

static void robot_state_task(void *pvParameters);
static void button_task(void *pvParameters);
static void tof_telemetry_task(void *pvParameters);

static void state_machine_handle_event(robot_event_t event);
static void robot_post_event(robot_event_t event);

static void ledc_timer_init(void);
static void i2c_master_init(void);

static esp_err_t vl53l0x_check_i2c(void);
static esp_err_t vl53l0x_init_sensor(void);

// LED RGB

static void rgb_led_init(void);
static void rgb_led_set_state(robot_state_t state);

// ============================================================
// INICIALIZAÇÃO DO LED RGB
// ============================================================

static void rgb_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_RGB_LED,
        .max_leds = RGB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
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

    ESP_ERROR_CHECK(
        led_strip_clear(g_rgb_led)
    );

    ESP_LOGI(
        TAG,
        "LED RGB inicializado no GPIO %d",
        GPIO_RGB_LED
    );
}

// ============================================================
// CONTROLE DO LED RGB POR ESTADO
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

            // VERDE: robô em repouso.

            green = RGB_LED_BRIGHTNESS;

            break;

        case RUN_VARREDURA:

            // AZUL: executando varredura.

            blue = RGB_LED_BRIGHTNESS;

            break;

        case VARREDURA_CONCLUIDA:

            // AMARELO: varredura concluída.

            red = RGB_LED_BRIGHTNESS;
            green = RGB_LED_BRIGHTNESS;

            break;

        case RUN_DESLOCAMENTO:

            // VERMELHO: executando deslocamento.

            red = RGB_LED_BRIGHTNESS;

            break;

        default:

            // Estado desconhecido: LED apagado.

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

    ESP_ERROR_CHECK(
        led_strip_refresh(g_rgb_led)
    );

    ESP_LOGI(
        TAG,
        "LED RGB: R=%u G=%u B=%u | Estado=%d",
        red,
        green,
        blue,
        state
    );
}

// ============================================================
// ENVIO DE EVENTOS PARA A MÁQUINA DE ESTADOS
// ============================================================

static void robot_post_event(robot_event_t event)
{
    if (g_event_queue == NULL) {
        return;
    }

    xQueueSend(g_event_queue, &event, 0);
}

// ============================================================
// MÁQUINA DE ESTADOS - TRANSIÇÕES
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

            break;

        case VARREDURA_CONCLUIDA:

            // Toque simples: inicia o deslocamento.
            if (event == EVENT_SINGLE_TAP) {
                g_robot_state = RUN_DESLOCAMENTO;
            }
            // Toque longo: reinicia a varredura desde o início.
            else if (event == EVENT_LONG_PRESS) {
                g_robot_state = RUN_VARREDURA;
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

    // --------------------------------------------------------
    // ATUALIZAÇÃO DO LED QUANDO OCORRE UMA TRANSIÇÃO
    // --------------------------------------------------------

    if (previous_state != g_robot_state) {

        ESP_LOGI(
            TAG,
            "Estado: %d -> %d",
            previous_state,
            g_robot_state
        );

        // Atualiza a cor correspondente ao novo estado.
        rgb_led_set_state(g_robot_state);
    }
}

// ============================================================
// ESTADO 1: IDLE
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
    servo_180_move(&g_garra, 45, 90);
}

// ============================================================
// ESTADO 2: RUN_VARREDURA
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

    // Ignora amostras inválidas ou antigas.
    if (distance < 0 || sample_time == 0) {
        return false;
    }

    if ((now - sample_time) > ((int64_t)TOF_MAX_AGE_MS * 1000)) {
        return false;
    }

    return distance < SCAN_DISTANCE_THRESHOLD_MM;
}

// Aguarda durante um movimento, verificando continuamente o sensor.
// Retorna true assim que um objeto é detectado.
static bool robot_scan_wait_or_detect(uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < duration_ms) {

        if (robot_object_detected()) {
            servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);
            ESP_LOGI(
                TAG,
                "Objeto detectado durante a varredura (distancia abaixo de %d mm)",
                SCAN_DISTANCE_THRESHOLD_MM
            );
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_CHECK_INTERVAL_MS));
        elapsed_ms += SCAN_CHECK_INTERVAL_MS;
    }

    return robot_object_detected();
}

// ============================================================
// ESTADO 2: RUN_VARREDURA
// ============================================================

static void robot_run_scan(void)
{
    ESP_LOGI(TAG, "Iniciando uma passagem de varredura");

    // Garante que a base esteja parada antes de posicionar o braço.
    servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);

    // Posiciona o braço para iniciar a varredura.
    servo_180_move(&g_punho_flexao, 140, 90);
    servo_180_move(&g_cotovelo, 50, 90);
    servo_180_move(&g_ombro, 100, 90);

    // Uma única passagem: direita -> esquerda -> centro.
    // Durante cada movimento, o VL53L0X é monitorado.

    ESP_LOGI(TAG, "Varredura: movimento para a direita");
    servo_360_move(&g_base, 1600, 0);
    if (robot_scan_wait_or_detect(1000)) {
        goto object_found;
    }

    servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);
    if (robot_scan_wait_or_detect(300)) {
        goto object_found;
    }

    ESP_LOGI(TAG, "Varredura: movimento para a esquerda");
    servo_360_move(&g_base, 1400, 0);
    if (robot_scan_wait_or_detect(2000)) {
        goto object_found;
    }

    servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);
    if (robot_scan_wait_or_detect(300)) {
        goto object_found;
    }

    ESP_LOGI(TAG, "Varredura: retorno ao centro");
    servo_360_move(&g_base, 1600, 0);
    if (robot_scan_wait_or_detect(1000)) {
        goto object_found;
    }

    // A passagem terminou sem encontrar o objeto.
    // Não repete os movimentos: permanece parado monitorando o sensor.
    servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);
    ESP_LOGI(
        TAG,
        "Passagem concluida sem deteccao. Base parada; aguardando objeto."
    );

    while (!robot_object_detected()) {
        vTaskDelay(pdMS_TO_TICKS(SCAN_CHECK_INTERVAL_MS));
    }

object_found:
    servo_360_move(&g_base, BASE_STOP_PULSE_US, 0);
    ESP_LOGI(TAG, "Objeto detectado. Varredura concluida.");

    // A máquina de estados passará para VARREDURA_CONCLUIDA.
    robot_post_event(EVENT_SCAN_FINISHED);
}

// ============================================================
// ESTADO 3: VARREDURA_CONCLUIDA
// ============================================================

static void robot_scan_finished(void)
{
    ESP_LOGI(
        TAG,
        "Varredura concluida. Aguardando comando."
    );


    //Distancia maxima que pega 130mm
    //Distancia maxima que pega 160mm

    /*
     * O robô permanece aguardando um comando.
     *
     * Toque simples:
     * EVENT_SINGLE_TAP -> RUN_DESLOCAMENTO
     *
     * Toque longo:
     * EVENT_LONG_PRESS -> RUN_VARREDURA
     */
}

// ============================================================
// ESTADO 4: RUN_DESLOCAMENTO
// ============================================================

static void robot_run_displacement(void)
{
    ESP_LOGI(
        TAG,
        "Iniciando operacao de deslocamento"
    );

    // 1. Posicionar o braço para agarrar o objeto.
	 servo_180_move(&g_punho_flexao, 144, 70);
	 servo_180_move(&g_cotovelo, 0, 90);
	 servo_180_move(&g_ombro, 90, 90);
	 servo_180_move(&g_punho_rotacao, 63, 90);
	 servo_180_move(&g_garra, 0, 70);
	 
     // 2. Fechar a garra.
	 
	 
     // 3. Transportar o objeto.
	 
	 
     // 4. Realizar o depósito.
	 
	 
     // 5. Retornar o braço à referência.

     //Ao terminar:
     //robot_post_event(EVENT_OPERATION_FINISHED);
     
     //Não enviar o evento enquanto a operação real
     //ainda não estiver implementada.
}

// ============================================================
// TAREFA PRINCIPAL DA MÁQUINA DE ESTADOS
// ============================================================

static void robot_state_task(void *pvParameters)
{
    (void)pvParameters;

    robot_state_t executed_state = (robot_state_t)-1;
    robot_event_t event;

    while (1) {

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
// CONVERSÃO DE MICROSEGUNDOS PARA DUTY PWM
// ============================================================

static uint32_t us_to_duty(int pulse_us)
{
    return ((uint32_t)pulse_us * 8192U) / 20000U;
}

// ============================================================
// ENVIO DOS ÂNGULOS E DISTÂNCIA PARA O PYTHON
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

    /*
     * Protocolo serial:
     *
     * BASE,OMBRO,COTOVELO,PUNHO_ROTACAO,
     * PUNHO_FLEXAO,GARRA,DISTANCIA_MM
     *
     * Exemplo:
     * 90,127,40,63,81,45,235
     *
     * -1 significa que ainda não existe medição válida.
     */

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
// CONFIGURAÇÃO DO TIMER PWM
// ============================================================

static void ledc_timer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&ledc_timer)
    );
}

// ============================================================
// INICIALIZAÇÃO DO BARRAMENTO I2C
// ============================================================

static void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = GPIO_VL53L0X_SDA,
        .scl_io_num = GPIO_VL53L0X_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &i2c_config,
            &g_i2c_bus
        )
    );

    ESP_LOGI(
        TOF_TAG,
        "Barramento I2C inicializado: SDA=%d, SCL=%d, frequencia=%d Hz",
        GPIO_VL53L0X_SDA,
        GPIO_VL53L0X_SCL,
        I2C_FREQUENCY
    );
}

// ============================================================
// TESTE DE COMUNICAÇÃO I2C
// ============================================================

static esp_err_t vl53l0x_check_i2c(void)
{
    ESP_LOGI(
        TOF_TAG,
        "Verificando dispositivo no endereco 0x%02X...",
        VL53L0X_ADDRESS
    );

    esp_err_t ret = i2c_master_probe(
        g_i2c_bus,
        VL53L0X_ADDRESS,
        1000
    );

    if (ret == ESP_OK) {

        ESP_LOGI(
            TOF_TAG,
            "Dispositivo respondeu no endereco 0x%02X",
            VL53L0X_ADDRESS
        );

    } else {

        ESP_LOGE(
            TOF_TAG,
            "Dispositivo nao respondeu no endereco 0x%02X: %s",
            VL53L0X_ADDRESS,
            esp_err_to_name(ret)
        );
    }

    return ret;
}

// ============================================================
// INICIALIZAÇÃO DO SENSOR VL53L0X
// ============================================================

static esp_err_t vl53l0x_init_sensor(void)
{
    esp_err_t ret;

    ret = vl53l0x_check_i2c();

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Inicializacao cancelada: sensor nao encontrado no I2C"
        );

        return ret;
    }

    ret = vl53l0x_create(
        &g_tof_sensor,
        g_i2c_bus
    );

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro ao criar instancia do sensor: %s",
            esp_err_to_name(ret)
        );

        g_tof_sensor = NULL;

        return ret;
    }

    ESP_LOGI(
        TOF_TAG,
        "Instancia do VL53L0X criada"
    );

    ret = vl53l0x_init(g_tof_sensor);

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro na inicializacao do VL53L0X: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    ESP_LOGI(
        TOF_TAG,
        "VL53L0X inicializado com sucesso"
    );

    ret = vl53l0x_set_profile(
        g_tof_sensor,
        VL53L0X_PROFILE_HIGH_SPEED
    );

    if (ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Erro ao configurar perfil HIGH_SPEED: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    ESP_LOGI(
        TOF_TAG,
        "Perfil HIGH_SPEED configurado"
    );

    g_tof_ready = true;

    return ESP_OK;
}

// ============================================================
// TAREFA DE LEITURA DO SENSOR E TELEMETRIA
// ============================================================

static void tof_telemetry_task(void *pvParameters)
{
    (void)pvParameters;

    vl53l0x_data_t data = {0};

    uint32_t sample_count = 0;

    ESP_LOGI(
        TOF_TAG,
        "Tarefa de telemetria iniciada"
    );

    while (1) {

        if (g_tof_ready && g_tof_sensor != NULL) {

            data = (vl53l0x_data_t){0};

            esp_err_t ret = vl53l0x_single_measure(
                g_tof_sensor,
                &data
            );

            sample_count++;

            if (ret == ESP_OK) {

                portENTER_CRITICAL(&g_data_mux);

                if (data.valid) {
                    g_distance_mm = (int)data.distance_mm;
                    g_distance_timestamp_us = esp_timer_get_time();
                } else {
                    // Não mantém uma distância antiga após uma amostra inválida.
                    g_distance_mm = -1;
                    g_distance_timestamp_us = 0;
                }

                portEXIT_CRITICAL(&g_data_mux);

                if ((sample_count % TOF_LOG_INTERVAL) == 0) {

                    ESP_LOGI(
                        TOF_TAG,
                        "Distancia=%u mm | Valida=%d | RangeStatus=%u | Sinal=%.3f MCPS | Ambiente=%.3f MCPS",
                        (unsigned int)data.distance_mm,
                        (int)data.valid,
                        (unsigned int)data.range_status,
                        (double)data.signal_rate_mcps,
                        (double)data.ambient_rate_mcps
                    );
                }

            } else {

                portENTER_CRITICAL(&g_data_mux);
                g_distance_mm = -1;
                g_distance_timestamp_us = 0;
                portEXIT_CRITICAL(&g_data_mux);

                if ((sample_count % TOF_LOG_INTERVAL) == 0) {

                    ESP_LOGE(
                        TOF_TAG,
                        "Falha na medicao: %s",
                        esp_err_to_name(ret)
                    );
                }
            }

        } else {

            if ((sample_count % TOF_LOG_INTERVAL) == 0) {

                ESP_LOGW(
                    TOF_TAG,
                    "Sensor indisponivel. Distancia permanece em -1."
                );
            }

            sample_count++;
        }

        send_angles_to_python();

        vTaskDelay(
            pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS)
        );
    }
}

// ============================================================
// SERVO DE 180 GRAUS - INICIALIZAÇÃO
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

    ledc_channel_config_t cfg = {
        .speed_mode = LEDC_MODE,
        .channel = servo->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = servo->gpio_num,
        .duty = us_to_duty(pulse_us),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&cfg)
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_MODE,
            servo->channel
        )
    );
}

// ============================================================
// SERVO DE 180 GRAUS - MOVIMENTAÇÃO
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

    int step =
        (servo->current_angle < target_angle) ? 1 : -1;

    while (servo->current_angle != target_angle) {

        int next_angle = servo->current_angle + step;

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

        if (speed_delay_ms > 0) {

            vTaskDelay(
                pdMS_TO_TICKS(speed_delay_ms)
            );
        }
    }
}

// ============================================================
// SERVO CONTÍNUO DE 360 GRAUS - INICIALIZAÇÃO
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

    ledc_channel_config_t cfg = {
        .speed_mode = LEDC_MODE,
        .channel = servo->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = servo->gpio_num,
        .duty = us_to_duty(BASE_STOP_PULSE_US),
        .hpoint = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&cfg)
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_MODE,
            servo->channel
        )
    );
}

// ============================================================
// SERVO CONTÍNUO DE 360 GRAUS - MOVIMENTAÇÃO
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
// INICIALIZAÇÃO DO BOTÃO
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

    ESP_ERROR_CHECK(
        gpio_config(&config)
    );
}

// ============================================================
// TAREFA DE LEITURA DO BOTÃO
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
        // DETECÇÃO DO INÍCIO DO PRESSIONAMENTO
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
        // DETECÇÃO DO TOQUE LONGO
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
        // DETECÇÃO DA LIBERAÇÃO DO BOTÃO
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
        // EXPIRAÇÃO DO TOQUE SIMPLES
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
// APLICAÇÃO PRINCIPAL
// ============================================================

void app_main(void)
{
    // --------------------------------------------------------
    // INICIALIZAÇÃO DO LED RGB
    // --------------------------------------------------------

    rgb_led_init();

    // Estado inicial: VERDE.
    rgb_led_set_state(IDLE);

    // --------------------------------------------------------
    // INICIALIZAÇÃO DO PWM
    // --------------------------------------------------------

    ledc_timer_init();

    // --------------------------------------------------------
    // BASE: SERVO CONTÍNUO DE 360 GRAUS
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
    // PUNHO: ROTAÇÃO
    // --------------------------------------------------------

    servo_180_init(
        &g_punho_rotacao,
        GPIO_PUNHO_ROTACAO,
        LEDC_CHANNEL_3,
        PUNHO_ROTACAO_MIN,
        PUNHO_ROTACAO_MAX
    );

    // --------------------------------------------------------
    // PUNHO: FLEXÃO
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
    // INICIALIZAÇÃO DO BARRAMENTO I2C
    // --------------------------------------------------------

    i2c_master_init();

    // --------------------------------------------------------
    // INICIALIZAÇÃO DO SENSOR VL53L0X
    // --------------------------------------------------------

    esp_err_t tof_ret = vl53l0x_init_sensor();

    if (tof_ret != ESP_OK) {

        ESP_LOGE(
            TOF_TAG,
            "Sensor nao inicializado. O restante da aplicacao continuara."
        );

        g_tof_ready = false;

    } else {

        ESP_LOGI(
            TOF_TAG,
            "Sensor pronto para realizar medicoes."
        );
    }

    // --------------------------------------------------------
    // INICIALIZAÇÃO DO BOTÃO
    // --------------------------------------------------------

    button_init();

    // --------------------------------------------------------
    // CRIAÇÃO DA FILA DE EVENTOS
    // --------------------------------------------------------

    g_event_queue = xQueueCreate(
        10,
        sizeof(robot_event_t)
    );

    if (g_event_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // --------------------------------------------------------
    // CRIAÇÃO DA TAREFA DE TELEMETRIA
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
    // CRIAÇÃO DA TAREFA DO BOTÃO
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
    // CRIAÇÃO DA TAREFA DA MÁQUINA DE ESTADOS
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

    ESP_LOGI(
        TAG,
        "Aplicacao inicializada"
    );

    // --------------------------------------------------------
    // LOOP PRINCIPAL
    // --------------------------------------------------------

    while (1) {

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}
