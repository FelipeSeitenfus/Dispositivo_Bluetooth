#include "asf.h"
#include "usart.h"
#include "platform.h"
#include "timer_hw.h"
#include "conf_timer.h"
#include "conf_extint.h"
#include "ble_manager.h"
#include "immediate_alert.h"
#include "find_me_app.h"
#include "find_me_target.h"

/* === MACROS ============================================================== */

void tc_cc0_cb(struct tc_module *const module_inst);
static struct usart_module cdc_uart_module;

static const ble_event_callback_t fmp_gap_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	fmp_target_connected_state_handler,
	fmp_target_disconnect_event_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static const ble_event_callback_t fmp_gatt_server_handle[] = {
	NULL,
	NULL,
	fmp_target_char_changed_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};


uint32_t timeout_count;
hw_timer_callback_t timer_callback;
at_ble_events_t event;
uint8_t ble_event_params[524];

gatt_service_handler_t ias_handle;

/* flag da tarefa do Timer */
volatile bool app_timer_done = false;
/*Flag da contagem de tempo*/
static uint8_t timer_interval = INIT_TIMER_INTERVAL;

/** @brief Interrupção para o serviço de alerta imediato */
find_me_callback_t immediate_alert_cb;

/*Interrupção do tempo*/
static void timer_callback_handler(void)
{
	/* para o timer */
	tc_disable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);

	/* Habilita a flag de execução da tarefa */
	app_timer_done = true;
}

/*tratamento do sinal bluetooth mandado para o dispositivo*/
static void app_immediate_alert(uint8_t alert_val)
{
	if (alert_val == IAS_HIGH_ALERT) {
		DBG_LOG("Find Me : High Alert");
		LED_On(LED0);
		timeout_count = LED_FAST_INTERVAL;
		tc_set_count_value(&tc_instance, 0);
		tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
		
	} else if (alert_val == IAS_MID_ALERT) {
		DBG_LOG("Find Me : Mild Alert");
		LED_On(LED0);
		timer_interval = LED_MILD_INTERVAL;
		timeout_count = LED_MILD_INTERVAL;
		tc_set_count_value(&tc_instance, 0);
		tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
			
	} else if (alert_val == IAS_NO_ALERT) {
		DBG_LOG("Find Me : No Alert");
		tc_disable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
		LED_Off(LED0);
	}
}

/**
 * \brief Find Me - função main
 */
int main(void)
{
	struct tc_config config_tc;
	struct usart_config usart_conf;
	
	//inicializando o sistema
	system_init();
	
	/* inicializando o controle serial */
	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = USART_RX_1_TX_0_XCK_1;
	usart_conf.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	usart_conf.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
	usart_conf.pinmux_pad2 = PINMUX_UNUSED;
	usart_conf.pinmux_pad3 = PINMUX_UNUSED;
	usart_conf.baudrate    = 115200;
	stdio_serial_init(&cdc_uart_module, SERCOM3, &usart_conf);
	usart_enable(&cdc_uart_module);

	/* Inicializando o timer */
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_CTRLA_MODE_COUNT16;
	config_tc.clock_source = GCLK_GENERATOR_0;
	config_tc.clock_prescaler = TC_CTRLA_PRESCALER(7);
	config_tc.counter_8_bit.period = 0;
	config_tc.counter_16_bit.compare_capture_channel[0] = (48000000ul/1024ul);
	config_tc.counter_16_bit.compare_capture_channel[1] = 0xFFFF;
	tc_init(&tc_instance, TC3, &config_tc);
	tc_enable(&tc_instance);
	tc_register_callback(&tc_instance, tc_cc0_cb, TC_CALLBACK_CC_CHANNEL0);
	
	timer_callback = timer_callback_handler;

	DBG_LOG("Initializing Find Me Application");

	/* inicializa o dispositivo bluetooth e seta o endereço*/
	ble_device_init(NULL);

/**
 * \brief Inicialização dos serviços de busca
 */

/** Inicia o serviço de alerta imediato do dispositivo de busca*/
	init_immediate_alert_service(&ias_handle);
	
/**
 * \Definição do serviço de alerta imediato para o dispositivo bluetooth*/
	ias_primary_service_define(&ias_handle);
	DBG_LOG("The Supported Services in Find Me Profile are:");
	DBG_LOG("  -> Immediate Alert Service");
	
	/* Seta os dados para o BLE*/
	if(!(ble_advertisement_data_set() == AT_BLE_SUCCESS))
	{
		DBG_LOG("Fail to set Advertisement data");
	}

	/* Status */
	if (at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,
	AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY,
	APP_FMP_FAST_ADV, APP_FMP_ADV_TIMEOUT,
	0) == AT_BLE_SUCCESS) {
		DBG_LOG("Bluetooth device is in Advertising Mode");
		} else {
		DBG_LOG("BLE Adv start Failed");
	}
	
	/* Registro de interrupção do BLE-GAP */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, fmp_gap_handle);
	
	/* Registro de interrupção do BLE-GATT-Server */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, fmp_gatt_server_handle);

	/*registra o destino da interrupção do dispositivo*/
	immediate_alert_cb = app_immediate_alert;

	/* Execução normal */
	while (1) {
		
		/* execução ociosa (a  espera de interrupção) do dispositivo Bluetooth */
		if (at_ble_event_get(&event, ble_event_params, BLE_EVENT_TIMEOUT) == AT_BLE_SUCCESS)
		{
			ble_event_manager(event, ble_event_params);
		}
		
		/* Caso o limite de tempo seja atingido */
		if (app_timer_done) {
			LED_Toggle(LED0);
			timeout_count = timer_interval;
			tc_set_count_value(&tc_instance, 0);
			tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
			app_timer_done = false;
		}
	}
}