#include "asf.h"
#include "usart.h"
#include "platform.h"
#include "timer_hw.h"
#include "tc_interrupt.h"
#include "conf_timer.h"
#include "conf_extint.h"
#include "ble_manager.h"
#include "immediate_alert.h"
#include "find_me_app.h"
#include "find_me_target.h"
#include "pt.h"

/* === MACROS ============================================================== */

void tc_cc0_cb(struct tc_module *const module_inst);
void configure_eeprom(void);

static struct usart_module cdc_uart_module;
struct tc_config config_tc;
struct usart_config usart_conf;

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
/** variáveis de acesso a memória*/
uint8_t last_alert = 0;
uint8_t page_data[EEPROM_PAGE_SIZE];

volatile char i = 0;
volatile char buffer;

gatt_service_handler_t ias_handle;

/** Flag da tarefa do Timer */
volatile bool app_timer_done = false;
/** Flag da contagem de tempo*/
static uint8_t timer_interval = INIT_TIMER_INTERVAL;

/** @brief Interrupção para o serviço de alerta imediato */
find_me_callback_t immediate_alert_cb;

/** Configuração da memória EEPROM.
* EEPROM é uma emulação de uma memória que depende da configuração dos fusos. O tamanho dela na placa deve ser menor que cinco.
* A configuração dos fusos da memória não foi feita da maneira correta. Então, ele fica em um loop infinito.
* Se a EEPROM estiver mal configurada ou corrompida, ela é resetada e seus dados são apagados.
**/

void configure_eeprom(void)
{
//! Inicialização da EEPROM.
	enum status_code error_code = eeprom_emulator_init();

//! Verifica se a configuração foi feita da maneira correta.
	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
			
		}
	}
	else if (error_code != STATUS_OK) {
		printf("Memory error!!!\n");
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
}


/** Configuração da interrupção da memória EEPROM */
#if (SAMD || SAMR21)
void SYSCTRL_Handler(void)
{
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
		eeprom_emulator_commit_page_buffer();
	}
}
#endif

static void configure_bod(void)
{
	#if (SAMD || SAMR21)
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	config_bod33.level = 48;
	bod_set_config(BOD_BOD33, &config_bod33);
	bod_enable(BOD_BOD33);

	SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
	#endif
}

/** Tratamento da interrupção do timer.
* O timer é desabilitado.
* Se app_timer_done é true, é uma indicação de que nenhum dispositivo foi conectado no tempo possível, ou não conseguiu conectar.
* Então, ele reinicia no modo Advertising Mode.
**/
static void timer_callback_handler(void)
{
	tc_disable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
	app_timer_done = true;
}

/** Tratamento do sinal bluetooth mandado pelo celular para o dispositivo.
* Ele indica se o sinal mandado foi alto (High), médio (Mild) ou fraco (No).
* Se o sinal for High ou Mild, o LED acende. Se o sinal for No, apaga.
**/
static void app_immediate_alert(uint8_t alert_val)
{
	if (alert_val == IAS_HIGH_ALERT) {
		DBG_LOG("Find Me : High Alert");
		LED_On(LED0);
		last_alert = 2;
		timeout_count = LED_FAST_INTERVAL;
		tc_set_count_value(&tc_instance, 0);
		tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
		
	} else if (alert_val == IAS_MID_ALERT) {
		DBG_LOG("Find Me : Mild Alert");
		LED_On(LED0);
		last_alert = 1;
		timeout_count = LED_MILD_INTERVAL;
		tc_set_count_value(&tc_instance, 0);
		tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
			
	} else if (alert_val == IAS_NO_ALERT) {
		DBG_LOG("Find Me : No Alert");
		last_alert = 0;
		tc_disable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
		LED_Off(LED0);
	}
	/** Gravação do último sinal dado durante a execução do aplicativo na memória. */
	page_data[0] = last_alert;
	eeprom_emulator_write_page(0, page_data);
	eeprom_emulator_commit_page_buffer();
}
/** Protothread
* A protothread pt_find_me é responsável por configurar e executar a aplicação.
**/
PT_THREAD(pt_find_me(struct pt *pt, char data)){
	PT_BEGIN(pt);
	
	/** Inicialização do sistema */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 0);
	system_init();
	PT_YIELD(pt);
	
	/** Inicialização do controle serial */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 1);
	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = USART_RX_1_TX_0_XCK_1;
	usart_conf.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	usart_conf.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
	usart_conf.pinmux_pad2 = PINMUX_UNUSED;
	usart_conf.pinmux_pad3 = PINMUX_UNUSED;
	usart_conf.baudrate    = 115200;
	stdio_serial_init(&cdc_uart_module, SERCOM3, &usart_conf);
	usart_enable(&cdc_uart_module);
	PT_YIELD(pt);
	
	/**
	 Inicialização do timer.
	 Um counter size de 32 bits permite um tempo maior de espera, dado que o sinal Bluetooth é instável.
	 */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 2);
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_CTRLA_MODE_COUNT32;
	config_tc.clock_source = GCLK_GENERATOR_0;
	config_tc.clock_prescaler = TC_CTRLA_PRESCALER(7);
	config_tc.counter_8_bit.period = 0;
	config_tc.counter_32_bit.compare_capture_channel[0] = (48000000ul/1024ul);
	config_tc.counter_32_bit.compare_capture_channel[1] = 0xFFFF;
	tc_init(&tc_instance, TC3, &config_tc);
	tc_enable(&tc_instance);
	tc_register_callback(&tc_instance, tc_cc0_cb, TC_CALLBACK_CC_CHANNEL0);
	
	/** Registro da interrupção do timer */
	timer_callback = timer_callback_handler;
	PT_YIELD(pt);
	
	/** Configuração da memória EEPROM da placa */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 3);
	configure_eeprom();
	configure_bod();
	PT_YIELD(pt);
	
	/** Inicialização da aplicação */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 4);
	DBG_LOG("Initializing Find Me Application");
	/** Inicialização do dispositivo bluetooth */
	ble_device_init(NULL);
	PT_YIELD(pt);
	
	/** Load da memória do último alerta dado por um celular na execução anterior*/
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 5);
	eeprom_emulator_read_page(0, page_data);
	last_alert = page_data[0];
	if(last_alert == 2){
		DBG_LOG("Last Alert: High Alert!");
	}
	else if(last_alert == 1){
		DBG_LOG("Last Alert: Mild Alert!");
	}
	else{
		DBG_LOG("Last Alert: No Alert!");
	}
	PT_YIELD(pt);

	/** Inicialização do serviço de Find Me */
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 6);
	init_immediate_alert_service(&ias_handle);
	/** Definição da aplicação para o dispositivo bluetooth.
	Indica que ele será usado para o serviço de Find Me.
	*/
	ias_primary_service_define(&ias_handle);
	DBG_LOG("The Supported Services in Find Me Profile are:");
	DBG_LOG("  -> Immediate Alert Service");
	PT_YIELD(pt);
	
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 7);
	if(!(ble_advertisement_data_set() == AT_BLE_SUCCESS))
	{
		DBG_LOG("Fail to set Advertisement data");
	}
	
	/** Status do dispositivo.
	Advertising Mode é o modo de espera do dispositivo após a inicialização.
	Até que um celular seja conectado com sucesso, ele fica nesse modo.
	*/
	if (at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,
	AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY,
	APP_FMP_FAST_ADV, APP_FMP_ADV_TIMEOUT,
	0) == AT_BLE_SUCCESS) {
		DBG_LOG("Bluetooth device is in Advertising Mode");
		} else {
		DBG_LOG("BLE Adv start Failed");
	}
	PT_YIELD(pt);
	
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 8);
	/** Registro de interrupção do BLE-GAP */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, fmp_gap_handle);
	/** Registro de interrupção do BLE-GATT-Server */
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, fmp_gatt_server_handle);
	/** Registro do destino da interrupção do dispositivo.
	Sempre que uma interrupção acontecer, é para a função app_immediate_alert que ele deve ir.
	*/
	immediate_alert_cb = app_immediate_alert;
	PT_YIELD(pt);
	
	/** Execução normal
	Ele fica no aguardo de uma interrupção. Quando ela for dada, a função app_immediate_alert é chamada.
	Caso o limite de tempo seja atingido sem que um dispositivo tenha mandado um sinal, ele volta para o Advertising Mode.
	*/
	buffer = i;
	PT_WAIT_UNTIL(pt, buffer == 9);
	while (1) {
		if (at_ble_event_get(&event, ble_event_params, BLE_EVENT_TIMEOUT) == AT_BLE_SUCCESS)
		{
			ble_event_manager(event, ble_event_params);
		}
		
		if (app_timer_done) {
			LED_Toggle(LED0);
			timeout_count = timer_interval;
			tc_set_count_value(&tc_instance, 0);
			tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
			app_timer_done = false;
		}
	}
	PT_YIELD(pt);
	PT_END(pt);
}

/**
 * \brief Find Me - função main
 */
static struct pt pt;
int main(void)
{
	/** Inicializando a protothread com PT_INIT(). */
	PT_INIT(&pt);
	i = 0;
	for(i = 0; i < 10; i++){
		pt_find_me(&pt, i);
	}
	return 0;
}