// NMR SPVP Logging Tool software
// 13.02.2021
//current version 0.2

#include <ti/pspiom/cslr/cslr.h>
#include "upp/upp.h"
#include "proger/proger.h"
#include "uart_hduplex/uart_hduplex.h"
#include "spi/spi.h"

//----------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <c6x.h>

#include "common_data.h"

#include "Common/OMAPL138_global.h"

#include "UART/uart_messages.h"
#include "UART/UART_drv.h"

#include "Timer/clocker_drv.h"
#include "Timer/timer_drv.h"

#include "PSC/psc.h"
#include "soc_C6748.h"

#include "GPIO/gpioMux.h"
#include "GPIO/gpio.h"

#include "Galois/rscoding.h"
#include "Galois/gf_data.h"

#include "Math/nmr_math.h"
#include "Math/data_processing.h"

#include "time.h"



//#define DEBUG_PROGER
//#define DEBUG_GPIO_STATES
//#define DEBUG_ADC_FRQ_READ
//#define DEBUG_GAMMA_COUNTER
//#define USE_TIMING
//#define DEBUG_POWER_GOOD
//#define DEBUG_TIME_COUNTER
#define USE_TELEMETRIC_UART
//#define DEBUG_TELEMETRIC
//#define DEBUG_SPI
//#define DEBUG_READCONF
//#define DEBUG_READ_PROGER_STATUS
//#define DEBUG_READ_PROGER_CONN_SPD
#define USE_PRESSURE_UNIT

#define INTC_Timer			4
#define INTC_UART			5
#define INTC_UPP			6
#define INTC_GPIO			7
#ifdef USE_TELEMETRIC_UART
#define INTC_UART_Tele		8
#endif



#pragma DATA_SECTION(upp_buffer_page1, "l2ram_data");		// 4096
#pragma DATA_SECTION(data_org, "l2ram_data");				// 8192 + 2*PAD
#pragma DATA_SECTION(ui16_buffer, "l2ram_data");			// 4096 + 2*PAD
#pragma DATA_SECTION(data1, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data2, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data3, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data4, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data5, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data6, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(data7, "l2ram_data");					// 8192 + 2*PAD
#pragma DATA_SECTION(temp_data, "l2ram_data");				// 8192 + 2*PAD
#pragma DATA_SECTION(data_nmr, "sharedram_data");			// 8192 + 2*PAD
#pragma DATA_SECTION(data_sum, "sharedram_data");			// 8192 + 2*PAD
#pragma DATA_SECTION(data_fin, "sharedram_data");
#pragma DATA_SECTION(w, "l2ram_data");						// 8192 + 2*PAD
#pragma DATA_SECTION(brev, "l2ram_data");					// 64
//#pragma DATA_SECTION(timing, "l2ram_data");		// 4096

extern void intcVectorTable(void);

// ----------------------------------------------------------------------------
int defineToolState(void);
int definePinState_CmdAddr(int pinNumber, uint8_t *cmd_addr);
void init_UART_MsgData(void);
void initDeviceSettings(uint8_t device);
Bool loadDeviceSettings(int *data, int len);
void create_Clockers(void);
void onDataAvailable(QUEUE8* bytes);
//void onRingDataAvailable(int from, int to, uint8_t *ring);
void executeServiceMsg(MsgHeader *_msg_header);
void executeShortMsg(MsgHeader *_msg_header);
void executeMultypackMsg(UART_Message *uart_msg);
void responseMultypackHeader(MsgHeader *_msg_header);
void requestLastMsg();
void sendServiceMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);
void sendShortMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);
void sendHeader(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs);
void sendByteArray(uint8_t *arr, uint16_t len, CSL_UartRegsOvly uartRegs);
void sendMultyPackMsg(UART_Message *uart_msg, CSL_UartRegsOvly uartRegs);
//void prepareRelaxData(void);
//void prepareOutputData(void);
void prepareOutputByteArray(OutBuffer *out_buff, SummationBuffer *sum_buff);
void summationDataToOutput(OutBuffer *out_buff, SummationBuffer *sum_buff);
void telemetryDataToOutput(OutBuffer *out_buff);
#ifdef USE_PRESSURE_UNIT
void pressureUnitDataToOutput(OutBuffer *out_buff);
#endif
//void extractDataFromPacks(UART_Message *uart_msg, uint8_t *arr, uint16_t *len);
void generateTestEchoData(int index, int count);
Bool extractDataFromPacks(UART_Message *uart_msg, uint8_t *arr, uint16_t *len);
void executeProcPack(Data_Proc *proc, int index);
void toMeasureTemperatures();
void setDefaultCommSettings();
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static int cnt_uart_isr = 0;

//upp vars and functions -------------------------------------------------------
volatile unsigned int reg1 = 0, reg2 = 0, reg3 = 0;
unsigned short byte_count = UPP_BUFF_SIZE, line_count = 1; 	// max value for both is 64*1024-1
volatile unsigned int upp_int_status = 0;
volatile Bool uppFull;
static volatile unsigned int upp_isr_count = 0;
volatile Bool modulesEnabled;
volatile unsigned int pins_reg = 0, pins_reg_prev = 0;
volatile uint8_t pins_cmd = 0xFF, led_pin15 = 0;

volatile Bool upp_resetted = false;

CSL_UppRegsOvly UPP0Regs = (CSL_UppRegsOvly) (CSL_UPP_0_REGS);
CSL_UppRegsOvly UPPRegs;


volatile unsigned int device_serial = 0;
ToolChannel *device_channels = NULL;
int device_channel_count = 0;
Bool device_settings_OK = False;		// = True if ToolChannel data was recieved successfully.

#define STB_LOW_LVL			0
#define STB_HIGH_LVL		1
#define STB_RISING_EDGE		3
#define STB_FALLING_EDGE	4
#define STB_ERROR			5
#define STB_PINS_COUNT 		4
volatile uint8_t stb[STB_PINS_COUNT];
//uint8_t cmd_addr = 0xFF;
int check_stb();


volatile uint8_t tmpb;

unsigned char *upp_buffer;
unsigned char upp_buffer_page1[UPP_BUFF_SIZE]; 				// буфер, куда передаются данные ЯМР (из АЦП) по быстрому каналу UPP (страница 1)
unsigned char upp_buffer_page2[UPP_BUFF_SIZE]; 				// буфер, куда передаются данные ЯМР (из АЦП) по быстрому каналу UPP (страница 2)
unsigned short ui16_buffer[DATA_MAX_LEN + 2 * PAD]; 		// исходные данные ЯМР (данные из АЦП) в формате uint16_t
float data_org[DATA_MAX_LEN + 2 * PAD]; 					// буфер 0 (D0) для хранения и обработки данных
float data1[DATA_MAX_LEN + 2 * PAD]; 						// буфер 1 (D1) для хранения и обработки данных
float data2[DATA_MAX_LEN + 2 * PAD]; 						// буфер 2 (D2) для хранения и обработки данных
float data3[DATA_MAX_LEN + 2 * PAD]; 						// буфер 3 (D3) для хранения и обработки данных
float data4[DATA_MAX_LEN + 2 * PAD]; 						// буфер 4 (D4) для хранения и обработки данных
float data5[DATA_MAX_LEN + 2 * PAD]; 						// буфер 5 (D5) для хранения и обработки данных
float data6[DATA_MAX_LEN + 2 * PAD]; 						// буфер 6 (D6) для хранения и обработки данных
float data7[DATA_MAX_LEN + 2 * PAD]; 						// буфер 7 (D7) для хранения и обработки данных
float temp_data[DATA_MAX_LEN + 2 * PAD]; 					// данные ЯМР после третьего этапа обработки (оконная функция в частотной области)
float w[DATA_MAX_LEN + 2 * PAD]; 							// массив поворачивающих множителей
float 	*ptr_data_org,										// указатели на буферы данных data_org, data1, data2, data3, data4, data5, data6, data7
		*ptr_data1,
		*ptr_data2,
		*ptr_data3,
		*ptr_data4,
		*ptr_data5,
		*ptr_data6,
		*ptr_data7,
		*ptr_temp_data,
		*ptr_w;
int 	len_data_org, 										// длины данных, хранящиеся в буферах data_org, data1, data2, data3, data4, data5, data6, data7
		len_data1,
		len_data2,
		len_data3,
		len_data4,
		len_data5,
		len_data6,
		len_data7;
unsigned short *ptr_ui16_buffer;
int len_ui16_buffer; 										// длина данных, хранящихся в буфере ui16_buffer
float *bank[10]; 											// = { ptr_data_org, ptr_data1, ptr_data2, ptr_data3, ptr_data4, ptr_data5, ptr_data6, ptr_data7 };
															// контейнер с указателями на все буферы данных


float **data_heap; 											// контейнер долгосрочного хранения данных (расположен в куче)
int data_heap_len[DATA_HEAP_COUNT]; 						// массив длин данных, хранящихся в массивах контейнера data_heap

int rad; 													// параметр, используемый для БПФ

unsigned char data_fin[ALLDATA_BUFF_SIZE]; 					// буфер для всех данных, передаваемых в рабочую станцию оператора (данные ЯМР + диэл. данные + мониторинг +...)
int data_fin_counter = 0; 									// счетчик данных (типа unsigned char) в буфере data_fin
//float data_nmr[NMR_DATA_LEN + 2 * PAD]; 					// результат обработки данных ЯМР
float data_nmr[ALLDATA_BUFF_LEN + 2 * PAD]; 				// буфер для накопления данных измерений перед помещением данных в выходной буфер (затем передача по кабелю)
float*ptr_data_nmr; 										// указатель на буфер data_nmr
int data_nmr_counter = 0; 									// счетчик данных (типа float) в буфере data_nmr
float data_sum[ALLDATA_BUFF_LEN + 2 * PAD]; 				// буфер для последовательного (поточечно по мере их измерения) накопления данных перед помещением их в выходной буфер
float*ptr_data_sum; 										// указатель на буфер data_sum

OutBuffer *output_data; 									// контейнер выходных данных для последующей записи в data_fin и передачи в рабочую станцию оператора
SummationBuffer *summ_data; 								// контейнер для хранения данных, получаемых поточечно (т.е. по одной точке с каждого эхо)

//----------------------------------------------

// ************ Clocker Objects ******************
Clocker **clockers; 										// Array of Clocker objects
Clocker *app_clocker; 										// Main clocker in the application
Clocker *clocker1; 											// Clocker for incoming UART messages (for header of message)
Clocker *clocker2; 											// Clocker for incoming UART messages (for body of message)
Clocker *clocker3; 											// Clocker for "Data_ready" message
Clocker *clocker4; 											// Clocker for telemetry measurements
Clocker *clocker5; 											// Clocker for SDSP measurements (~200 ms)
// ***********************************************

volatile int uartStatus;
volatile Bool dataUnavailable;
volatile Bool transmitterFull;

volatile int app_finish = 0;

QUEUE8 *uart_queue;
volatile int in_cnt = 0;

GF_Data *gf_data;

uint8_t out_msg_number = 1; 								// номер исходящих сообщений (предполагается сквозная нумерация)
MsgHeader *in_msg_header; 									// заголовок принимаемого сообщения
MsgHeader *out_msg_header; 									// заголовок отправляемого сообщения
MsgCommSettings *msg_settings; 								// параметры для передачи сообщений: длина блоков, количество восстанавливаемых ошибок и т.д.
QUEUE8 *head_q; 											// контейнер для заголовка сообщения, принимаемого из UART
//QUEUE8 *body_q; 											// контейнер для очередного пакета сообщения, принимаемого из UART
BUFFER8 *body_q;
UART_Message in_msg; 										// принимаемое UART-сообщение
UART_Message out_msg; 										// отправляемое UART-сообщение
GetDataReq data_req; 										// требуемые оператором данные

volatile MultyStateIn msg_header_state = NOT_DEFINED; 		// состояние приема заголовка сообщения (msg_header)
//volatile MultyStateIn msg_packet_state = NOT_DEFINED; 	// состояние приема очередного пакета сообщения
volatile MultyStateIn incom_msg_state = NOT_DEFINED; 		// состояние всего принимаемого сообщения
volatile MultyStateOut outcom_msg_state = NOT_BUILT; 		// состояние всего отправляемого сообщения
volatile int pack_counter = 0; 								// счетчик приходящих пакетов принимаемого сообщения

volatile Bool input_data_enabled; 							// флаг, равный True, если прием данных разрешен
volatile Bool fpga_prg_started; 							// флаг, равный True, если программа на ПЛИС запущена (если команда NMR_TOOL_START была получена)

uint8_t msg_was_treated = 0;								// флаг, указывающий на успех/неуспех приема и обработки многопакетного сообщения (= 0 или = коду ошибки, см. MultiPackMsg_Err)

volatile NMRToolState tool_state = UNKNOWN_STATE; 			// флаг, устанавливаемый: - в READY, когда заканчивается обработка всех данных (фурье и пр.),
															// - в FREE, когда ЯМК готов к приему команд по UART и к передаче данных по UART к управляющей станции оператора,
															// - в BUSY, когда ЯМК "занят", т.е. производит измерение и/или обработку сигнала ЯМР
Processing_Params *processing_params;
Data_Proc *instr_prg; 										// блоки инструкций для обработки данных
STACKPtrF *data_stack; 										// стек для данных (массивов) типа float: data1, data2, data3, ...
float XX[XX_LEN]; 											// ячейки памяти X0, X1, X2, X3

volatile uint8_t device_id; 								// идентификатор устройства, данные которого обрабатываются по сигналу GPIO GP[1]

uint8_t pp_is_seq_done = 0;									// индикатор завершения последовательности по команде COM_STOP


// Telemetric variables -------------------------------------------------------
#ifdef USE_TELEMETRIC_UART
uint8_t telemetric_data[TELEMETRIC_UART_BUF_LEN]; 			// контейнер для телеметрических данных
volatile unsigned int UART_telemetric_counter = 0; 			// счетчик байт, приходящих от плат телеметрии
volatile unsigned int UART_telemetric_pack_counter = 0; 	// счетчик пакетов длиной TELEMETRIC_DATA_LEN байт, приходящих от плат телеметрии
volatile unsigned int UART_telemetric_local_counter = 0; 	// локальный счетчик для подсчета данных внутри пакета
volatile uint8_t telemetric_board_status = 0;
volatile TelemetryState telemetry_ready = TELE_NOT_READY; 	// флаг готовности телеметрической информации

volatile int temperature_mode = TEMP_MODE_UNKNOWN;			// тип прибора (KMRK, NMKT и SDSP имеют разный способ измерения температуры): 1 - KMRK, NMKT и другие; 2 - SPVP
volatile int temp_request_mode = TEMP_NOT_DEFINED;			// 0 - исходное состояние, 1 - получен ответ на запуск измерения температуры, 2 - получены температуры (только для SPVP)
volatile int temp_sensors = 0;								// количество сенсоров (только для SPVP)
volatile int voltage_request_mode = VOLT_NOT_DEFINED;		// 0 - исходное состояние, 1 - запуск измерений напряжений, 2 - напряжения измерены (только для SPVP)
#endif
// ----------------------------------------------------------------------------

// Pressure Unit variables ----------------------------------------------------
#ifdef USE_PRESSURE_UNIT
unsigned int press_unit_data;								// контейнер для данных прижимного устройства
volatile PressureUnitState press_unit_ready = PRESS_UNIT_NOT_READY;	// флаг готовности измерить данные прижимного устройства
#endif
// ---------------------------------------------------------------------------



CSL_UartRegsOvly uartRegs;
CSL_UartRegsOvly uartRegs_Telemetric;
UART_Settings uartSettings;
UART_Settings uartSettings_Telemetric;

CSL_TmrRegsOvly tmrRegs;
//CSL_TmrRegsOvly tmrRegs1; 								// For time measuremets
Timer_Settings timerSettings; 								//Timer_Settings timerSettings1;

clock_t t_start, t_stop, t_overhead;
#ifdef USE_TIMING
TimingProc_Buffer timing_buffer;
#endif

uint8_t free_test[100];
int free_index = 0;

Data_Cmd *instr = 0;


// Start Program **************************************************************
void main(void)
{
#ifdef USE_TIMING
	TSCL = 0;
	TSCH = 0;
#endif

	CacheALL_disable();
	srand(time(NULL));

	shutdown_ARM();
	disableARM();
	_disable_interrupts();

	// ********** Init Devices ************************************************ //
	create_Clockers();


	//_enable_interrupts(); // ??? should be _disable_interrupts(); I think??? (aivanov)

	// Timer initialization ---------------------------------------------------
	timerSettings.freq = 24000u;
	timerSettings.enabled = False;

	tmrRegs = tmr0Regs; 									// add Timer0 to application

	setup_Timer(tmrRegs, timerSettings);										printf("System timer was initialized.\n");
	setup_Timer_INTC(tmrRegs, INTC_Timer);										printf("Timer system interrupt was mapped to DSP INT%d.\n", INTC_Timer);
	// ------------------------------------------------------------------------


	// UART initialization ----------------------------------------------------
	uartSettings.BaudRate = 115200;
	uartSettings.DataBits = 8;
	uartSettings.StopBits = 1;
	uartSettings.Parity = NO_PARITY;
	uartSettings.LoopBackMode = False;
	uartSettings.FIFOMode = False;
	uartSettings.FIFOLen = 1;

	uartRegs = uart1Regs; 									// add UART1 to application (for BigGreenBoard #1 - UART1 is for communication board)

	reset_UART(uartRegs);
	setup_UART(uartRegs, uartSettings);											printf("UART1 was initialized.\n");
	setup_UART_INTC(uartRegs, INTC_UART);										printf("UART1 system interrupt was mapped to DSP INT%d.\n", INTC_UART);
	// ------------------------------------------------------------------------


	// Telemetric Board UART initialization -----------------------------------
#ifdef USE_TELEMETRIC_UART
	uartSettings_Telemetric.BaudRate = 19200;
	uartSettings_Telemetric.DataBits = 8;
	uartSettings_Telemetric.StopBits = 1;
	uartSettings_Telemetric.Parity = NO_PARITY;
	uartSettings_Telemetric.LoopBackMode = False;
	uartSettings_Telemetric.FIFOMode = True;
	uartSettings_Telemetric.FIFOLen = 1;

	uartRegs_Telemetric = uart2Regs;

	reset_UART(uartRegs_Telemetric);
	setup_UART(uartRegs_Telemetric, uartSettings_Telemetric);					printf("UART2 was initialized for the Telemetric board.\n");
	setup_UART_INTC(uartRegs_Telemetric, INTC_UART_Tele);						printf("UART2 system interrupt was mapped to DSP INT%d.\n", INTC_UART_Tele);
#endif
	// ------------------------------------------------------------------------


	// Enable all Interrupts --------------------------------------------------
	int *INTCs = (int*) calloc(5, sizeof(int));
	INTCs[0] = INTC_Timer; 					// int 4 added for Timer0
	INTCs[1] = INTC_UART; 					// int 5 added for Logging UART
	INTCs[2] = INTC_UPP; 					// int 6 added for UPP
	INTCs[3] = INTC_GPIO; 					// int 7 added for GPIO
	INTCs[4] = INTC_UART_Tele; 				// int 8 added for Telemetric UART
	enable_all_INTC(5, INTCs);													printf("All interrupts were enabled.\n");
	// ------------------------------------------------------------------------


	// Enable devices ---------------------------------------------------------
	timerSettings.enabled = True;
	enable_Timer(tmrRegs);														printf("System timer was enabled.\n");

	enable_UART(uartRegs);														printf("UART1 for the cable communication was enabled.\n");

#ifdef USE_TELEMETRIC_UART
	memset(telemetric_data, 0x00, TELEMETRIC_UART_BUF_LEN);
	enable_UART(uartRegs_Telemetric); 											printf("UART2 for the Telemetric board was enabled.\n"); // start operations on Telemetric UART
#endif
	// ------------------------------------------------------------------------

	// GPIO
	PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON,
			PSC_MDCTL_NEXT_ENABLE); 						// initialization of GPIO support in PSC module
	GPIOBank0Pin1PinMuxSetup();								// Pin Multiplexing of pin 1 of GPIO Bank 0 (ADC echo window)
	GPIOBank0Pin2PinMuxSetup();								// Pin Multiplexing of pin 2 of GPIO Bank 0
	GPIOBank0Pin3PinMuxSetup();								// Pin Multiplexing of pin 3 of GPIO Bank 0 (RF sequence)
	GPIODirModeSet(SOC_GPIO_0_REGS, 2, GPIO_DIR_INPUT);		// Sets the pin 1( GP0[1] )
	GPIODirModeSet(SOC_GPIO_0_REGS, 3, GPIO_DIR_INPUT);		// Sets the pin 2( GP0[2] )
	GPIODirModeSet(SOC_GPIO_0_REGS, 4, GPIO_DIR_INPUT);		// Sets the pin 3( GP0[3] )
	// ------------------------------------------------------------------------

	// init device settings ---------------------------------------------------
	//proger_stop();
	//device_serial = proger_rd_device_serial();
	//initDeviceSettings(device_serial);

	//main_proger_wr_pulseprog_default();
	// ------------------------------------------------------------------------


	//upp initialization ------------------------------------------------------
	_disable_interrupts();
	init_upp();																	printf("UPP was initialized.\n");
	init_upp_ints(); 															printf("UPP system interrupt was mapped to DSP INT%d.\n", INTC_UPP);	// disable for upp_check_poll usage
	_enable_interrupts();
	// ------------------------------------------------------------------------


	// Set temperature Mode ---------------------------------------------------
	proger_stop();
	device_serial = proger_rd_device_serial();
	initDeviceSettings(device_serial);
	switch (device_serial)
	{
		case NMKT:
		case KMRK:
		case NMR_KERN: 	temperature_mode = TEMP_MODE_NOT_SPVP; break;
		case SPVP:		temperature_mode = TEMP_MODE_SPVP; break;
		default: break;
	}

	temperature_mode = TEMP_MODE_SPVP;		// temporary !!!

	main_proger_wr_pulseprog_default();
	// ------------------------------------------------------------------------



	// ********** Finish (Initialization of Devices) **************************

	//uint8_t simple_uart_message[128] = {'D', 'S', 'P', '_', '0', '.', '4', '9', ' ', '(', '2', '9', '.', '1', '1', '.', '1', '6', ')', '\n', 0};
	//write_data_UART(uart1Regs, simple_uart_message, 20);

	// ********** Init variables and structs ************
	// UART message system
	init_UART_MsgData();

	// NMR data containers
	memset(&upp_buffer_page1[0], 0x0, UPP_BUFF_SIZE);
	upp_buffer = &upp_buffer_page1[0];
	memset(&data_org[0], 0x0, UPP_DATA_SIZE);
	memset(&data1[0], 0x0, UPP_DATA_SIZE);
	memset(&data2[0], 0x0, UPP_DATA_SIZE);
	memset(&data3[0], 0x0, UPP_DATA_SIZE);
	memset(&data4[0], 0x0, UPP_DATA_SIZE);
	memset(&data5[0], 0x0, UPP_DATA_SIZE);
	memset(&data6[0], 0x0, UPP_DATA_SIZE);
	memset(&data7[0], 0x0, UPP_DATA_SIZE);
	memset(&temp_data[0], 0x0, UPP_DATA_SIZE);
	memset(&w[0], 0x0, UPP_DATA_SIZE);
	ptr_data_org = data_org + PAD;
	ptr_ui16_buffer = ui16_buffer + PAD;
	ptr_data1 = data1 + PAD;
	ptr_data2 = data2 + PAD;
	ptr_data3 = data3 + PAD;
	ptr_data4 = data4 + PAD;
	ptr_data5 = data5 + PAD;
	ptr_data6 = data6 + PAD;
	ptr_data7 = data7 + PAD;
	ptr_temp_data = temp_data + PAD;
	ptr_w = w + PAD;
	len_ui16_buffer = 0;
	len_data1 = 0;
	len_data2 = 0;
	len_data3 = 0;
	len_data4 = 0;
	len_data5 = 0;
	len_data6 = 0;
	len_data7 = 0;

	bank[0] = ptr_data_org;
	bank[1] = ptr_data1;
	bank[2] = ptr_data2;
	bank[3] = ptr_data3;
	bank[4] = ptr_data4;
	bank[5] = ptr_data5;
	bank[6] = ptr_data6;
	bank[7] = ptr_data7; 									// контейнер с указателями на все буферы данных
	bank[8] = ptr_temp_data;
	bank[9] = ptr_w;

	data_heap = (float**) calloc(DATA_HEAP_COUNT, sizeof(float*));
	int i;
	for (i = 0; i < DATA_HEAP_COUNT; i++)
	{
		float *heap_arr = (float*) calloc(DATA_MAX_LEN, sizeof(float));
		data_heap[i] = heap_arr;
		data_heap_len[i] = 0;
	}

	ptr_data_nmr = data_nmr + PAD;
	output_data = (OutBuffer*) malloc(sizeof(OutBuffer));
	OutBuffer_Init(output_data, ptr_data_nmr);
	ptr_data_sum = data_sum + PAD;
	summ_data = (SummationBuffer*) malloc(sizeof(SummationBuffer));
	SummationBuffer_Init(summ_data, ptr_data_sum, ALLDATA_BUFF_LEN, &XX[0]);

	memset(&data_fin[0], 0x0, ALLDATA_BUFF_SIZE * sizeof(unsigned char));
	memset(ptr_data_nmr, 0xFF, ALLDATA_BUFF_LEN * sizeof(float)); // заполнить массив результатов обработки данных ЯМР числом NaN
	memset(ptr_data_sum, 0xFF, ALLDATA_BUFF_LEN * sizeof(float)); // заполнить массив результатов обработки данных ЯМР числом NaN

	// Math functions tabulation
	initGaussTab();
	initBiGaussTab();

	// Default Parameters for ADC data processing
	processing_params = (Processing_Params*) malloc(sizeof(Processing_Params));
	setDefaultProcParams(processing_params);

	instr_prg = (Data_Proc*) malloc(sizeof(Data_Proc));
	init_DataProc(instr_prg); 								// инициализация структуры для хранения программы обработки данных ЯМР/СДСП и т.п.

	data_stack = (STACKPtrF*) malloc(sizeof(STACKPtrF));

	device_id = 0; 											// No device

	// twiddle factors
	rad = rad_gen(CMPLX_DATA_MAX_LEN);
	tw_gen(ptr_w, CMPLX_DATA_MAX_LEN);

	// Compute overhead of calling clock() twice and init TimingData
	t_start = clock();
	t_stop = clock();
	t_overhead = t_stop - t_start;



#ifdef USE_TIMING
	uint32_t tsch = TSCH;
	uint32_t tscl = TSCL;
	TimingProc_Buffer_Init(&timing_buffer, tsch, tscl);
#endif

	upp_start(byte_count, line_count, upp_buffer);

	startClocker(clocker3);
	startClocker(clocker4);
	telemetry_ready = TELE_NOT_READY;
#ifdef USE_PRESSURE_UNIT
	press_unit_ready = PRESS_UNIT_NOT_READY;
	press_unit_data = 0;
#endif

	int upp_counter;
	volatile int soft_echo_counter = 0;
	volatile int hard_echo_counter = 0;

	volatile int error_echo_counter = 0;
	volatile int recievied_adc_points_count = 0xFFFFFFFF;
	volatile int hard_adc_points_counter = 0;

	volatile int launch_counter = 0;
	uint16_t soft_arr[NMR_DATA_LEN];
	uint16_t hard_arr[NMR_DATA_LEN];
	memset(&soft_arr[0], 0x0, 400 * sizeof(uint16_t));
	memset(&hard_arr[0], 0x0, 400 * sizeof(uint16_t));

	fpga_prg_started = False;


	instr = (Data_Cmd*) malloc(sizeof(Data_Cmd));
	init_DataProcCmd(instr);

	// Start main loop --------------------------------------------------------
	printf("Start!\n");
	P15_CLR();
	tmpb = 0;
	//volatile int ready_msg_sent = 0;
	while (app_finish == 0)
	{
		/*if (tool_state != UNKNOWN_STATE)*/ //tool_state = defineToolState();
		check_stb();

		if (stb[3] == STB_FALLING_EDGE) 	tool_state = NOT_READY;
		else if (stb[3] == STB_RISING_EDGE) tool_state = READY;
		else if (stb[3] == STB_HIGH_LVL) 	tool_state = FREE;
		else if (stb[3] == STB_LOW_LVL)	 	tool_state = BUSY;

		if (tool_state == READY) // прибор готов к приему/передаче данных по кабелю (получен сигнал GP0[3] "up")
		{
#ifdef USE_TIMING
			// proc_id:
			// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
			// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
			// byte 3 - hard_echo_counter (highest byte)
			// byte 2 - hard_echo_counter (lowest byte)
			// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
			uint32_t tsch = TSCH;
			uint32_t tscl = TSCL;

			uint8_t gpio_status = (uint8_t)((READY << 4) | 3);
			uint32_t proc_id = gpio_status;

			if (TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl) == True && launch_counter == 4)
			//if (TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl) == True && soft_echo_counter < 200)
			//if (TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl) == True && hard_echo_counter != 100)
			{
				TimingProc_Buffer_Print2(&timing_buffer);
				printf("%d\n\n", hard_echo_counter);
			}
#endif

			//printf("\n");

			launch_counter++;

			//_disable_interrupts();		// commented 12.10.2020
			QUEUE8_clear(uart_queue);
			QUEUE8_clear(head_q);
			BUFFER8_clear(body_q);

			clearMsgHeader(in_msg_header);

			msg_header_state = NOT_DEFINED;
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;
			//_enable_interrupts();			// commented 12.10.2020

			//upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
			//memset(upp_buffer, 0x0, UPP_BUFF_SIZE);

			/*
			//if (telemetry_ready == TELE_READY)
			{
				toMeasureTemperatures();
				telemetry_ready = TELE_NOT_READY;
			}*/
#ifdef USE_TELEMETRIC_UART
			if (telemetry_ready == TELE_NOT_READY)
			{
				temp_request_mode = TEMP_NOT_DEFINED;
				voltage_request_mode = VOLT_NOT_DEFINED;
				toMeasureTemperatures();
				//telemetry_ready = TELE_READY;
				//telemetry_ready = TELE_NOT_READY;
			}
#endif

/*			uint8_t pg = (uint8_t) proger_rd_pwr_pg();
			uint8_t tele_flag = 0;
			if (UART_telemetric_counter == TELEMETRIC_UART_BUF_LEN) tele_flag = 1;
#ifdef USE_PRESSURE_UNIT
			if (press_unit_ready == PRESS_UNIT_READY) tele_flag = 1;
#endif */
			unsigned int tele_flag = 0;
#ifdef USE_TELEMETRIC_UART
			if (temperature_mode == TEMP_MODE_NOT_SPVP) if (UART_telemetric_counter % TELEMETRIC_DATA_LEN == 0 && UART_telemetric_counter > 0) tele_flag = 1;
			else if (temperature_mode == TEMP_MODE_SPVP)
			{
				if (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY)
				{
					tele_flag = 1;
				}
			}
#endif
#ifdef USE_PRESSURE_UNIT
			if (press_unit_ready == PRESS_UNIT_READY) tele_flag = 1;
#endif

			uint8_t pg = (uint8_t) proger_rd_pwr_pg();
			uint8_t pp_is_started  = proger_is_started();
			pp_is_seq_done = proger_is_seq_done();
			uint8_t out_mask = pg | (tele_flag << 1) | (pp_is_started << 2) | (pp_is_seq_done << 3);
			sendByteArray(NMRTool_Ready[out_mask], SRV_MSG_LEN + 2, uartRegs);
			//ready_msg_sent++;
			//if (ready_msg_sent > 1) printf("Ready: %d\n", ready_msg_sent);

			if (timerSettings.enabled == False)
			{
				timerSettings.enabled = True;
				enable_Timer(tmrRegs);
			}

		}
		else if (tool_state == NOT_READY) // прибор завершил сеанс приема/передачи данных по кабелю (получен сигнал GP0[3] "down")
		{
#ifdef USE_TIMING
			// proc_id:
			// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
			// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
			// byte 3 - hard_echo_counter (highest byte)
			// byte 2 - hard_echo_counter (lowest byte)
			// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
			uint32_t tsch = TSCH;
			uint32_t tscl = TSCL;
			TimingProc_Buffer_Init(&timing_buffer, tsch, tscl);

			uint32_t proc_id = 0;
			uint8_t gpio_status = (uint8_t)((NOT_READY << 4) | 3);
			proc_id = gpio_status;

			TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

			memset(&data_fin[0], 0x0, ALLDATA_BUFF_SIZE * sizeof(unsigned char));
			data_fin_counter = 0;

			memset(&data_nmr[0], 0xFF, NMR_DATA_LEN * sizeof(float)); // заполнить массив результатов обработки данных ЯМР числом NaN
			data_nmr_counter = 0;

			SummationBuffer_ClearAll(summ_data);
			OutBuffer_ClearAll(output_data);

			soft_echo_counter = 0;
			error_echo_counter = 0;
			tool_state = BUSY;
			outcom_msg_state = NOT_BUILT;

			GPIOBank0Pin1_initState(GPIO_HIGH_STATE);
			sendByteArray(&NMRTool_NotReady[0], SRV_MSG_LEN + 2, uartRegs);
			//ready_msg_sent = 0;
			//printf("Not Ready: %d\n", notready_msg_sent);

			upp_counter = 0;
			//upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
			memset(upp_buffer, 0x0, UPP_BUFF_SIZE);
			upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

			if (timerSettings.enabled == True)
			{
				timerSettings.enabled = False;
				disable_Timer(tmrRegs);
			}
		}

		if (tool_state == BUSY) // прибор не доступен для приема/передачи данных по кабелю (находится в состоянии приема и обработки данных ЯМР, GP0[3] = "down")
		{
			uppFull = False;

			//if (stb[1] == 3 || stb[1] == 4)
			//{
			//	printf("Pin1 = %d,\tPins_cmd = %d\n", stb[1], pins_cmd);
			//}

			if (stb[1] == STB_FALLING_EDGE && pins_cmd == SDSP_TOOL)
			{
				device_id = SDSP_TOOL;
				int channel_data_id = proger_rd_ch_number();
				processing_params->channel_id = channel_data_id;
			}
			if (stb[1] == STB_RISING_EDGE && device_id == SDSP_TOOL /*&& pins_cmd != 0x00*/)
			{
				setupDDR2Cache();
				enableCacheL1();

				int proc_index = pins_cmd;
				if (proc_index < MAX_PROCS)
				{
					move_ToFirstDataProcCmd(proc_index - 1, instr_prg);
					executeProcPack(instr_prg, proc_index - 1);
				}

				device_id = 0; // No device
				disableCache();
			}
			// *********************************************************

			// ******************** NMR Tool... ************************
			//if (pin1_state == GPIO_FALL_STATE && cmd_addr == 0xFE)
			if (stb[1] == STB_FALLING_EDGE && pins_cmd == NMR_TOOL)
			{
				device_id = NMR_TOOL;
				int channel_data_id = proger_rd_ch_number();
				processing_params->channel_id = channel_data_id;

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_FALLING_EDGE << 4) | 1);
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

			}
			//else if (pin1_state == GPIO_RISE_STATE && device_id == NMR_TOOL && cmd_addr != 0x00)
			if (stb[1] == STB_RISING_EDGE && device_id == NMR_TOOL && pins_cmd != 0x00)
			{
				//P15_SET();
				upp_resetted = upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
				if (upp_resetted == false)
				{
					upp_resetted = true;
				}

				upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

				/*t_start = clock();
				t_stop = clock();
				t_overhead = t_stop - t_start;
				t_start = clock();*/

				setupDDR2Cache();
				enableCacheL1();

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				//dummyDelay(2);

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_RISING_EDGE << 4) | 1);
				hard_echo_counter = proger_rd_echo_count();
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (hard_echo_counter << 8) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif
#ifndef USE_TIMING
				dummyDelay(2);
				hard_echo_counter = proger_rd_echo_count();
#endif
				recievied_adc_points_count = proger_rd_adc_points_count();
				upp_counter++;
				soft_echo_counter++;

				//soft_arr[soft_echo_counter - 1] = soft_echo_counter;
				//hard_arr[soft_echo_counter - 1] = hard_echo_counter;
				//if (soft_echo_counter != hard_echo_counter) error_echo_counter++;

				/*if (hard_echo_counter == 0)
				 {
				 printf("hard counter = %d\tsoft counter = %d\n", hard_echo_counter, soft_echo_counter);
				 }*/

				processing_params->current_echo = hard_echo_counter;
				processing_params->points_count = recievied_adc_points_count;
				processing_params->echo_count = soft_echo_counter; // это неверно, т.к. счетчик soft_echo_counter не обнуляется между спадами

				//int group_index = proger_rd_group_index();
				//processing_params->group_index = group_index;

				//setupDDR2Cache();
				//enableCacheL1();

				int proc_index = pins_cmd;
				if (proc_index <= MAX_PROCS)
				{
					move_ToFirstDataProcCmd(proc_index - 1, instr_prg);
					executeProcPack(instr_prg, proc_index - 1);
				}

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				tsch = TSCH;
				tscl = TSCL;

				proc_id = 0;
				//gpio_status = 1;
				uint32_t h_echo_number = proger_rd_echo_count();
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (h_echo_number << 8) | (uint32_t)pins_cmd;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

				//if (proc_index != 1) printf("nmr = %d, ", proc_index);

				device_id = 0; // No device

				memset(upp_buffer, 0x0, UPP_BUFF_SIZE);
				disableCache();

//#3			upp_resetted = upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
				if (upp_resetted == false)
				{
					upp_resetted = true;
				}

				upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР
				//dummyDelay(2);			// delay 50 mks

				//t_stop = clock();
				//printf("\t NMR data processing time: %d clock cycles\n", (t_stop - t_start) - t_overhead);
				//P15_CLR();
			}

			// ******************** GAMMA Tool... ************************
			if (stb[1] == STB_FALLING_EDGE && pins_cmd == GAMMA_TOOL)
			{
				device_id = GAMMA_TOOL;
				int channel_data_id = proger_rd_ch_number();
				processing_params->channel_id = channel_data_id;

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_FALLING_EDGE << 4) | 1);
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif
			}
			if (stb[1] == STB_RISING_EDGE && device_id == GAMMA_TOOL && pins_cmd != 0x00)
			{
				setupDDR2Cache();
				enableCacheL1();


#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_RISING_EDGE << 4) | 1);
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

				int proc_index = pins_cmd;
				if (proc_index < MAX_PROCS)
				{
					move_ToFirstDataProcCmd(proc_index - 1, instr_prg);
					executeProcPack(instr_prg, proc_index - 1);
				}

				device_id = 0; // No device
				disableCache();

				/*led_pin15 = ~led_pin15;
				 if (led_pin15) P15_SET();
				 else P15_CLR();*/

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				tsch = TSCH;
				tscl = TSCL;

				proc_id = 0;
				gpio_status = 1;
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif
			}

			// ******************** DUMMY Tool... ************************
			//if (pin1_state == GPIO_FALL_STATE && cmd_addr == DUMMY_TOOL)	//cmd_addr 0xC8 - фиктивное устройство
			if (stb[1] == STB_FALLING_EDGE && pins_cmd == DUMMY_TOOL)
			{
				device_id = DUMMY_TOOL;
				int channel_data_id = proger_rd_ch_number();
				processing_params->channel_id = channel_data_id;

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_FALLING_EDGE << 4) | 1);
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

			}
			//if (pin1_state == GPIO_RISE_STATE && device_id == DUMMY_TOOL && cmd_addr != 0x00)//
			if (stb[1] == STB_RISING_EDGE && device_id == DUMMY_TOOL && pins_cmd != 0x00)
			{
				//P15_SET();

				setupDDR2Cache();
				enableCacheL1();

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				uint32_t tsch = TSCH;
				uint32_t tscl = TSCL;

				uint32_t proc_id = 0;
				uint8_t gpio_status = (uint8_t)((STB_RISING_EDGE << 4) | 1);
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif

				//int proc_index = cmd_addr;
				int proc_index = pins_cmd;
				if (proc_index < MAX_PROCS)
				{
					move_ToFirstDataProcCmd(proc_index - 1, instr_prg);
					executeProcPack(instr_prg, proc_index - 1);
				}

				device_id = 0; // No device
				disableCache();

				//printf("dummy = %d, ", proc_index);

#ifdef USE_TIMING
				// proc_id:
				// |-- H.byte 4 --|--- byte 3 ---|--- byte 2 ---|-- 1 byte 1 --|
				// H.byte 4 (highest byte) - id устройства, с которым в данный момент идет работа (см. NMR_TOOL, GAMMA_TOOL и т.д.)
				// byte 3 - hard_echo_counter (highest byte)
				// byte 2 - hard_echo_counter (lowest byte)
				// L.byte 1 (lowest byte) - GPIO 1-3 state: 4 h.bits - GPIO status, 4 l.bits - GPIO number (1,2 or 3)
				tsch = TSCH;
				tscl = TSCL;

				proc_id = 0;
				gpio_status = 1;
				proc_id = ((proc_id | (uint32_t)device_id) << 24) | (uint32_t)gpio_status;

				TimingProc_Buffer_Add(&timing_buffer, proc_id, tsch, tscl);
#endif
			}

			if (timerSettings.enabled == True)
			{
				timerSettings.enabled = False;
				disable_Timer(tmrRegs);
			}
		}

		if (tool_state == FREE) // прибор находится в состоянии приема/передачи данных по кабелю (GP0[3] = "up")
		{
			if (timerSettings.enabled == False)
			{
				timerSettings.enabled = True;
				enable_Timer(tmrRegs);
			}
		}

		// если incom_msg_state == NOT_DEFINED или STARTED
		if (incom_msg_state < FINISHED)
		{
			if (!dataUnavailable)
			{
				//_disable_interrupts();			// commented 12.10.2020
				dataUnavailable = True;
				onDataAvailable(uart_queue);
				//_enable_interrupts();				// commented 12.10.2020
			}
		}
		else if (incom_msg_state == FINISHED)
		{
			// если заголовок служебного сообщения (а значит и все служебное сообщение) успешно принят и декодирован
			if (in_msg_header->msg_type == MTYPE_SERVICE)
			{
				//_disable_interrupts();			// commented 12.10.2020
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				//QUEUE8_clear(body_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();				// commented 12.10.2020

				executeServiceMsg(in_msg_header);

				clearMsgHeader(in_msg_header);
				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;

				//startClocker(clocker3);
			}
			// если заголовок короткого сообщения (а значит и все короткое сообщение) успешно принят и декодирован
			else if (in_msg_header->msg_type == MTYPE_SHORT)
			{
				//_disable_interrupts();			// commented 12.10.2020
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				//QUEUE8_clear(body_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();				// commented 12.10.2020

				executeShortMsg(in_msg_header);

				clearMsgHeader(in_msg_header);
				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;
			}
			// если успешно принят и декодирован заголовок многопакетного сообщения
			else if (in_msg_header->msg_type == MTYPE_MULTYPACK)
			{
				//_disable_interrupts();			// commented 12.10.2020
				QUEUE8_clear(uart_queue);
				QUEUE8_clear(head_q);
				//QUEUE8_clear(body_q);
				BUFFER8_clear(body_q);
				//_enable_interrupts();				// commented 12.10.2020

				stopClocker(clocker2);

				executeMultypackMsg(&in_msg);

				clearMsgHeader(in_msg_header);

				int i;
				uint16_t pack_cnt = in_msg.pack_cnt;
				for (i = 0; i < pack_cnt; i++) free(in_msg.msg_packs[i]); // added 3.09.2015
				in_msg.pack_cnt = 0;
				in_msg.msg_header->pack_count = 0;
				//deleteMsgPackets(&in_msg);

				msg_header_state = NOT_DEFINED;
				incom_msg_state = NOT_DEFINED;
			}
		}
		// если принимается многопакетное сообщение (заголовок уже принят)
		else if (incom_msg_state == PACKS_STARTED)
		{
			if (!dataUnavailable)
			{
				//_disable_interrupts();			// commented 12.10.2020
				dataUnavailable = True;
				onDataAvailable(uart_queue);
				//_enable_interrupts();				// commented 12.10.2020
			}
		}
		// если incom_msg_state = FAILED или incom_msg_state = TIMED_OUT
		else if (incom_msg_state == FAILED || incom_msg_state == TIMED_OUT)
		{
			if (tmpb == 1) requestLastMsg();
			tmpb = 0;

			//_disable_interrupts();				// commented 12.10.2020
			clearMsgHeader(in_msg_header);
			QUEUE8_clear(uart_queue);
			QUEUE8_clear(head_q);
			//QUEUE8_clear(body_q);
			BUFFER8_clear(body_q);
			msg_header_state = NOT_DEFINED;
			incom_msg_state = NOT_DEFINED;
			//_enable_interrupts();					// commented 12.10.2020
		}

		//if (incom_msg_state == NOT_DEFINED && tool_state != BUSY)
		/*if (incom_msg_state == NOT_DEFINED && tool_state == FREE)
		{
			if (telemetry_ready == TELE_NOT_READY && temp_request_mode == TEMP_NOT_DEFINED)
			{
				temp_request_mode = TEMP_NOT_DEFINED;
				voltage_request_mode = VOLT_NOT_DEFINED;
				toMeasureTemperatures();

				//telemetry_ready = TELE_NOT_READY;
			}
		}*/

		//uint8_t pp_is_started  = proger_is_started();
		uint8_t _pp_is_seq_done = proger_is_seq_done();
		if (_pp_is_seq_done)
		{
			uint8_t clocker_state = getClockerState(clocker3);
			if (clocker_state == CLR_STOPPED)
			{
				pp_is_seq_done = _pp_is_seq_done;

				fpga_prg_started = False;
				//proger_stop();		// check it ! // эта комманда приведет к сбросу флажка proger_is_seq_done() в ноль.
													  //Включение/выключение этой команды может сильно сказаться на работоспособности ЯМР-Керн в части определения конца последовательности

				timerSettings.enabled = True;
				enable_Timer(tmrRegs);

				startClocker(clocker3);
				startClocker(clocker4);
				incom_msg_state = NOT_DEFINED;
			}
		}
	}
	// ------------------------------------------------------------------------

	disable_Timer(tmrRegs);

	//_disable_interrupts();					// commented 12.10.2020

	// Finish...
	printf("End of NMR_Tool.\n\n");

}/* End of main */

int defineToolState(void)
{
	int state = UNKNOWN_STATE;

	switch (GPIOBank0Pin3_State())
	{
	case GPIO_RISE_STATE:	state = READY;	break;
	case GPIO_HIGH_STATE:	state = FREE;	break;
	case GPIO_FALL_STATE:	state = NOT_READY;	break;
	case GPIO_LOW_STATE:	state = BUSY;	break;
	default: break;
	}

	return state;
}

int definePinState_CmdAddr(int pinNumber, uint8_t * volatile cmd_addr)
{
	*cmd_addr = 0xFF;
	int pin_state = 0xFF;

	switch (pinNumber)
	{
	case 0:	pin_state = GPIOBank0Pin0_CmdAddr_State(cmd_addr);	break;
	case 1:	pin_state = GPIOBank0Pin1_CmdAddr_State(cmd_addr);	break;
	case 2:	pin_state = GPIOBank0Pin2_CmdAddr_State(cmd_addr);	break;
	case 3:	pin_state = GPIOBank0Pin3_CmdAddr_State(cmd_addr);	break;
	case 4:	pin_state = GPIOBank0Pin4_CmdAddr_State(cmd_addr);	break;
	default: break;
	}

	return pin_state;
}

// Использование START_BYTE и STOP_BYTE
void onDataAvailable(QUEUE8* bytes)
{
	static int cnt = 0;
	if (input_data_enabled)
	{
		cnt++;
		if (msg_header_state < FINISHED)
		{
			int sz = QUEUE8_count(bytes);
			if (msg_header_state == STARTED && sz < HEADER_LEN) return;

			if (incom_msg_state == STARTED && sz == HEADER_LEN)
			{
				while (sz-- > 0) QUEUE8_put(QUEUE8_get(bytes), head_q);

				int res = findMsgHeader(head_q, in_msg_header, gf_data);
				if (res == E_RS_OK)
				{
					res = checkMsgHeader(in_msg_header);
					if (res == E_MSG_OK)
					{
						msg_header_state = FINISHED;
						if (in_msg_header->msg_type == MTYPE_SERVICE || in_msg_header->msg_type == MTYPE_SHORT) incom_msg_state = FINISHED;
						else if (in_msg_header->msg_type == MTYPE_MULTYPACK)
						{
							stopClocker(clocker1);
							QUEUE8_clear(bytes);
							gf_data->index_body = in_msg_header->rec_errs - 1;
							incom_msg_state = PACKS_STARTED;
							responseMultypackHeader(in_msg_header);
							msg_was_treated = MSG_NO_PACKS;
							int pack_count = (int) in_msg_header->pack_count;
							int pack_len = (int) in_msg_header->pack_len;
							//int inter_pack_delays = msg_settings->pack_delay * pack_count;
							uint64_t packs_delay = 1000 /* + inter_pack_delays * 2*/ + (pack_count * pack_len) / (uartSettings.BaudRate / 8.0) * 1000 * 2; // 2 - двойной запас по времени // it was 50
							//uint64_t packs_delay = 5000 + (pack_count * pack_len) / (uartSettings.BaudRate / 8.0) * 1000;
							initClocker(packs_delay, clocker2_ISR, clocker2);
							startClocker(clocker2);
						}
					}
					else
					{
						msg_header_state = FAILED;
						incom_msg_state = FAILED;
					}
					stopClocker(clocker1);
					return;
				}
				else if (res == E_RS_NOTFOUND)
				{
					msg_header_state = FAILED;
					incom_msg_state = FAILED;
				}
				else if (res == E_RS_LEN)
				{
					msg_header_state = FAILED;
					incom_msg_state = FAILED;
				}

				QUEUE8_clear(head_q);
				//clearMsgHeader(in_msg_header);
				//msg_header_state = NOT_DEFINED;
			}
			else if (msg_header_state == NOT_DEFINED && sz > 1)
			{
				QUEUE8_clear(bytes);
			}
		}

		if (incom_msg_state == PACKS_STARTED)
		{
			msg_was_treated = MSG_DATA_NOT_ALL;
			int sz = QUEUE8_count(bytes);
			int pack_count = (int) in_msg_header->pack_count;
			int pack_len = (int) in_msg_header->pack_len;
			//printf(" sz = %d ;",sz);
			if (sz >= pack_count * pack_len)
			{
				setupDDR2Cache();
				enableCacheL1();
				//while (sz-- > 0) QUEUE8_put(QUEUE8_get(bytes), body_q);
				while (sz-- > 0) BUFFER8_put(QUEUE8_get(bytes), body_q);
				incom_msg_state = PACKS_FINISHED;
				msg_was_treated = MSG_DECODE_ERR;
				disableCache();
			}

			if (incom_msg_state == PACKS_FINISHED)
			{
				//stopClocker(clocker2);

				// Enable DDR cache
				setupDDR2Cache();
				enableCacheL1();

				//int res = findMsgPackets(body_q, &in_msg, gf_data);
				int res = findMsgPackets2(body_q, &in_msg, gf_data);
				if (res == E_RS_OK)
				{
					msg_was_treated = MSG_BAD_PACKETS;
					res = checkMsgPackets(&in_msg);
					if (res == E_MSG_OK)
					{
						msg_was_treated = MSG_EXTRACT_ERR;
						msg_header_state = FINISHED;
						incom_msg_state = FINISHED;
					}
					else
					{
						int i;
						for (i = 0; i < in_msg.pack_cnt; i++) free(in_msg.msg_packs[i]); // added 3.09.2015
						in_msg.pack_cnt = 0;

						msg_header_state = FAILED;
						incom_msg_state = FAILED;
					}
				}
				else
				{
					int i;
					for (i = 0; i < in_msg.pack_cnt; i++) free(in_msg.msg_packs[i]); // added 3.09.2015
					in_msg.pack_cnt = 0;

					msg_was_treated = MSG_DECODE_ERR;
					msg_header_state = FAILED;
					incom_msg_state = FAILED;
				}

				disableCache();
			}
		}
	}
}
/////


void executeServiceMsg(MsgHeader *_msg_header)
{
	sendServiceMsg(_msg_header, uartRegs);
}

void executeShortMsg(MsgHeader *_msg_header)
{
	uint8_t cmd = _msg_header->data[0];

	switch (cmd)
	{
	case GET_DATA:
	{
		if (tool_state == UNKNOWN_STATE) return;
		//if (fpga_prg_started == False && UART_telemetric_counter % TELEMETRIC_DATA_LEN > 0) return;
		if (temperature_mode == TEMP_MODE_SPVP && fpga_prg_started == False && telemetry_ready == TELE_NOT_READY) return;

		Bool to_out = (fpga_prg_started == False);
#ifdef USE_TELEMETRIC_UART
		if (temperature_mode == TEMP_MODE_NOT_SPVP) to_out = to_out && (UART_telemetric_counter % TELEMETRIC_DATA_LEN > 0);
		else if (temperature_mode == TEMP_MODE_SPVP) to_out = to_out && (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY);
#endif
#ifdef USE_PRESSURE_UNIT
		int attempts_cnt = 5;
		if (proger_read_mtr_adc_status())
		{
			proger_mtr_adc_start_conversion();
			while (attempts_cnt > 0)
			{
				dummyDelay(1);
				if (proger_read_mtr_adc_status()) break;
				else --attempts_cnt;
			}
		}

		to_out = to_out && (attempts_cnt == 0);
#endif
		if (to_out) return;

		//printf("Get Data !\n");

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_MULTYPACK;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		hdr->pack_count = 0;
		hdr->pack_len = msg_settings->pack_len;
		hdr->block_len = msg_settings->block_len;
		hdr->rec_errs = msg_settings->rec_errs;

		uint16_t *pos = (uint16_t*) malloc(sizeof(uint16_t));
		*pos = 0;
		uint16_t dpos = *pos;
		uint8_t pack_number = 1;

		// Enable DDR cache
		setupDDR2Cache();
		enableCacheL1();

#ifdef USE_TELEMETRIC_UART
		if (telemetry_ready == TELE_READY)
		{
			telemetryDataToOutput(output_data);
			//printf("Telemetry is ready!\n");
		}
#endif
#ifdef USE_PRESSURE_UNIT
		if (press_unit_ready == PRESS_UNIT_READY)
		{
			pressureUnitDataToOutput(output_data);
		}
#endif

		//telemetryDataToOutput(output_data);
		summationDataToOutput(output_data, summ_data);
		prepareOutputByteArray(output_data, summ_data);

		int pack_len = msg_settings->pack_len;
		if (msg_settings->packlen_autoadjust == True)
		{
			int rs_part_len = 2 * hdr->rec_errs;
			pack_len = estimateBestPackLen(data_fin_counter + 3, hdr->block_len, rs_part_len);
		}
		hdr->pack_len = pack_len;

		while (dpos < data_fin_counter + 3)
		{
			//MsgPacket* pack = (MsgPacket*) malloc(sizeof(MsgPacket));
			MsgPacket* pack = out_msg.msg_packs[pack_number - 1];
			clearMsgPacket(pack);
			pack->pack_len = hdr->pack_len;
			pack->block_len = hdr->block_len;
			pack->msg_id = hdr->id;
			pack->packet_number = pack_number;
			pack->rec_errs = gf_data->index_body + 1; // index_body есть степень полинома. Кол-во исправляемых ошибок = index_body+1
			pack->start_marker = MTYPE_PACKET;

			pushDataToMsgPacket(&data_fin[0], data_fin_counter + 3, pos, pack, gf_data);

			out_msg.msg_packs[pack_number - 1] = pack;
			out_msg.pack_cnt = pack_number;
			hdr->pack_count = pack_number;

			pack_number++;
			dpos = *pos;
		}
		free(pos);

		disableCache();

		sendMultyPackMsg(&out_msg, uartRegs);
		break;
	}
	case HEADER_OK:
	{
		if (outcom_msg_state == HEADER_SENT)
		{
			sendMultyPackMsg(&out_msg, uartRegs);

			clearMsgHeader(out_msg.msg_header);

			/*if (tool_state == UNKNOWN_STATE)
			{
				startClocker(clocker3);
			}*/

			/*uint16_t pack_cnt = out_msg.pack_cnt;
			 int i;
			 for (i = 0; i < pack_cnt; i++)
			 {
			 free(out_msg.msg_packs[i]);
			 //clearMsgPacket(out_msg.msg_packs[i]);
			 }*/
		}
		break;
	}
	case NMRTOOL_CONNECT:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = NMRTOOL_CONNECT;
		hdr->data[1] = (uint8_t)(device_serial & 0xFF);

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;

		clearMsgHeader(out_msg.msg_header);

		timerSettings.enabled = True;
		enable_Timer(tmrRegs);

		proger_stop();

		break;
	}
	case NMRTOOL_CONNECT_DEF:
		{
			setDefaultCommSettings();

			MsgHeader *hdr = out_msg.msg_header;
			hdr->msg_type = MTYPE_SHORT;
			hdr->reader = PC_MAIN;
			hdr->writer = LOGGING_TOOL;
			hdr->id = _msg_header->id;
			memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
			hdr->data[0] = NMRTOOL_CONNECT_DEF;
			hdr->data[1] = (uint8_t)(device_serial & 0xFF);

			sendShortMsg(hdr, uartRegs);
			outcom_msg_state = MESSAGE_SENT;

			clearMsgHeader(out_msg.msg_header);

			timerSettings.enabled = True;
			enable_Timer(tmrRegs);

			proger_stop();

			break;
		}
	case NMRTOOL_START:
	{
		proger_stop();

		fpga_prg_started = True;

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		stopClocker(clocker3);
		stopClocker(clocker4);	// added 16.08.2017
		incom_msg_state = NOT_DEFINED;
		//tool_state = FREE; 	// commented 16.03.2016
		//upp_reset_soft(); // перезапуск DMA, чтобы не дописывались данные в upp_buffer в процессе обработки
		//upp_start(byte_count, line_count, upp_buffer); // старт UPP канала для приема новых данных ЯМР

		timerSettings.enabled = False;
		disable_Timer(tmrRegs);

		proger_start();
		// Added 9.03.2019 --------------------------
		/*uint8_t pp_is_started  = proger_is_started();
		int attempts = 0;
		while (attempts < 10 && !pp_is_started)
		{
			proger_start();
			dummyDelay(1);
			pp_is_started  = proger_is_started();
			attempts++;
			if (!pp_is_started) printf("not started yet!\n");
		}
		// ------------------------------------------
		*/

		break;
	}
	case NMRTOOL_STOP:
	{
		fpga_prg_started = False;

		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		//tool_state = UNKNOWN_STATE;	// commented 16.03.2016
		proger_stop();

		timerSettings.enabled = True;
		enable_Timer(tmrRegs);

		startClocker(clocker3);
		startClocker(clocker4);		// added 16.08.2017
		incom_msg_state = NOT_DEFINED;

		break;
	}
	case PRESS_UNIT_OPEN:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// move up the pressure unit (Open)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_UP);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_UP);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 1000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		//printf("OPEN! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	case PRESS_UNIT_CLOSE:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// move down the pressure unit (Close)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_DOWN);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_DOWN);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 1000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		//printf("CLOSE! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	case PRESS_UNIT_STOP:
	{
		MsgHeader *hdr = out_msg.msg_header;
		hdr->msg_type = MTYPE_SHORT;
		hdr->reader = PC_MAIN;
		hdr->writer = LOGGING_TOOL;
		hdr->id = _msg_header->id;
		memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
		hdr->data[0] = DATA_OK;

		sendShortMsg(hdr, uartRegs);
		outcom_msg_state = MESSAGE_SENT;
		clearMsgHeader(out_msg.msg_header);

		// stop the pressure unit (Stop)
		dummyDelay(1);
		unsigned char res = proger_cmd_mtr(CMD_MOTOR_STOP);
		dummyDelay(1);
		//res = proger_cmd_mtr(CMD_MOTOR_STOP);
		unsigned char cmd_mtr_state = proger_read_mtr_status();

		clocker4->max_val = 3000;
		startClocker(clocker4);
		incom_msg_state = NOT_DEFINED;
		//printf("STOP! Motor status : %X\n", cmd_mtr_state);
		break;
	}
	default: break;
	}
}

void executeMultypackMsg(UART_Message *uart_msg)
{
	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = uart_msg->msg_header->id;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));

	if (uart_msg->pack_cnt > 0)
	{
		uint8_t cmd = uart_msg->msg_packs[0]->data[PACK_HEAD_LEN];
		if (cmd == DATA_PROC)
		{
			proger_stop();

			enableCacheL1();
			uint8_t *data_arr = (uint8_t*) calloc(MAX_BODY_LEN, sizeof(uint8_t));
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, data_arr, &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;

				uint16_t prg_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = prg_len + PACK_HEAD_LEN;

				if (data_arr[pos++] == 0xFF)
				{
					cmd = data_arr[pos++];
					if (cmd == FPGA_PRG)
					{
						uint16_t instr_len = (uint16_t) data_arr[pos] | ((uint16_t) data_arr[pos + 1] << 8);
						clear_AllDataProc(instr_prg);
						fill_DataProc(instr_prg, data_arr + pos + 2, instr_len);

						hdr->data[0] = DATA_OK;
						hdr->data[1] = msg_was_treated;
						sendShortMsg(hdr, uartRegs);

						proger_mem_init();
						proger_wr_pulseprog(data_arr + PACK_HEAD_LEN, (unsigned int) (prg_len));

						//hdr->data[0] = DATA_OK;
						//sendShortMsg(hdr, uartRegs);

						outcom_msg_state = MESSAGE_SENT;
						clearMsgHeader(out_msg.msg_header);

						fpga_prg_started = True;

						stopClocker(clocker3);
						stopClocker(clocker4);		// added 16.08.2017
						incom_msg_state = NOT_DEFINED;
						tool_state = FREE;
						proger_start();

						free(data_arr);

						return;
					}
				}
			}

			free(data_arr);
			disableCache();

			hdr->data[0] = DATA_FAILED;
			hdr->data[1] = msg_was_treated;
			sendShortMsg(hdr, uartRegs);

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			startClocker(clocker3);
			startClocker(clocker4);		// added 16.08.2017
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;
			proger_start();
		}
		else if (cmd == SET_WIN_PARAMS)
		{
			enableCacheL1();

			uint8_t data_arr[64];
			float fdata_arr[16];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;
				uint16_t prg_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = PACK_HEAD_LEN;

				memcpy(&fdata_arr[0], &data_arr[pos], prg_len * sizeof(uint8_t));

				processing_params->echo_func = (uint8_t) fdata_arr[0];
				processing_params->echo_x0 = (int) fdata_arr[1];
				processing_params->echo_sigma = (int) fdata_arr[2];

				processing_params->spectr_func = (uint8_t) fdata_arr[3];
				processing_params->spectr_x0 = (int) fdata_arr[4];
				processing_params->spectr_sigma = (int) fdata_arr[5];

				hdr->data[0] = DATA_OK;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			else
			{
				hdr->data[0] = DATA_FAILED;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			//startClocker(clocker3);

			//free(data_arr);
			disableCache();
		}
		else if (cmd == SET_COMM_PARAMS)
		{
			enableCacheL1();

			uint8_t data_arr[64];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;
				uint16_t pos = PACK_HEAD_LEN;

				msg_settings->pack_len = (uint8_t) data_arr[pos];
				msg_settings->block_len = (uint8_t) data_arr[pos + 1];
				msg_settings->rec_errs = (uint8_t) data_arr[pos + 2];
				gf_data->index_body = msg_settings->rec_errs - 1;

				uint8_t bools = (uint8_t) data_arr[pos + 3];
				msg_settings->packlen_autoadjust = (0x1 & bools);
				msg_settings->antinoise_coding = (0x2 & bools) >> 1;
				msg_settings->interleaving = (0x4 & bools) >> 2;

				msg_settings->pack_delay = data_arr[pos + 4];

				hdr->data[0] = DATA_OK;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			else
			{
				hdr->data[0] = DATA_FAILED;
				hdr->data[1] = msg_was_treated;
				sendShortMsg(hdr, uartRegs);
			}
			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			//startClocker(clocker3);

			disableCache();
			//proger_start();
		}
		else if (cmd == LOG_TOOL_SETTINGS)
		{
			enableCacheL1();

			uint8_t data_arr[1000];
			uint16_t len = 0;
			Bool res = extractDataFromPacks(uart_msg, &data_arr[0], &len);
			if (res == True)
			{
				msg_was_treated = MSG_OK;

				uint16_t data_len = (uint16_t) data_arr[1] | ((uint16_t) data_arr[2] << 8);
				uint16_t pos = PACK_HEAD_LEN;

				uint32_t ui32_data[250];
				int i;
				for (i = 0; i < data_len/sizeof(uint32_t); i++)
				{
					uint32_t v32 = data_arr[pos] | (data_arr[pos+1] << 8) | (data_arr[pos+2] << 16) | (data_arr[pos+3] << 24);
					ui32_data[i] = v32;
					pos += sizeof(uint32_t);
				}
				if (loadDeviceSettings(&ui32_data[0], i))
				{
					hdr->data[0] = DATA_OK;
					hdr->data[1] = msg_was_treated;
					hdr->data[2] = (uint8_t)(device_serial & 0xFF);
					device_settings_OK = True;
				}
				else
				{
					hdr->data[0] = DATA_FAILED;
					hdr->data[1] = msg_was_treated;
					hdr->data[2] = (uint8_t)(device_serial & 0xFF);
					device_settings_OK = False;
				}
			}

			sendShortMsg(hdr, uartRegs);

			incom_msg_state = NOT_DEFINED;
			tool_state = FREE;

			outcom_msg_state = MESSAGE_SENT;
			clearMsgHeader(out_msg.msg_header);

			disableCache();
		}
	}
}

void responseMultypackHeader(MsgHeader *_msg_header)
{
	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = _msg_header->id;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
	hdr->data[0] = HEADER_OK;

	sendShortMsg(hdr, uartRegs);
	outcom_msg_state = MESSAGE_SENT;
}


void requestLastMsg()
{
	Bool wasMsgLost = False;
	int uart_queue_size = QUEUE8_count(uart_queue);
	int i;
	for (i = 0; i < uart_queue_size; i++)
	{
		if (QUEUE8_at(i, uart_queue) == MTYPE_SHORT)
		{
			wasMsgLost = True;
			if (uart_queue_size > i + 3)
			{
				if (QUEUE8_at(i+3, uart_queue) == HEADER_OK)
				{
					sendMultyPackMsg(&out_msg, uartRegs);
					clearMsgHeader(out_msg.msg_header);
					return;
				}
				else if (QUEUE8_at(i+3, uart_queue) == GET_DATA)
				{
					int j;
					for (j = 0; j < i; j++) QUEUE8_get(uart_queue);

					int new_size = QUEUE8_count(uart_queue);
					if (new_size < HEADER_LEN) return;

					uint8_t arr[HEADER_LEN];
					for (j = 0; j < HEADER_LEN; j++)
					{
						arr[j] = QUEUE8_get(uart_queue);
					}
					QUEUE8_clear(uart_queue);

					for (j = 0; j < HEADER_LEN; j++)
					{
						QUEUE8_put(arr[j], uart_queue);
					}

					msg_header_state = STARTED;
					incom_msg_state = STARTED;
					//tmpb = 1;
					onDataAvailable(uart_queue);
					return;

				}
			}
		}
		//if (QUEUE8_at(i, uart_queue) == MTYPE_MULTYPACK) wasMsgLost = True;
	}
	if (wasMsgLost == False) return;

	QUEUE8_clear(uart_queue);

	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = 0;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
	hdr->data[0] = REPEAT_CMD;

	sendShortMsg(hdr, uartRegs);
	outcom_msg_state = MESSAGE_SENT;
	incom_msg_state = NOT_DEFINED;
	msg_header_state = NOT_DEFINED;
	tool_state = FREE;

	stopClocker(clocker1);
}

void create_Clockers(void)
{
	// *********** create clockers *************
	clockers = (Clocker**) calloc(NUM_CLOCKERS, sizeof(Clocker*));

	// create system application clocker
	app_clocker = (Clocker*) malloc(sizeof(Clocker));
	clockers[0] = app_clocker;
	initClocker(CLR_MAX_VALUE, app_clocker_ISR, app_clocker);

	// create UART message clockers (for header trapping)
	clocker1 = (Clocker*) malloc(sizeof(Clocker));
	clockers[1] = clocker1;
	initClocker(100, clocker1_ISR, clocker1); // it was 20

	// create UART message clockers (for packet trapping)
	clocker2 = (Clocker*) malloc(sizeof(Clocker));
	clockers[2] = clocker2;
	initClocker(1000, clocker2_ISR, clocker2); // it was 120

	// create clocker for repetition time tests (delay between pulse sequences)
	clocker3 = (Clocker*) malloc(sizeof(Clocker));
	clockers[3] = clocker3;
	initClocker(1000, clocker3_ISR, clocker3);

	// create clocker for telemetry measurements (delay between measurements)
	clocker4 = (Clocker*) malloc(sizeof(Clocker));
	clockers[4] = clocker4;
	initClocker(3000, clocker4_ISR, clocker4);

	// create clocker for SDSP measurements (delay for measurements)
	clocker5 = (Clocker*) malloc(sizeof(Clocker));
	clockers[5] = clocker5;
	initClocker(200, clocker5_ISR, clocker5);

	startClocker(app_clocker);
	// ******************************************
}

void setDefaultCommSettings()
{
	msg_settings->block_len = 20;
	gf_data->index_body = 1;
	msg_settings->rec_errs = gf_data->index_body + 1;
	msg_settings->pack_len = 200;
	msg_settings->antinoise_coding = True;
	msg_settings->packlen_autoadjust = False;
	msg_settings->interleaving = False;
	msg_settings->pack_delay = 0;
}

void init_UART_MsgData(void)
{
	//int i;

	dataUnavailable = True;
	transmitterFull = True;

	gf_data = (GF_Data*) malloc(sizeof(GF_Data));
	gfdata_init(gf_data, MAX_REC_ERRS);
	gf_data->index = 1; // номер порождающего полинома, который равен максимальному количеству корректируемых в данном сообщении ошибок минус 1
	gf_data->index_hdr = 1; // номер порождающего полинома, который равен максимальному количеству корректируемых в заголовке ошибок минус 1
	gf_data->index_body = 1; // номер порождающего полинома, который равен максимальному количеству корректируемых в теле сообщения ошибок минус 1
	gf_data->index_ftr = 1; // номер порождающего полинома, который равен максимальному количеству корректируемых в концовке сообщения ошибок минус 1

	input_data_enabled = True;

	msg_settings = (MsgCommSettings*) malloc(sizeof(MsgCommSettings));
	msg_settings->block_len = 20;
	msg_settings->rec_errs = gf_data->index_body + 1;
	msg_settings->pack_len = 200;
	msg_settings->antinoise_coding = True;
	msg_settings->packlen_autoadjust = False;
	msg_settings->interleaving = False;

	int i;
	in_msg_header = (MsgHeader*) malloc(sizeof(MsgHeader));
	clearMsgHeader(in_msg_header);
	in_msg.msg_header = in_msg_header;
	/*for (i = 0; i < MAX_PACK_CNT; i++)
	 {
	 MsgPacket *pack = (MsgPacket*)malloc(sizeof(MsgPacket));
	 clearMsgPacket(pack);
	 in_msg.msg_packs[i] = pack;
	 }*/
	for (i = 0; i < MAX_PACK_CNT; i++) in_msg.msg_packs[i] = NULL;
	in_msg.pack_cnt = 0;

	out_msg_header = (MsgHeader*) malloc(sizeof(MsgHeader));
	clearMsgHeader(out_msg_header);
	out_msg.msg_header = out_msg_header;
	for (i = 0; i < MAX_PACK_CNT; i++)
	{
		MsgPacket *pack = (MsgPacket*) malloc(sizeof(MsgPacket));
		clearMsgPacket(pack);
		out_msg.msg_packs[i] = pack;
	}
	out_msg.pack_cnt = 0;

	head_q = (QUEUE8*) malloc(sizeof(QUEUE8));
	//body_q = (QUEUE8*) malloc(sizeof(QUEUE8));
	body_q = (BUFFER8*) malloc(sizeof(BUFFER8));
	QUEUE8_init(HEADER_LEN, head_q);
	//QUEUE8_init(MAX_BODY_LEN, body_q);
	BUFFER8_init(body_q);

	uart_queue = (QUEUE8*) malloc(sizeof(QUEUE8));
	QUEUE8_init(MAX_BODY_LEN, uart_queue);
}

unsigned int GP0_1_rise(void)
{
	return 0;
}

/*-----------------------------------------------------------------------------
 * 							Interrupt Functions
 *---------------------------------------------------------------------------*/
interrupt void TIMER0_12_isr(void)
{
	int i;
	for (i = 0; i < NUM_CLOCKERS; i++)
	{
		if (clockers[i]->state == CLR_STARTED)
		{
			if (clockers[i]->counts >= clockers[i]->max_val)
			{
				clockers[i]->state = CLR_FINISHED;
				clockers[i]->ptr_isr();
				clockers[i]->counts = 0;
				clockers[i]->tag++;
			}
			else clockers[i]->counts++;
		}
	}
}

interrupt void UART_isr(void)
{
	uint8_t byte;

	// Determine Prioritized Pending UART Interrupt
	uartStatus = CSL_FEXT(uartRegs->IIR, UART_IIR_INTID);

	// Set Appropriate Bool
	if (uartStatus == E_DATA_READY)
	{
		uartStatus = read_UART(uartRegs, &byte);
		if (uartStatus == E_OK)
		{
#ifdef LOOPBACK_COMM_UART
			CSL_FINS(uartRegs->THR, UART_THR_DATA, byte);
#endif
			// -------- START and STOP bytes --------
			Bool flag = False;
			if (msg_header_state == NOT_DEFINED)
			{
				if (byte == START_BYTE)
				{
					msg_header_state = STARTED;
					flag = True;
					startClocker(clocker1);
				}
			}
			else if (msg_header_state == STARTED)
			{
				int sz = QUEUE8_count(uart_queue);
				if (sz == HEADER_LEN)
				{
					if (byte == STOP_BYTE)
					{
						incom_msg_state = STARTED;
						flag = True;
					}
				}
			}
			// ------------------------------------

			if (flag == False) QUEUE8_put(byte, uart_queue);

			cnt_uart_isr++;
		}
		dataUnavailable = FALSE;
	}
	else if (uartStatus == E_TRAN_BUF_EMPTY) transmitterFull = FALSE;
}


interrupt void upp_isr(void)
{
	upp_isr_count++;
//	printf("\tUPP ISR OK!  # \t%d.", ++upp_isr_count);

	// Determine Pending Interrupt
	upp_int_status = UPP0Regs->UPISR;

	if ((upp_int_status & CSL_UPP_UPISR_EOWI_MASK) 	== (1 << CSL_UPP_UPISR_EOWI_SHIFT))
	{
//		printf("\tEOW chA");
		CSL_FINST(UPP0Regs->UPIER, UPP_UPIER_EOWI, TRUE);
		// clear int-t flag
		uppFull = TRUE;
	};

	if ((upp_int_status & CSL_UPP_UPISR_EOLI_MASK) == (1 << CSL_UPP_UPISR_EOLI_SHIFT))
	{
		//printf("\tEOL chA");
		CSL_FINST(UPP0Regs->UPIER, UPP_UPIER_EOLI, TRUE);
		// clear int-t flag
		//uppFull = TRUE;
	};
//	printf("\n");
}

interrupt void GPIO_isr(void)
{

}

interrupt void UART_Telemitric_isr(void)
{
#ifdef USE_TELEMETRIC_UART
	uint8_t byte;

	int uartStatus = read_UART(uartRegs_Telemetric, &byte);
	if (uartStatus == E_OK)
	{
		if (temperature_mode == TEMP_MODE_NOT_SPVP)
		{
			if (UART_telemetric_counter >= TELEMETRIC_UART_BUF_LEN)
			{
				telemetric_board_status = 0;
				UART_telemetric_counter = 0;
				UART_telemetric_local_counter = 0;
			}
			if (UART_telemetric_local_counter >= TELEMETRIC_DATA_LEN) UART_telemetric_local_counter = 0;

			int pos = TELEMETRIC_DATA_LEN * (UART_telemetric_pack_counter - 1) + UART_telemetric_local_counter;
			UART_telemetric_local_counter++;
			UART_telemetric_counter++;

			switch (UART_telemetric_pack_counter)
			{
			case 1: // DU board
				telemetric_board_status |= 1;
				break;
			case 2: // TU board
				telemetric_board_status |= 2;
				break;
			case 3: // PU board
				telemetric_board_status |= 4;
				break;
			}

			telemetric_data[pos] = byte;
		}
		else if (temperature_mode == TEMP_MODE_SPVP)
		{
			if (temp_request_mode == TEMP_NOT_DEFINED && byte == 1)
			{
				dummyDelay(1000);
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');

				temp_request_mode = TEMP_STARTED;
				temp_sensors = 0;
				UART_telemetric_counter = 0;
				UART_telemetric_local_counter = 0;
			}
			else if (temp_request_mode == TEMP_STARTED)
			{
				temp_sensors = byte;
				temp_request_mode = TEMP_COUNTED;
			}
			else if (temp_request_mode == TEMP_COUNTED && temp_sensors > 0)
			{
				telemetric_data[UART_telemetric_counter++] = byte;
				UART_telemetric_local_counter++;
			}

			if (UART_telemetric_local_counter == 2*temp_sensors && temp_request_mode == TEMP_COUNTED)
			{
				temp_request_mode = TEMP_READY;		// прием температур завершен!
				UART_telemetric_local_counter = 0;

				// измерение напряжений
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'v');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'f');
				CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
				voltage_request_mode = VOLT_STARTED;
				return;
			}

			if (voltage_request_mode == VOLT_STARTED && UART_telemetric_local_counter < 8)
			{
				telemetric_data[UART_telemetric_counter++] = byte;
				UART_telemetric_local_counter++;
				//printf("Byte: %d\n", byte);
			}

			if (voltage_request_mode == VOLT_STARTED && UART_telemetric_local_counter == 8)
			{
				voltage_request_mode = VOLT_READY;		// прием напряжений завершен!
				UART_telemetric_local_counter = 0;
				//printf("Bytes totally: %d\n", UART_telemetric_counter);
				telemetry_ready = TELE_READY;
			}
		}
	}
#endif
}

void app_clocker_ISR(void)
{
	//printf("\nFinished !\n");
}

void clocker1_ISR(void)
{
	incom_msg_state = TIMED_OUT;

	if (QUEUE8_count(uart_queue) > 0)
	{
		tmpb = 1;
		//requestLastMsg();
	}
}

void clocker2_ISR(void)
{
	incom_msg_state = TIMED_OUT;

	MsgHeader *hdr = out_msg.msg_header;
	hdr->msg_type = MTYPE_SHORT;
	hdr->reader = PC_MAIN;
	hdr->writer = LOGGING_TOOL;
	hdr->id = in_msg.msg_header->id;
	memset(&hdr->data[0], 0x0, SRV_DATA_LEN * sizeof(uint8_t));
	hdr->data[0] = DATA_FAILED;
	hdr->data[1] = msg_was_treated;

	int d_len = uart_queue->cnt;
	//printf("Received bytes: %d\n",d_len);

	sendShortMsg(hdr, uartRegs);

	msg_was_treated = MSG_OK;
}

void clocker3_ISR(void)
{
	if (incom_msg_state == NOT_DEFINED)
	{
		uint8_t pg = (uint8_t) proger_rd_pwr_pg();
		uint8_t tele_flag = 0;
#ifdef USE_TELEMETRIC_UART
		if (temperature_mode == TEMP_MODE_NOT_SPVP)
		{
			if (UART_telemetric_counter % TELEMETRIC_DATA_LEN == 0 && UART_telemetric_counter > 0) tele_flag = 1;
		}
		if (temperature_mode == TEMP_MODE_SPVP)
		{
			if (temp_request_mode == TEMP_READY || voltage_request_mode == VOLT_READY)
			{
				tele_flag = 1;
			}
		}
#endif
#ifdef USE_PRESSURE_UNIT
		if (press_unit_ready == PRESS_UNIT_READY) tele_flag = 1;
		//press_unit_ready = PRESS_UNIT_NOT_READY;
#endif
		uint8_t pp_is_started  = proger_is_started();
		uint8_t pp_is_seq_done = proger_is_seq_done();
		uint8_t out_mask = pg | (tele_flag << 1) | (pp_is_started << 2) | (pp_is_seq_done << 3);
		sendByteArray(NMRTool_Ready[out_mask], SRV_MSG_LEN + 2, uartRegs);
		//pp_is_seq_done = proger_is_seq_done();
	}
	startClocker(clocker3);
}

void clocker4_ISR(void)
{
#ifdef USE_TELEMETRIC_UART
	//if (incom_msg_state == NOT_DEFINED)
	{
		temp_request_mode = TEMP_NOT_DEFINED;
		voltage_request_mode = VOLT_NOT_DEFINED;
		toMeasureTemperatures();
		//telemetry_ready = TELE_READY;
		telemetry_ready = TELE_NOT_READY;
	}
#endif
#ifdef USE_PRESSURE_UNIT
	press_unit_ready = PRESS_UNIT_READY;
#endif
	startClocker(clocker4);
}

void clocker5_ISR(void)
{

}

void sendServiceMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	if (!_msg_header) return;

	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->writer & 0x0F) << 4) | (_msg_header->reader & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->data[0];
	arr_poly->data[4] = _msg_header->data[1];
	arr_poly->data[5] = _msg_header->data[2];
	arr_poly->data[6] = _msg_header->data[3];
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);

	int g_num = gf_data->index_hdr;

	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	memcpy(arr_poly->data + HEAD_INFO_LEN, rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = 0; i < HEADER_LEN; i++) write_UART(uartRegs, arr_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}

void sendShortMsg(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->reader & 0x0F) << 4) | (_msg_header->writer & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->data[0];
	arr_poly->data[4] = _msg_header->data[1];
	arr_poly->data[5] = _msg_header->data[2];
	arr_poly->data[6] = _msg_header->data[3];
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);
	arr_poly->power = HEAD_INFO_LEN - 1;

	int g_num = gf_data->index_hdr;

	GFPoly_self_inv(arr_poly);
	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	GFPoly_self_inv(rec_poly);
	memcpy(&_msg_header->rec_data[0], rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = HEAD_INFO_LEN - 1; i >= 0; i--) write_UART(uartRegs, arr_poly->data[i]);
	for (i = 0; i < HEAD_REC_LEN; i++) write_UART(uartRegs, rec_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}

void sendHeader(MsgHeader *_msg_header, CSL_UartRegsOvly uartRegs)
{
	GFPoly *rec_poly = GFPoly_alloc();
	GFPoly *arr_poly = GFPoly_alloc();
	GFPoly_init(HEADER_LEN - 1, arr_poly);

	arr_poly->data[0] = _msg_header->msg_type;
	arr_poly->data[1] = ((_msg_header->reader & 0x0F) << 4) | (_msg_header->writer & 0x0F);
	arr_poly->data[2] = _msg_header->id;
	arr_poly->data[3] = _msg_header->pack_count;
	arr_poly->data[4] = _msg_header->pack_len;
	arr_poly->data[5] = _msg_header->block_len;
	arr_poly->data[6] = _msg_header->rec_errs;
	arr_poly->data[7] = Crc8(arr_poly->data, HEAD_INFO_LEN - 1);
	arr_poly->power = HEAD_INFO_LEN - 1;

	int g_num = gf_data->index_hdr;

	GFPoly_self_inv(arr_poly);
	RS_encode(arr_poly, gf_data->gf_polys[g_num], gf_data->gf, rec_poly);
	GFPoly_self_inv(rec_poly);
	memcpy(&_msg_header->rec_data[0], rec_poly->data, HEAD_REC_LEN);

	uint8_t start_byte = START_BYTE;
	uint8_t stop_byte = STOP_BYTE;

	int i;
	write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
	for (i = HEAD_INFO_LEN - 1; i >= 0; i--) write_UART(uartRegs, arr_poly->data[i]);
	for (i = 0; i < HEAD_REC_LEN; i++) write_UART(uartRegs, rec_poly->data[i]);
	write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

	GFPoly_destroy(arr_poly);
	GFPoly_destroy(rec_poly);
	free(arr_poly);
	free(rec_poly);
}

void sendMultyPackMsg(UART_Message *uart_msg, CSL_UartRegsOvly uartRegs)
{
	MsgHeader *_msg_header = uart_msg->msg_header;

	if (_msg_header->msg_type == MTYPE_MULTYPACK)
	{
		uint8_t start_byte = START_BYTE;
		uint8_t stop_byte = STOP_BYTE;
		if (outcom_msg_state == NOT_BUILT || outcom_msg_state == MESSAGE_SENT)
		{
			sendHeader(_msg_header, uartRegs);
			outcom_msg_state = HEADER_SENT;
		}
		else if (outcom_msg_state == HEADER_SENT)
		{
			uint16_t pack_cnt = uart_msg->pack_cnt;
			int i;
			for (i = 0; i < pack_cnt; i++)
			{
				MsgPacket *_msg_packet = uart_msg->msg_packs[i];

				write_UART(uartRegs, start_byte); // отправка стартового байта перед пакетом
				sendByteArray(&_msg_packet->data[0], _msg_packet->pack_len, uartRegs);
				write_UART(uartRegs, stop_byte); // отправка стопового байта после пакета

				if (msg_settings->pack_delay > 0)
				{
					uint32_t dummy_counts = (uint32_t) (10 * msg_settings->pack_delay); // dummyDelay(10) ~= 1 ms
					dummyDelay(dummy_counts);
				}
			}
			outcom_msg_state = MESSAGE_SENT;
		}

	}
}

void sendByteArray(uint8_t *arr, uint16_t len, CSL_UartRegsOvly uartRegs)
{
	int i;
	for (i = 0; i < len; i++) write_UART(uartRegs, arr[i]);
}

void summationDataToOutput(OutBuffer *out_buff, SummationBuffer *sum_buff)
{
	if (sum_buff->max_size <= 0) return;
	if (sum_buff->pos <= 0) return;

	int outdata_count = out_buff->outdata_counter;
	uint8_t data_id = sum_buff->data_id;
	uint8_t channel_data_id = sum_buff->channel_id;
	int data_len = sum_buff->pos;
	int group_index = sum_buff->group_index;

	if (data_len + out_buff->full_size > NMR_DATA_LEN) return;

	memcpy(out_buff->out_data + out_buff->full_size, sum_buff->sum_data, data_len * sizeof(float));
	out_buff->data_id[outdata_count] = data_id;
	out_buff->channel_id[outdata_count] = channel_data_id;
	out_buff->outdata_len[outdata_count] = data_len;
	out_buff->full_size += data_len;
	out_buff->group_index[outdata_count] = group_index;
	out_buff->outdata_counter++;
}

#ifdef USE_TELEMETRIC_UART
void telemetryDataToOutput(OutBuffer *out_buff)
{
	float *dst = out_buff->out_data;
	int dst_pos = out_buff->full_size;
	int data_cnt = out_buff->outdata_counter;
	//memcpy((uint8_t*)dst + dst_pos, &telemetric_data[0], TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));

	if (temperature_mode == TEMP_MODE_NOT_SPVP)
	{
		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[0], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3; // 9 bytes occupy 3 floats (sizeof(float) = 4)
			out_buff->data_id[data_cnt] = DT_DU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3;// = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status >>= 1;

		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[TELEMETRIC_DATA_LEN], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3;
			out_buff->data_id[data_cnt] = DT_TU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3; // = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status >>= 1;

		if ((telemetric_board_status & 0x01) == 1)
		{
			memcpy((uint8_t*) (dst + dst_pos), &telemetric_data[2*TELEMETRIC_DATA_LEN], TELEMETRIC_DATA_LEN * sizeof(uint8_t));
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = 3;
			out_buff->data_id[data_cnt] = DT_PU;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += 3; // = 3*4 bytes (4 = sizeof(float))
		}
		telemetric_board_status = 0;

		out_buff->full_size += dst_pos;

		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));
	}
	else if (temperature_mode == TEMP_MODE_SPVP)
	{
		//if ((telemetric_board_status & 0x01) == 1)
		if (temp_request_mode == TEMP_READY)
		{
			uint16_t cnt = 0;
			uint16_t sensors = temp_sensors;
			uint8_t temp[TELEMETRIC_UART_BUF_LEN2+sizeof(uint16_t)];	// let's enable up to 12 temperature meters, also 2 bytes for the number of temperature meters
			memset(&temp[0], 0x00, TELEMETRIC_UART_BUF_LEN2*sizeof(uint8_t));
			memcpy(&temp[cnt], &sensors, sizeof(uint16_t));
			cnt += 1*sizeof(uint16_t);
			memcpy((uint8_t*) (&temp[cnt]), &telemetric_data[0], 2*sensors*sizeof(uint8_t));
			cnt += 2*sensors*sizeof(uint8_t);
			memcpy((uint8_t*) (dst + dst_pos), &temp[0], cnt);
			out_buff->outdata_counter++;
			out_buff->outdata_len[data_cnt] = (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float)); // the number of float values
			out_buff->data_id[data_cnt] = DT_T;
			out_buff->group_index[data_cnt++] = 0;
			dst_pos += (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
			//dst_pos += 5;// = (2*sensors(up to 8) + 4) bytes / sizeof(float)
		}
		if (voltage_request_mode == VOLT_READY)
		{
			uint16_t cnt = 0;
			uint16_t sensors = 4;			// 4 voltage meters
			uint8_t temp[TELEMETRIC_UART_BUF_LEN2+sizeof(uint16_t)]; // 8 bytes of 4 voltage sensors * sizeof(uint16_t) + the number of sensors (4 totally)
			memset(&temp[0], 0x00, (sensors*sizeof(uint16_t)+sizeof(uint16_t)));
			memcpy(&temp[cnt], &sensors, sizeof(uint16_t));
			cnt += 1*sizeof(uint16_t);
			memcpy((uint8_t*) (&temp[cnt]), &telemetric_data[2*temp_sensors], sensors*sizeof(uint16_t));
			cnt += sensors*sizeof(uint16_t);
			memcpy((uint8_t*) (dst + dst_pos), &temp[0], cnt*sizeof(uint8_t));
			out_buff->outdata_counter++;
			//out_buff->outdata_len[data_cnt] = 3; 	// 4 bytes for number '4' (int) and 8 bytes of voltage data occupy 3 elements of type 'float'
			out_buff->outdata_len[data_cnt] = (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
			out_buff->data_id[data_cnt] = DT_U;
			out_buff->group_index[data_cnt++] = 0;
			//dst_pos += 3;							// = (4 + 2*4 bytes)/sizeof(float)
			dst_pos += (uint16_t)((cnt+(sizeof(float)-1))/sizeof(float));
		}

		telemetric_board_status = 0;

		out_buff->full_size += dst_pos;

		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN2 * sizeof(uint8_t));
	}

	UART_telemetric_counter = 0;
	UART_telemetric_pack_counter = 0;
	telemetry_ready = TELE_NOT_READY;
}
#endif

#ifdef USE_PRESSURE_UNIT
void pressureUnitDataToOutput(OutBuffer *out_buff)
{
	int adc_channel = 0;
	unsigned int mtr_adc_value = proger_read_mtr_adc_value (adc_channel);
	signed int mtr_counter = proger_read_counter_mtr ();
	unsigned int mtr_status = proger_read_mtr_status ();

	/*if ((mtr_status & 0x4) >> 2 && clocker4->max_val < 10000)
	{
		clocker4->max_val = 10000;
		startClocker(clocker4);
	}*/

	float *dst = out_buff->out_data;
	int dst_pos = out_buff->full_size;
	int data_cnt = out_buff->outdata_counter;

	memcpy((uint8_t*) (dst + dst_pos), &mtr_adc_value, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	memcpy((uint8_t*) (dst + dst_pos), &mtr_counter, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	memcpy((uint8_t*) (dst + dst_pos), &mtr_status, sizeof(uint32_t));
	dst_pos += 1;// = 1*4 bytes (4 = sizeof(float))
	out_buff->outdata_counter++;
	out_buff->outdata_len[data_cnt] = 3; // 9 bytes occupy 3 floats (sizeof(float) = 4)
	out_buff->data_id[data_cnt] = DT_PRESS_UNIT;
	out_buff->group_index[data_cnt++] = 0;

	out_buff->full_size += dst_pos;

	press_unit_ready = PRESS_UNIT_NOT_READY;

	//uint32_t tmp[1024];
	//memcpy(&tmp[0], out_buff->out_data, dst_pos*sizeof(uint32_t));
	//int tt = 0;
}
#endif

void prepareOutputByteArray(OutBuffer *out_buff, SummationBuffer *sum_buff)
{
	int i;
	int index = 0;

	int outdata_count = out_buff->outdata_counter;
	int pos = 0;
	for (i = 0; i < outdata_count; i++)
	{
		uint8_t data_id = out_buff->data_id[i];
		uint16_t data_len = (uint16_t) out_buff->outdata_len[i];
		uint16_t group_index = (uint16_t) out_buff->group_index[i];
		uint8_t channel_data_id = (uint8_t)out_buff->channel_id[i];

		switch (data_id)
		{
		case DT_SGN_SE_ORG:
		case DT_NS_SE_ORG:
		case DT_SGN_FID_ORG:
		case DT_NS_FID_ORG:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) channel_data_id;
			data_fin[index++] = (uint8_t) group_index;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				int16_t min = (int16_t) out_buff->out_data[pos];
				int16_t max = min;
				for (j = 0; j < data_len; j++)
				{
					int16_t x = (int16_t) out_buff->out_data[j + pos];
					if (x < min) min = x;
					if (x > max) max = x;
				}

				float b = -min;
				float a = (max - min) / 255.0;
				if (a == 0) a = 1;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					int16_t x = (int16_t) out_buff->out_data[j + pos];
					uint8_t val = (uint8_t) ((x + b) / a);
					data_fin[j + index] = val;
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;
				/*if (outdata_count > 1 && i < outdata_count - 2)
				 {
				 data_fin[index++] = 0x53;
				 data_fin[index++] = 0x35;
				 }*/
				data_fin_counter = index;
			}

			break;
		}
		case DT_SGN_SE:
		case DT_NS_SE:
		case DT_NS_QUAD_SE_RE:
		case DT_NS_QUAD_SE_IM:
		case DT_NS_QUAD_FID_RE:
		case DT_NS_QUAD_FID_IM:
		case DT_SGN_QUAD_SE_RE:
		case DT_SGN_QUAD_SE_IM:
		case DT_SGN_QUAD_FID_RE:
		case DT_SGN_QUAD_FID_IM:
		case DT_NS_FFT_FID_RE:
		case DT_NS_FFT_SE_RE:
		case DT_SGN_FFT_FID_RE:
		case DT_SGN_FFT_SE_RE:
		case DT_NS_FFT_FID_IM:
		case DT_NS_FFT_SE_IM:
		case DT_SGN_FFT_FID_IM:
		case DT_SGN_FFT_SE_IM:
		case DT_SGN_FFT_FID_AM:
		case DT_NS_FFT_FID_AM:
		case DT_SGN_FFT_SE_AM:
		case DT_NS_FFT_SE_AM:
		case DT_SGN_POWER_SE:
		case DT_SGN_POWER_FID:
		case DT_NS_POWER_SE:
		case DT_NS_POWER_FID:
		case DT_RFP:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) channel_data_id;
			data_fin[index++] = (uint8_t) group_index;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				volatile int tt = 0;
				float min = out_buff->out_data[pos];
				float max = min;
				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					if (x < min) min = x;
					if (x > max) max = x;
				}

				float b = -min;
				float a = (max - min) / 255.0;
				if (a == 0) a = 1.0;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint8_t val = (uint8_t) ((x + b) / a);
					data_fin[j + index] = val;
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;
				/*if (outdata_count > 1 && i < outdata_count - 2)
				 {
				 data_fin[index++] = 0x53;
				 data_fin[index++] = 0x35;
				 }*/
				data_fin_counter = index;
			}
			break;
		}
		case DT_SGN_RELAX:
		case DT_SGN_RELAX2:
		case DT_SGN_RELAX3:
		case DT_AFR1_RX:
		case DT_AFR2_RX:
		case DT_AFR3_RX:
		case DT_SOLID_ECHO:
		case DT_T1T2_NMR:
		case DT_DsT2_NMR:
		case DT_FREQ_TUNE:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) channel_data_id;
			data_fin[index++] = (uint8_t) group_index;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			if (data_len > 0)
			{
				int j;
				float min = out_buff->out_data[pos];
				float max = min;
				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint32_t *b = (uint32_t*) &x;
					if (*b != 0xffffffff)
					{
						if (x < min) min = x;
						if (x > max) max = x;
					}
				}

				float b = -min;
				float a = (max - min) / 254.0;
				//if (a == 0) a = 1.0;
				memcpy(&data_fin[index], (uint8_t*) (&a), sizeof(float));
				index += sizeof(float);
				memcpy(&data_fin[index], (uint8_t*) (&b), sizeof(float));
				index += sizeof(float);

				for (j = 0; j < data_len; j++)
				{
					float x = out_buff->out_data[j + pos];
					uint32_t *bb = (uint32_t*) (&(out_buff->out_data[j + pos]));
					if (*bb == 0xffffffff) 	data_fin[j + index] = 0xff;
					else
					{
						if (min == max) data_fin[j + index] = 1;
						else
						{
							uint8_t val = (uint8_t) ((x + b) / a);
							data_fin[j + index] = val;
						}
					}
				}

				pos += data_len;
				index += data_len * sizeof(uint8_t);
				if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;
				/*if (outdata_count > 1 && i < outdata_count - 2)
				 {
				 data_fin[index++] = 0x53;
				 data_fin[index++] = 0x35;
				 }*/
				data_fin_counter = index;
			}
			break;
		}
		case DT_GAMMA:
		case DT_PRESS_UNIT:
		{
			uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(float));
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) channel_data_id;
			data_fin[index++] = (uint8_t) group_index;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			//float *gl = (float*) (out_buff->out_data + pos);
			//float bb = *gl;
			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos), data_in_bytes);
			pos += data_len;
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;
			/*if (outdata_count > 1 && i < outdata_count - 2)
			 {
			 data_fin[index++] = 0x53;
			 data_fin[index++] = 0x35;
			 }*/
			data_fin_counter = index;

			break;
		}
		case DT_DU:
		case DT_PU:
		case DT_TU:
		{
			//uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(float));
			//uint16_t data_in_bytes = (uint16_t) (data_len * sizeof(uint8_t));
			uint16_t data_in_bytes = (uint16_t) (TELEMETRIC_DATA_LEN);
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) channel_data_id;
			data_fin[index++] = (uint8_t) group_index;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos), data_in_bytes);
			pos += 3;
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1)
				data_fin[index++] = 0xFF;
			/*if (outdata_count > 1 && i < outdata_count - 2)
			 {
			 data_fin[index++] = 0x53;
			 data_fin[index++] = 0x35;
			 }*/
			data_fin_counter = index;

			break;
		}
		case DT_T:
		{
			uint8_t temp[100];
			memcpy(&temp[0], out_buff->out_data, 100);
			if (temperature_mode != TEMP_MODE_SPVP) return;
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);

			int sensors = *(uint8_t*)(out_buff->out_data+pos);
			//pos++;
			uint16_t data_in_bytes =  2*sensors;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos) + 2, data_in_bytes);
			//pos += 4;			// 8 sensors * 2 bytes / sizeof(float)
			pos += (2*sensors+2+(sizeof(float)-1))/sizeof(float);
			index += data_in_bytes;		// 8 sensors * 2 bytes
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		case DT_U:
		{
			if (temperature_mode != TEMP_MODE_SPVP) return;
			data_fin[index++] = (uint8_t) data_id;
			data_fin[index++] = (uint8_t) (group_index & 0x00FF);
			data_fin[index++] = (uint8_t) ((group_index >> 8) & 0x00FF);

			int sensors = *(uint8_t*)(out_buff->out_data+pos);
			uint16_t data_in_bytes = 2*sensors;
			//pos++;
			data_fin[index++] = (uint8_t) (data_in_bytes & 0x00FF);
			data_fin[index++] = (uint8_t) ((data_in_bytes >> 8) & 0x00FF);

			memcpy(&data_fin[index], (uint8_t*) (out_buff->out_data + pos)+2, data_in_bytes);
			pos += (data_in_bytes + 2 + (sizeof(float)-1))/sizeof(float);
			index += data_in_bytes;
			if (outdata_count > 1 && i < outdata_count - 1) data_fin[index++] = 0xFF;

			data_fin_counter = index;

			break;
		}
		default: break;
		}
	}

	SummationBuffer_ClearAll(sum_buff);
	OutBuffer_ClearAll(out_buff);
}

void generateTestEchoData(int index, int count)
{
	if (count > NMR_DATA_LEN) count = NMR_DATA_LEN;
	if (index > 1000)
		index = 1000;

	float mul = (uint16_t) (relax_ampls[index]) / 2;

	int shift = (NMR_DATA_LEN - count) / 2;
	int i;
	for (i = 0; i < count; i++)
	{
		uint16_t tab_echo = echo_ui16_60mks[i + shift] - 2048;
		uint16_t y = (uint16_t) (mul * tab_echo) + 2048;

		upp_buffer[2 * i] = (uint8_t) (y & 0x00FF);
		upp_buffer[2 * i + 1] = (uint8_t) (y >> 8);
	}
}

Bool extractDataFromPacks(UART_Message *uart_msg, uint8_t *arr, uint16_t *len)
{
	uint32_t i;
	uint32_t pos = 0;
	//uint8_t temp[2000];
	for (i = 0; i < uart_msg->pack_cnt; i++)
	{
		MsgPacket *pack = uart_msg->msg_packs[i];
		uint32_t data_len = pack->data_len;
		if (pos > MAX_BODY_LEN)
		{
			*len = 0;
			return False;
		}
		memcpy(arr + pos, &pack->data[PACK_HEAD_LEN], (data_len - PACK_SRV_LEN) * sizeof(uint8_t));
		//memcpy(&temp[pos], &pack->data[PACK_HEAD_LEN], (data_len-PACK_SRV_LEN)*sizeof(uint8_t));
		pos += data_len - PACK_SRV_LEN;
	}

	if (pos < 3)
	{
		*len = 0;
		return False;
	}

	uint32_t data_pos = 1;
	Bool fin = False;
	while (fin == False)
	{
		uint8_t byte1 = arr[data_pos];
		uint8_t byte2 = arr[data_pos + 1];
		uint16_t data_len = ((uint16_t) byte2 << 8) | (uint16_t) byte1;

		data_pos += data_len + 2; // 1 byte for separator '0xFF' and 1 byte for cmd in data array
		if (data_pos > pos || data_pos > MAX_BODY_LEN)
		{
			*len = 0;
			return False;
		}

		if (arr[data_pos] != 0xFF)
		{
			fin = True;
			*len = data_pos;
		}
		else data_pos += 2;
	}

	return True;
}

void executeProcPack(Data_Proc *proc, int index)
{
	//Data_Cmd *instr = (Data_Cmd*) malloc(sizeof(Data_Cmd));
	//init_DataProcCmd(instr);

	if (proc->proc_lens[index] <= 0) //return;					// если в пакете инструкций не было найдено инструкций, то выход
	{
		return;
	}

	while (next_DataProcCmd(index, proc, instr) == True)
	{
		switch (instr->cmd)
		{
		case INS_WR_D0_ST:	STACKPtrF_push(ptr_data_org, data_stack);	break;
		case INS_WR_D1_ST:	STACKPtrF_push(ptr_data1, data_stack);	break;
		case INS_WR_D2_ST:	STACKPtrF_push(ptr_data2, data_stack);	break;
		case INS_WR_D3_ST:	STACKPtrF_push(ptr_data3, data_stack);	break;
		case INS_WR_DX_ST:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];

			switch (indexX)
			{
			case 0:	STACKPtrF_push(ptr_data_org, data_stack);	break;
			case 1:	STACKPtrF_push(ptr_data1, data_stack);	break;
			case 2:	STACKPtrF_push(ptr_data2, data_stack);	break;
			case 3:	STACKPtrF_push(ptr_data3, data_stack);	break;
			case 4:	STACKPtrF_push(ptr_data4, data_stack);	break;
			case 5:	STACKPtrF_push(ptr_data5, data_stack);	break;
			case 6:	STACKPtrF_push(ptr_data6, data_stack);	break;
			case 7:	STACKPtrF_push(ptr_data7, data_stack);	break;
			default: break;
			}
			break;
		}
		case INS_WR_HX_ST:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			STACKPtrF_push(data_heap[indexX], data_stack);
			break;
		}
		case INS_CP_ST_D0:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data_org);	break;
		case INS_CP_ST_D1:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data1);	break;
		case INS_CP_ST_D2:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data2);	break;
		case INS_CP_ST_D3:	copy_DataTo(data_stack, DATA_MAX_LEN, ptr_data3);	break;
		case INS_CP_ST_DX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];

			float *dst = 0;
			switch (indexX)
			{
			case 0:	dst = ptr_data_org;	break;
			case 1:	dst = ptr_data1;	break;
			case 2:	dst = ptr_data2;	break;
			case 3:	dst = ptr_data3;	break;
			case 4:	dst = ptr_data4;	break;
			case 5:	dst = ptr_data5;	break;
			case 6:	dst = ptr_data6;	break;
			case 7:	dst = ptr_data7;	break;
			default: break;
			}

			copy_DataTo(data_stack, DATA_MAX_LEN, dst);
			break;
		}
		case INS_CP_ST_HX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			copy_DataTo(data_stack, DATA_MAX_LEN, data_heap[indexX]);
			break;
		}
		case INS_MV_ST_D0:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data_org);	break;
		case INS_MV_ST_D1:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data1);	break;
		case INS_MV_ST_D2:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data2);	break;
		case INS_MV_ST_D3:	move_FromStack(data_stack, DATA_MAX_LEN, ptr_data3);	break;
		case INS_MV_ST_DX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];

			float *dst = 0;
			switch (indexX)
			{
			case 0:	dst = ptr_data_org;	break;
			case 1:	dst = ptr_data1;	break;
			case 2:	dst = ptr_data2;	break;
			case 3:	dst = ptr_data3;	break;
			case 4:	dst = ptr_data4;	break;
			case 5:	dst = ptr_data5;	break;
			case 6:	dst = ptr_data6;	break;
			case 7:	dst = ptr_data7;	break;
			default: break;
			}
			move_FromStack(data_stack, DATA_MAX_LEN, dst);
			break;
		}
		case INS_MV_ST_HX:
		{
			if (instr->count < 1) return;
			int indexX = (int) instr->params[0];
			move_FromStack(data_stack, DATA_MAX_LEN, data_heap[indexX]);
			break;
		}
		case INS_ST_SWAP:		STACKPtrF_swap(data_stack);	break;
		case INS_EMUL_NS:		emulate_NoiseData(upp_buffer, processing_params);	break;
		case INS_EMUL_FID:		emulate_FIDData(upp_buffer, processing_params);	break;
		case INS_EMUL_SE:		emulate_EchoData(upp_buffer, processing_params);	break;
		case INS_EMUL_FID_NS:	emulate_FIDNoiseData(instr, upp_buffer, processing_params);	break;
		case INS_EMUL_SE_NS:	emulate_EchoNoiseData(instr, upp_buffer, processing_params);	break;
		case INS_OPER_FID_ORG:	cast_UPPDataToFID_U16(upp_buffer, processing_params->points_count, ptr_ui16_buffer);	break;
		case INS_OPER_SE_ORG:	cast_UPPDataToSE_U16(upp_buffer, processing_params->points_count, ptr_ui16_buffer);	break;
		case INS_OPER_FID:		cast_UPPDataToFID(upp_buffer, processing_params->points_count, ptr_data_org);	break;
		case INS_OPER_SE:		cast_UPPDataToSE(upp_buffer, processing_params->points_count, ptr_data_org);	break;
		case INS_OPER_FID_D:	cast_UPPDataToFID2(upp_buffer, processing_params, ptr_data_org);	break;
		case INS_OPER_SE_D:		cast_UPPDataToSE2(upp_buffer, processing_params, ptr_data_org);	break;
		case INS_DO_OPER1:		do_MathOperationVal(data_stack, DATA_MAX_LEN, summ_data, instr);	break;
		case INS_DO_OPER2:		do_MathOperationBin(data_stack, DATA_MAX_LEN, instr);	break;
		case INS_DO_XX_OPER:	do_MathOperationXX(summ_data, instr);	break;
		case INS_ADD_TO_XX:		add_ValueToXX(summ_data, instr);	break;
		case INS_ACC_DAT:		accumulate_Data(data_stack, DATA_MAX_LEN, processing_params);	break;
		case INS_SMOOTH_DAT:	accsmooth_Data(data_stack, DATA_MAX_LEN, processing_params, instr);	break;
		case INS_QD_FID:		do_QuadDetect(data_stack);	break;
		case INS_QD_SE:			do_QuadDetect(data_stack);	break;
		case INS_WIN_TIME:		setWinFuncParamsPro(instr, TIME_DOMAIN_DATA, processing_params);	break;
		case INS_WIN_FREQ:		setWinFuncParamsPro(instr, FREQ_DATA, processing_params);	break;
		case INS_APP_WIN_TIME:	apply_WinFunc(TIME_DOMAIN_DATA, processing_params, data_stack);	break;
		case INS_APP_WIN_FREQ:	apply_WinFunc(FREQ_DATA, processing_params, data_stack);	break;
		case INS_FPW:			calc_PowerSpec(data_stack, DATA_MAX_LEN);	break;
		case INS_FAMPL:			calc_AmplSpec(data_stack, DATA_MAX_LEN);	break;
		case INS_AMP1:			estimate_SignalAmp1(data_stack, DATA_MAX_LEN);	break;
		case INS_SPEC_MAX:		estimate_MaxSpectrum1(data_stack, DATA_MAX_LEN / 2, processing_params);	break;
		case INS_SUM_DAT:		summarize_Data(data_stack, DATA_MAX_LEN / 2, summ_data, instr);	break;
		case INS_SUM_REL_DAT:	summarize_DataForRelax(data_stack, DATA_MAX_LEN / 2, summ_data, processing_params, instr);	break;
		case INS_ST_AVER:		average_Data(data_stack, processing_params->points_count, summ_data, instr);	break;
		case INS_ZERO_ST:		fill_ByValue(data_stack, DATA_MAX_LEN, 0x00);	break;
		case INS_NAN_ST:		fill_ByValue(data_stack, DATA_MAX_LEN, 0xFF);	break;
		case INS_CL_ST:			STACKPtrF_clear(data_stack);	break;
		case INS_WR_XX_SUMBUF:	write_ValueToSummationBuffer(summ_data, instr);	break;
		case INS_MV_ST_OUTBUF:	move_ToOutputBuffer(data_stack, output_data, processing_params, instr->type);	break;
		case INS_ACC_TO_OUTBUF:	move_AccToOutputBuffer(data_stack, summ_data, output_data, processing_params);	break;
		case INS_XX_TO_OUTBUF:	move_XXToOutBuffer(summ_data, output_data, processing_params, instr);	break;
		case INS_ST_DEC_OUTBUF:	decimateDataInOutputbuffer(data_stack, output_data, processing_params, instr);	break;
		case INS_GET_GAMMA:
		{
			uint32_t gamma_counts = proger_rd_gamma_count();

			float *XX = 0;
			int num = instr->params[0];
			switch (num)
			{
			case X0:	XX = &summ_data->xx[0];	break;
			case X1:	XX = &summ_data->xx[1];	break;
			case X2:	XX = &summ_data->xx[2];	break;
			case X3:	XX = &summ_data->xx[3];	break;
			default: break;
			}

			if (XX != 0)
			{
				*XX = (float) gamma_counts;
			}
			break;
		}
		case INS_FFT:
		{
			if (data_stack->cnt < 2) break; 				// в стеке должны быть буффер-приемник данных и источник данных (на вершине стека)
			float *src = STACKPtrF_pop(data_stack);
			memcpy(&temp_data[0], src - PAD, UPP_DATA_SIZE);// дублирование данных src, т.к. они разрушаются при выполнении функции DSPF_sp_fftSPxSP(...)
			float *dst = STACKPtrF_first(data_stack);
			DSPF_sp_fftSPxSP(CMPLX_DATA_MAX_LEN, ptr_temp_data, ptr_w, dst, brev, rad, 0, CMPLX_DATA_MAX_LEN);
			break;
		}
		case INS_IF_DATA_NUM:
		{
			if (instr->count != 2) break;
			int data_number = (int) instr->params[0];
			int instr_count = (int) instr->params[1];
			if (processing_params->current_echo != data_number)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_WR_X0:
		{
			if (instr->count != 1) break;
			XX[0] = (float) instr->params[0];
			break;
		}
		case INS_WR_X1:
		{
			if (instr->count != 1) break;
			XX[1] = (float) instr->params[0];
			break;
		}
		case INS_WR_X2:
		{
			if (instr->count != 1) break;
			XX[2] = (float) instr->params[0];
			break;
		}
		case INS_WR_X3:
		{
			if (instr->count != 1) break;
			XX[3] = (float) instr->params[0];
			break;
		}
		case INS_WR_ACC_GRIX:
		{
			if (instr->count != 1) break;
			summ_data->group_index = (int) instr->params[0];
			summ_data->channel_id = processing_params->channel_id; //proger_rd_ch_number();
			//int data_cnt = output_data->outdata_counter;
			//output_data->group_index[data_cnt] = (int) instr->params[0];
			break;
		}
		case INS_IF_COND_X0:
		{
			if (instr->count != 3) break; 	// инструкция содержит три параметра: число, с которым сравнивается X0, код условия,
					   	   	   	   	   	   	// количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) 		res = (XX[0] == value); // condition code == 0 - "=="
			else if (cond_code == 1) 	res = (XX[0] > value); // condition code == 1 - ">"
			else if (cond_code == 2)	res = (XX[0] >= value); // condition code == 2 - ">="
			else if (cond_code == 3)	res = (XX[0] < value); // condition code == 3 - "<"
			else if (cond_code == 4)	res = (XX[0] <= value); // condition code == 4 - "<="
			else if (cond_code == 5)	res = (XX[0] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X1:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X1, код условия,
					   	   	   	   	   	   // количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0)			res = (XX[1] == value); // condition code == 0 - "=="
			else if (cond_code == 1)	res = (XX[1] > value); // condition code == 1 - ">"
			else if (cond_code == 2)	res = (XX[1] >= value); // condition code == 2 - ">="
			else if (cond_code == 3)	res = (XX[1] < value); // condition code == 3 - "<"
			else if (cond_code == 4)	res = (XX[1] <= value); // condition code == 4 - "<="
			else if (cond_code == 5)	res = (XX[1] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X2:
		{
			if (instr->count != 3) break;  // инструкция содержит три параметра: число, с которым сравнивается X2, код условия,
					   	   	   	   	   	   // количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) 		res = (XX[2] == value); // condition code == 0 - "=="
			else if (cond_code == 1)	res = (XX[2] > value); // condition code == 1 - ">"
			else if (cond_code == 2)	res = (XX[2] >= value); // condition code == 2 - ">="
			else if (cond_code == 3)	res = (XX[2] < value); // condition code == 3 - "<"
			else if (cond_code == 4)	res = (XX[2] <= value); // condition code == 4 - "<="
			else if (cond_code == 5)	res = (XX[2] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND_X3:
		{
			if (instr->count != 3) break; 		// инструкция содержит три параметра: число, с которым сравнивается X3, код условия,
					   	   	   	   	   	   	    // количество инструкций, на которое необходимо перейти в случае невыполнения условия
			float value = instr->params[0];
			int cond_code = instr->params[1];
			int instr_count = (int) instr->params[2];
			Bool res = False;
			if (cond_code == 0) 		res = (XX[3] == value); // condition code == 0 - "=="
			else if (cond_code == 1)	res = (XX[3] > value); // condition code == 1 - ">"
			else if (cond_code == 2)	res = (XX[3] >= value); // condition code == 2 - ">="
			else if (cond_code == 3)	res = (XX[3] < value); // condition code == 3 - "<"
			else if (cond_code == 4)	res = (XX[3] <= value); // condition code == 4 - "<="
			else if (cond_code == 5)	res = (XX[3] != value); // condition code == 5 - "<>"
			if (res == False)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_IF_COND:
		{
			if (instr->count != 3) break; 	// инструкция содержит три параметра: два числа, которые сравнивается, количество инструкций,
					   	   	   	   	   	    // на которое необходимо перейти в случае неравенства чисел
			float value1 = instr->params[0];
			float value2 = instr->params[1];
			int instr_count = (int) instr->params[2];
			if (value1 != value2)
			{
				pass_DataProcCmds(index, proc, instr_count);
			}
			break;
		}
		case INS_CLEAR_HX:			clearDataHeap(data_heap, &data_heap_len[0], instr);	break;
		case INS_NOISE_UPP_PRE1:	noiseUppDataPreprocessing1(upp_buffer, bank, data_heap, &data_heap_len[0], processing_params, instr); break;
		case INS_SGN_UPP_PRE1:		signalUppDataPreprocessing1(upp_buffer, bank, data_heap, processing_params, instr); break;
		case INS_NS_SGN_UPP_PRE3:	signal_noise_UppDataPreprocessing3(upp_buffer, bank, data_heap, &data_heap_len[0], processing_params, instr); break;
		case INS_NOISE_PROC1:		noiseProcessing1(upp_buffer, bank, rad, instr, processing_params, output_data); break;
		case INS_SGN_PROC1:			signalProcessing1(upp_buffer, bank, rad, instr, processing_params, summ_data, output_data); break;
		case INS_NOISE_PROC2:		noiseProcessing2(bank, data_heap, rad, instr, processing_params, output_data); break;
		case INS_SGN_PROC2:			signalProcessing2(bank, data_heap, rad, instr, processing_params, summ_data, output_data); break;
		case INS_SGN_PROC3:			signalProcessing3(bank, data_heap, rad, instr, processing_params, summ_data, output_data); break;
		case INS_GO_TO:
		{
			if (instr->count != 1) break;
			int instr_count = (int) instr->params[0];
			pass_DataProcCmds(index, proc, instr_count);
			break;
		}
		case INS_NO_OP:	break;
		default: break; // выход, если встретилась незнакомая команда
		}

		//free_DataProcCmd(instr);
	}

}

int check_stb()
{
	static volatile uint32_t pin, pin_prev;
	static volatile int first_run = true;
	volatile uint32_t i;
	volatile int ret_code;

	if (first_run == true)
	{
		pins_reg_prev = GPIO_B0_RD();
		first_run = false;
	}
	pins_reg = GPIO_B0_RD();

	for (i = 0; i < STB_PINS_COUNT; i++)
	{
		pin = (pins_reg & (1 << i)) >> i;
		pin_prev = (pins_reg_prev & (1 << i)) >> i;
		if ((pin_prev == 0) && (pin == 0)) stb[i] = STB_LOW_LVL;
		else if ((pin_prev == 1) && (pin == 1)) stb[i] = STB_HIGH_LVL;
		else if ((pin_prev == 0) && (pin == 1))	stb[i] = STB_RISING_EDGE;
		else if ((pin_prev == 1) && (pin == 0))	stb[i] = STB_FALLING_EDGE;
		else
		{
			stb[i] = STB_ERROR;
			ret_code = STB_ERROR;
		}
	}

	pins_reg_prev = pins_reg;

	pins_reg = (pins_reg >> 5) & 0x000000FF;
	pins_cmd = pins_reg;

	return (ret_code);
}

void toMeasureTemperatures()
{
	UART_telemetric_counter = 0;
	UART_telemetric_pack_counter = 0;
	telemetric_board_status = 0;

	if (temperature_mode == TEMP_MODE_NOT_SPVP)	// KMRK, NMKT и другие кроме SPVP
	{
		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN * sizeof(uint8_t));

		// DU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		//proger_restart_time_counter();
		dummyDelay(100);
		//volatile int tt = proger_read_time_counter();

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// TU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// PU Board
		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 1
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '1');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);

		// Temperature channel 2
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '2');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(100);
	}
	else if (temperature_mode == TEMP_MODE_SPVP)	// SPVP only
	{
		memset(&telemetric_data[0], 0x0, TELEMETRIC_UART_BUF_LEN2 * sizeof(uint8_t));

		// Temperature channel 0
		UART_telemetric_pack_counter++;
		UART_telemetric_local_counter = 0;
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 't');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'c');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, '0');
		CSL_FINS(uartRegs_Telemetric->THR, UART_THR_DATA, 'n');
		dummyDelay(1000);
	}
}

void initDeviceSettings(uint8_t device)
{
	switch (device)
	{
	case 4:		// KMRK
	{
		device_channel_count = 8;
		device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

		device_channels[0].type = NMR_CHANNEL;
		device_channels[0].channel_id = 0;
		device_channels[0].freq_set_num = 0;
		device_channels[0].frq1 = 0;
		device_channels[0].frq2 = 0;
		device_channels[0].frq3 = 0;
		device_channels[0].frq4 = 0;
		device_channels[0].frq5 = 0;
		device_channels[0].frq6 = 0;
		device_channels[0].addr_rx = 0;
		device_channels[0].addr_tx = 0;

		device_channels[1].type = NMR_CHANNEL;
		device_channels[1].channel_id = 1;
		device_channels[1].freq_set_num = 1;
		device_channels[1].frq1 = 0;
		device_channels[1].frq2 = 0;
		device_channels[1].frq3 = 0;
		device_channels[1].frq4 = 0;
		device_channels[1].frq5 = 0;
		device_channels[1].frq6 = 0;
		device_channels[1].addr_rx = 0;
		device_channels[1].addr_tx = 0;

		device_channels[2].type = GK_CHANNEL;
		device_channels[2].freq_set_num = 0;
		device_channels[2].channel_id = 2;

		device_channels[3].type = SDSP_CHANNEL;
		device_channels[3].freq_set_num = 0;
		device_channels[3].channel_id = 3;

		device_channels[4].type = AFR_CHANNEL;
		device_channels[4].freq_set_num = 1;
		device_channels[4].channel_id = 4;

		device_channels[5].type = AFR_CHANNEL;
		device_channels[5].freq_set_num = 1;
		device_channels[5].channel_id = 5;

		device_channels[6].type = RF_PULSE_CHANNEL;
		device_channels[6].freq_set_num = 1;
		device_channels[6].channel_id = 6;

		device_channels[7].type = RF_PULSE_CHANNEL;
		device_channels[7].freq_set_num = 1;
		device_channels[7].channel_id = 7;

		break;
	}
	default:
	{
		device_channel_count = 8;
		device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

		device_channels[0].type = NMR_CHANNEL;
		device_channels[0].freq_set_num = 0;
		device_channels[0].channel_id = 0;
		device_channels[0].frq1 = 0;
		device_channels[0].frq2 = 0;
		device_channels[0].frq3 = 0;
		device_channels[0].frq4 = 0;
		device_channels[0].frq5 = 0;
		device_channels[0].frq6 = 0;
		device_channels[0].addr_rx = 0;
		device_channels[0].addr_tx = 0;

		device_channels[1].type = NMR_CHANNEL;
		device_channels[1].freq_set_num = 1;
		device_channels[1].channel_id = 1;
		device_channels[1].frq1 = 0;
		device_channels[1].frq2 = 0;
		device_channels[1].frq3 = 0;
		device_channels[1].frq4 = 0;
		device_channels[1].frq5 = 0;
		device_channels[1].frq6 = 0;
		device_channels[1].addr_rx = 1;
		device_channels[1].addr_tx = 1;

		device_channels[2].type = GK_CHANNEL;
		device_channels[2].freq_set_num = 0;
		device_channels[2].channel_id = 2;

		device_channels[3].type = SDSP_CHANNEL;
		device_channels[3].freq_set_num = 0;
		device_channels[3].channel_id = 3;

		device_channels[4].type = AFR_CHANNEL;
		device_channels[4].freq_set_num = 1;
		device_channels[4].channel_id = 4;
		device_channels[4].addr_rx = 0;
		device_channels[4].addr_tx = 2;

		device_channels[5].type = AFR_CHANNEL;
		device_channels[5].freq_set_num = 1;
		device_channels[5].channel_id = 5;
		device_channels[5].addr_rx = 1;
		device_channels[5].addr_tx = 2;

		device_channels[6].type = RF_PULSE_CHANNEL;
		device_channels[6].freq_set_num = 1;
		device_channels[6].channel_id = 6;
		device_channels[6].addr_rx = 2;
		device_channels[6].addr_tx = 0;

		device_channels[7].type = RF_PULSE_CHANNEL;
		device_channels[7].freq_set_num = 1;
		device_channels[7].channel_id = 7;
		device_channels[7].addr_rx = 2;
		device_channels[7].addr_tx = 1;

		break;
	}
	}
}


Bool loadDeviceSettings(int *data, int len)
{
	int struct_size = sizeof(ToolChannel);
	int pos = 0;
	int channels_number = len*sizeof(uint32_t)/struct_size;

	if (channels_number > 0) free(device_channels);
	else return False;

	device_channel_count = channels_number;
	device_channels = (ToolChannel*)calloc(device_channel_count, sizeof(ToolChannel));

	int index = 0;
	while (index < channels_number)
	{
		uint32_t _data_type = data[pos++];
		uint32_t _channel_id = data[pos++];
		uint32_t _freq_set_num = data[pos++];
		uint32_t _addr_rx = data[pos++];
		uint32_t _addr_tx = data[pos++];
		uint32_t _freq1 = data[pos++];
		uint32_t _freq2 = data[pos++];
		uint32_t _freq3 = data[pos++];
		uint32_t _freq4 = data[pos++];
		uint32_t _freq5 = data[pos++];
		uint32_t _freq6 = data[pos++];
		device_channels[index].type = _data_type;
		device_channels[index].channel_id = _channel_id;
		device_channels[index].freq_set_num = _freq_set_num;
		device_channels[index].addr_rx = _addr_rx;
		device_channels[index].addr_tx = _addr_tx;
		device_channels[index].frq1 = _freq1;
		device_channels[index].frq2 = _freq2;
		device_channels[index].frq3 = _freq3;
		device_channels[index].frq4 = _freq4;
		device_channels[index].frq5 = _freq5;
		device_channels[index].frq6 = _freq6;
		index++;
	}

	return True;
}
