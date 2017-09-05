/*
===============================================================================
 Name        : test0.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "chip.h"
#include "board.h"
#include <uart.h>

// data buffer for UART interrupt handler
#define UART_BUFSIZE_RX 32
uint8_t uartBuffRx[UART_BUFSIZE_RX];
#define UART_BUFSIZE_TX 256
uint8_t uartBuffTx[UART_BUFSIZE_TX];

/* ADC Channel (default = ADC0(P0_11)) */
#define DEFAULT_ADC_CH ADC_CH0
int adc_channel = DEFAULT_ADC_CH;

/* ADC Setup Info */
static ADC_CLOCK_SETUP_T ADCSetup;

/* ADC Sampling Rate */
int adc_clock = 96000; //96ksample/sec

/* ADC Data Buffer */
#define NFRAME 480 // frame size
uint16_t adc_buff[NFRAME];

/* Data transfer mode */
#define XFER_BURST 0
#define XFER_CONT 1
int transfer_mode = XFER_BURST;
int burst_length = 500;
int slice_count = 0;
int frame_count = 0;

/* Trigger Setting */
#define TRIGGER_FREE 0
#define TRIGGER_POSITIVE 1
#define TRIGGER_NEGATIVE 2
int trigger_mode = TRIGGER_FREE;

uint16_t trigger_level = 512; //Level (0-1023, 512=center)

/* running state */
#define STATE_STOP 0
#define STATE_RUN 1
#define STATE_SINGLE 2
int state = STATE_RUN;

/* SysTick Flag (set 1 by SysTick Handler) */
static volatile int tic_f = 0;
//static int tic_count = 0;

/* SysTick Handler (Frame synchronize) */
void SysTick_Handler(void) {
		tic_f = 1;
}


/* ADC functions */
void adc_init() {
#if defined CHIP_LPC11UXX
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, FUNC2); //ADC0
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, FUNC2); //ADC1
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, FUNC2); //ADC2
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, FUNC2); //ADC3
#else
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_11, FUNC2);
#endif
	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	ADCSetup.burstMode = true;
	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, adc_clock);
	Chip_ADC_EnableChannel(LPC_ADC, adc_channel, ENABLE);
	Chip_ADC_SetStartMode(LPC_ADC, ADC_NO_START, ADC_TRIGGERMODE_RISING);
}


/*
 * Start Burst Sampling
 */
void adc_start() {
	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, adc_clock);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
}

/*
 * Stop Burst Sampling
 */
void adc_stop() {
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);
}

/*
 * Trigger process
 */
#define WAIT_TRIGGER_LIMIT 10000
#define WAIT_TRIGGER_OK 1
#define WAIT_TRIGGER_ERR 0
int wait_trigger() {
	int count = 0;

	if(trigger_mode == TRIGGER_FREE) return WAIT_TRIGGER_OK; //Wait OK

	adc_buff[0] = adc_buff[1] = trigger_level;
	while(count < WAIT_TRIGGER_LIMIT) {
		adc_buff[0] = adc_buff[1];
		while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}
		Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, adc_buff+1);
		if(trigger_mode == TRIGGER_POSITIVE) {
			if((adc_buff[0] > trigger_level)&&(adc_buff[1] <= trigger_level)) break;
		}
		else if(trigger_mode == TRIGGER_NEGATIVE){
			if((adc_buff[0] < trigger_level)&&(adc_buff[1] >= trigger_level)) break;
		}
		count++;
	}
	if(count < WAIT_TRIGGER_LIMIT) {
		return WAIT_TRIGGER_OK; //Wait OK
	}
	else {
		return WAIT_TRIGGER_ERR; //Error
	}
}

/*
 * Read ADC data to buffer (NFRAME byte)
 */
void read_block() {
	int i;

	for(i = 0; i < burst_length; i++) {
		while (Chip_ADC_ReadStatus(LPC_ADC, adc_channel, ADC_DR_DONE_STAT) != SET) {}
		Chip_ADC_ReadValue(LPC_ADC, adc_channel, adc_buff+i);
	}
}

/*
 * Read single ADC data
 */
uint16_t read_sample() {
	uint16_t raw_sample = 0;

	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, adc_clock);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
	while (Chip_ADC_ReadStatus(LPC_ADC, adc_channel, ADC_DR_DONE_STAT) != SET) {}
	Chip_ADC_ReadValue(LPC_ADC, adc_channel, &raw_sample);
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	return (raw_sample);
}

void wait1ms() {
	volatile int c;

	// software loop
	for(c = 0; c < 16000; c++) {}

	//short wait after buffer empty
	//while((LPC_USART->LSR & UART_LSR_TEMT)==0) {}
	//for(c = 0; c < 9000; c++) {}
}

void send_block() {
	int i;
	uint8_t data;

	if(transfer_mode == XFER_CONT) return;

	for(i = 0; i < burst_length; i++) {
		/* Upper 5bit (bit5 = 0) */
		data = (adc_buff[i]>>5)&0x1f;
		uart_putc(data);
		/* Lower 5bit (bit5 = 1) */
		data = (adc_buff[i]&0x1f);
		if(i == burst_length-1) {
			data |= 0x40; //Add FlagSet(Burst-EOR)
		}
		else {
			data |= 0x20; //Add FlagSet(Burst-Low)
		}
		uart_putc(data);
		if((i < burst_length-50)&&((i%50) == 49)) wait1ms(); // interval for every 100Bytes (workaround/RN42-bug)
	}
}

/*
 * Send single data (in Continuous-Mode)
 */
void send_sample(uint16_t sample, int eor) {
	int data;

	/* Upper 5bit (flagset = 000) */
	data = (sample>>5) & 0x1f;
	uart_putc(data);

	/* Lower 5bit */
	data = sample & 0x1f;
	if(eor == 1) {
		data |= 0xa0; //Add-FlagSet(Continuous-EOR)
	}
	else {
		data |= 0x60; //Add FlagSet(Continuous-Low)
	}
	uart_putc(data);
}

/*
 * Send force discard command  to Host
 */
void send_discard() {
	uart_putc(0xc0);
}

/* Command analysis from Host */
/*
 * Switch to continuous mode
 */
void set_continuous_mode( char *buff) {
	int rate;

	switch(*buff) {
	case '0':
		rate = 24;//24Hz=2sec/div
		break;
	case '1':
		rate = 48;//48Hz=1sec/div
		break;
	case '2':
		rate = 96;//96Hz=0.5sec/div
		break;
	case '3':
		rate = 480;//*not use*
		break;
	}
	adc_clock = 96000; //sampling rate for trigger checking
	transfer_mode = XFER_CONT;
	burst_length = 1;
	frame_count = 0;
	SysTick_Config(SystemCoreClock/rate);
}

/*
 * Switch to burst mode
 */
void set_burst_mode( char *buff ) {
	int rate;

	switch(*buff) {
	case '0':
		rate = 9600;//9.6kHz=5msec/div
		break;
	case '1':
		rate = 48000;//48kHz=1msec/div
		break;
	case '2':
		rate = 96000;//96kHz=0.5msec/div
		break;
	case '3':
		rate = 192000;//not use (DMA is necessary)
		break;
	}
	adc_clock = rate;
	transfer_mode = XFER_BURST;
	burst_length = NFRAME;
	frame_count = 0;
	SysTick_Config(SystemCoreClock/8);
}

/*
 *  Interpret command from Host
 */
void set_trigger_mode(char *buff) {
	switch(*buff) {
	case 'P':
		trigger_mode = TRIGGER_POSITIVE;
		break;
	case 'N':
		trigger_mode = TRIGGER_NEGATIVE;
		break;
	case 'F':
		trigger_mode = TRIGGER_FREE;
		break;
	}
}

void set_run_state(char *buff) {
	switch(*buff) {
	case '0':
		state = STATE_STOP;
		break;
	case '1':
		state = STATE_RUN;
		frame_count = 0;
		break;
	case '2':
		state = STATE_SINGLE;
		frame_count = 0;
		send_discard();
		break;
	}
}

void set_gpio_state(char *buff) {
	switch(*buff) {
	case '0':
		Chip_GPIO_SetPinState(LPC_GPIO, 0, 7, false);
		break;
	case '1':
		Chip_GPIO_SetPinState(LPC_GPIO, 0, 7, true);
		break;
	}
}

void set_adc_input(char *buff) {
	int old_channel;

	old_channel = adc_channel;
	if((*buff >= '0')&&(*buff <= '3')) {
		adc_channel = *buff-'0';
	}
	// Change ADC channel
	Chip_ADC_EnableChannel(LPC_ADC, old_channel, DISABLE);
	Chip_ADC_EnableChannel(LPC_ADC, adc_channel, ENABLE);
}

void exec_cmd(char *buff) {
	switch(*buff) {
	case 'C':
		set_continuous_mode(buff+1);
		send_discard();
		break;
	case 'B':
		set_burst_mode(buff+1);
		send_discard();
		break;
	case 'R':
		set_run_state(buff+1);
		break;
	case 'T':
		set_trigger_mode(buff+1);
		break;
	case 'G':
		set_gpio_state(buff+1);
		break;
	case 'A':
		set_adc_input(buff+1);
		break;
	}
}


#define CMD_LEN 16
static char cmd_buff[CMD_LEN]; //command buffer
static char *cmd_bufp;

/* MAIN */
int main(void) {
	int trig; //trigger result
	uint16_t sample_data;
	int d; //byte data from host

	/* System Init */
	SystemCoreClockUpdate();
#ifndef NO_BOARD_LIB
	Board_Init();
#endif
	adc_init();
	uart_init(0, 4, -1, -1, 115200, uartBuffRx, UART_BUFSIZE_RX, uartBuffTx, UART_BUFSIZE_TX);
	set_burst_mode("1\n"); //default="B1\n"=5msec/divß

	/* SysTick Timer for block interval (1/8fps = 125msec) */
	SysTick_Config(SystemCoreClock/8);

	/* GPIO init (P0_7 for output) */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 7);

	/* Main Loop */
	while (1) {
		cmd_bufp = cmd_buff;

		/* Check command from UART */
		while((d = uart_getc()) >= 0) {
			if(d == '\n') {
				exec_cmd(cmd_buff);
				cmd_bufp = cmd_buff;
			}
			else {
				*cmd_bufp = (char)d;
				cmd_bufp++;
				if(cmd_bufp-cmd_buff > CMD_LEN) {
					cmd_bufp = cmd_buff;
				}
			}
		}

		/* wait SysTick */
		while(tic_f == 0) {};
		tic_f = 0;

		if(transfer_mode == XFER_BURST) {
			/* Burst-mode */
			adc_start();
			trig = wait_trigger();
			read_block();
			adc_stop();
			if(trig == WAIT_TRIGGER_OK) {
				if(state != STATE_STOP) {
					send_block();
				}
				if(state == STATE_SINGLE) {
					state = STATE_STOP;
				}
			}
		}
		else { // Continuous mode

			/* Continuous mode */
			//adc_start();
			if(frame_count == 0) wait_trigger();
			sample_data = read_sample();
			//adc_stop();
			if(state != STATE_STOP) {
				if(frame_count >= NFRAME) {
					send_sample(sample_data, 1);
					frame_count = 0;
					if(state == STATE_SINGLE) {
						state = STATE_STOP;
					}
				}
				else {
					send_sample(sample_data, 0);
					frame_count++;
				}
			}
		}
	}
	return 1;
}
