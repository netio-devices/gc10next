/*
 * Copyright (c) 2023 Kanno System Labo
 *
 * Released under the MIT license.
 * see https://opensource.org/licenses/MIT
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "class/cdc/cdc_device.h"
#include "ssd1306.h"
#include "eeprom.h"
#include "image.h"

#define FW_VERSION "FWNX1R02"
#define MODEMAX 2
#define TOTAL_NUM 300

const uint LED_PIN = 25;
const float conversion_factor = 3.3f / ( 1 << 12) * 3;

static int tc = 0;
uint32_t total_cnt;
uint32_t ch1_cnt;
uint32_t ch2_cnt;

static uint hvgPwmSlice;
static uint bzsPwmSlice;

uint16_t result;

static bool df = false; // Detection Flag

bool is_sound_enabled = false;
bool is_button_pressed = false;
bool sout = true;

bool pd = false;

uint8_t cptr;
uint8_t dispMode;

uint16_t gamma_sensitivity;
uint16_t alarm_trigger_cpm;
uint16_t hvg_pulsewidth;
char CmdBuf[16];

uint32_t uptime;
uint32_t cpm;
uint32_t sma[4];
uint16_t cbuf[20];
uint16_t cpx;
uint16_t lp;	// Long button press detection counter

uint16_t cnt = 0;
uint16_t fifocpm[TOTAL_NUM];
uint32_t sum = 0;
float avrg = 0;
float max_avrg = 0.00f;
float min_avrg = 1000.00f;

uint16_t tone;

uint8_t cbuf_idx;
uint8_t sec_tmr;
uint8_t sma_idx;

uint8_t gbx = 0;	// graph plotting queue index
uint16_t gpq[96];	// queue for graph plotting

int16_t cmax;
int16_t cmin;

ssd1306_t disp;

void SerCmdProc(char ch);
void SerCmdExec(void);
void dispCPM(void);
void prepareDisp(void);

bool __no_inline_not_in_flash_func(get_bootsel_button)() {
	const uint CS_PIN_INDEX = 1;
	uint32_t flags = save_and_disable_interrupts();

	hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
			GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
			IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

	for (volatile int i = 0; i < 1000; ++i);

	bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

	hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
			GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
			IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

	restore_interrupts(flags);

	return button_state;
}

void gpio_callback(uint gpio, uint32_t events) {
	if (events & 0x04) {
		gpio_put(LED_PIN, 1);
		if (is_sound_enabled) {
			pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 1000/3);
		}

		df = true;
		cpx++;
		total_cnt++;

		if (gpio == 22) {
			ch1_cnt++;
		} else
		if (gpio == 2) {
			ch2_cnt++;
		}
	}
}

void on_uart_rx() {
	while (uart_is_readable(uart1)) {
		SerCmdProc(uart_getc(uart1));
	}
}

void SerCmdProc(char ch)
{
	switch(ch) {
		case '\b':
			if (cptr > 0) {
				cptr--;
				printf("\b \b");
			}
			break;
		case '\r':
			if (cptr < 16) {
				CmdBuf[cptr] = 0x00;
				printf("\r\n");
				SerCmdExec();
			}
			cptr = 0;
			break;
		case '\n':
			break;
		default:
			printf("%c", ch);
			if (cptr < (16 - 1)) {
				CmdBuf[cptr++] = ch;
			} else {
				cptr = 0;
				printf("?\r\n");
			}
			break;
	}
}

bool msecTimer_callback(repeating_timer_t *rt) {
	if (df == true) {
		tc++;
		if (tc > 20) {
			df = false;
			tc = 0;
			gpio_put(LED_PIN, 0);
			pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 0);
		}
	}

	if (pd == true) {
		prepareDisp();
		pd = false;
	}
}

bool secTimer_callback(repeating_timer_t *rt) {
	uint8_t i;

	sec_tmr++;
	uptime++;	

	if (sec_tmr % 3 == 0) {
		cbuf[cbuf_idx++] = cpx;
		if (cbuf_idx == 20) cbuf_idx = 0;

		cpx = 0;
		cpm = 0;
		for (i = 0; i < 20; i++) cpm += cbuf[i];
	}

	if (cnt == TOTAL_NUM) cnt = 0;

	sum -= fifocpm[cnt];
	fifocpm[cnt] = cpm;
	sum += fifocpm[cnt];
	cnt++;

	avrg = (float)sum / TOTAL_NUM;

	if (uptime > (300 + 60)) {
		if (avrg > max_avrg)
			max_avrg = avrg;

		if (avrg < min_avrg)
			min_avrg = avrg;
	}

	if (sec_tmr == 60) {
		sec_tmr = 0;
	}

	dispCPM();
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void prepareDisp(void) {
	char buf[12];

	if (dispMode == 0) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'R');
		ssd1306_draw_char(&disp, 0, 8, 1, 'T');

		if (uptime < 60) {
			sprintf(buf, "[NV]");
			ssd1306_draw_string(&disp, 8, 4, 1, buf);
		}

		sprintf(buf, "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		int k;
		for (k = 0; k < 98; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 35);
				ssd1306_draw_pixel(&disp, k, 63);
			}
		}
		for (k = 35; k < 64; k++) {
			if (k % 2 == 0) {
				ssd1306_draw_pixel(&disp, 0, k);
				ssd1306_draw_pixel(&disp, 100, k);
			}
		}

		ssd1306_draw_line(&disp, 98, 35, 102, 35);
		ssd1306_draw_line(&disp, 98, 63, 102, 63);

		ssd1306_show(&disp);
	} else
	if (dispMode == 1) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'M');
		ssd1306_draw_char(&disp, 0, 8, 1, 'A');

		if (uptime < 300 + 60) {
			sprintf(buf, "[NV]");
			ssd1306_draw_string(&disp, 8, 4, 1, buf);
		}

		sprintf(buf,  "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		int k;
		for (k = 0; k < 128; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 38);
			}
		}
		for (k = 38; k < 64; k++) {
			if (k % 3 == 0) {
				ssd1306_draw_pixel(&disp, 92, k);
			}
		}

		ssd1306_draw_string(&disp, 0, 44, 1, "MAX:");
		ssd1306_draw_string(&disp, 0, 56, 1, "MIN:");

		sprintf(buf, "N=");
		ssd1306_draw_string(&disp, 98, 44, 1, buf);

		sprintf(buf, "300");
		ssd1306_draw_string(&disp, 98, 56, 1, buf);

		ssd1306_show(&disp);
	} else
	if (dispMode == 2) {
		ssd1306_clear(&disp);

		sprintf(buf,  "FWV:%s", FW_VERSION);
		ssd1306_draw_string(&disp, 0, 0, 1, buf);

		sprintf(buf,  "TCN:%lu", total_cnt);
		ssd1306_draw_string(&disp, 0, 9, 1, buf);

		sprintf(buf,  "CH1:%lu", ch1_cnt);
		ssd1306_draw_string(&disp, 0, 18, 1, buf);

		sprintf(buf,  "CH2:%lu", ch2_cnt);
		ssd1306_draw_string(&disp, 0, 27, 1, buf);

		sprintf(buf,  "OTS:%lu", uptime);
		ssd1306_draw_string(&disp, 0, 36, 1, buf);

		sprintf(buf, "VCC:%f", result * conversion_factor);
		ssd1306_draw_string(&disp, 0, 45, 1, buf);

		ssd1306_show(&disp);
	}

}

void dispCPM(void) {
	uint8_t i, j;
	uint32_t tmp;
	char buf[12];

	sma[sma_idx++] = cpm;
	if (sma_idx == 4) sma_idx = 0;

	tmp  = 0;
	for (i = 0; i < 4; i++) tmp += sma[i];
	tmp = ((tmp << 1) / 4 + 1) >> 1;

	if (dispMode == 0) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'R');
		ssd1306_draw_char(&disp, 0, 8, 1, 'T');

		if (uptime < 60) {
			sprintf(buf, "[NV]");
			ssd1306_draw_string(&disp, 8, 4, 1, buf);
		}

		sprintf(buf, "%lu", tmp);
		ssd1306_draw_string(&disp, 96 - strlen(buf) * 16, 0, 2, buf);

		sprintf(buf, "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "%.4f", (float)tmp / gamma_sensitivity);
		if(strlen(buf) > 6) {
			sprintf(buf, "%.3f", (float)tmp / gamma_sensitivity);
		}
		ssd1306_draw_string(&disp, 0, 18, 2, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		cmax = 0;
		cmin = INT16_MAX;

		gpq[gbx] = tmp;

		for (i = 0; i<96; i++) {
			if (gpq[i] > cmax) {
				cmax = gpq[i];
			}
		}

		cmax = cmax + 8;
		cmax = ((cmax / 10) + 1) * 10;
		cmin = 0;

		int y;

		j = 0;
		if(gbx != 95)  {
			for (i = gbx + 1; i<96; i++) {
				if (gpq[i]) {
					y = map(gpq[i], cmin, cmax, 63, 34);
					ssd1306_draw_pixel(&disp, j++, y);
				}
			}
			for (i = 0; i <= gbx; i++) {
				if (gpq[i]) {
					y = map(gpq[i], cmin, cmax, 63, 34);
					ssd1306_draw_pixel(&disp, j++, y);
				}
			}
		} else {
			for (i = 0; i<96; i++) {
				if (gpq[i]) {
					y = map(gpq[i], cmin, cmax, 63, 34);
					ssd1306_draw_pixel(&disp, j++, y);
				}
			}
		}

		gbx++;
		if (gbx==96) {
			gbx = 0;
		}

		int k;
		for (k = 0; k < 98; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 35);
				ssd1306_draw_pixel(&disp, k, 63);
			}
		}
		for (k = 35; k < 64; k++) {
			if (k % 2 == 0) {
				ssd1306_draw_pixel(&disp, 0, k);
				ssd1306_draw_pixel(&disp, 100, k);
			}
		}

		ssd1306_draw_line(&disp, 98, 35, 102, 35);
		ssd1306_draw_line(&disp, 98, 63, 102, 63);

		sprintf(buf,  "%lu", cmax);
		ssd1306_draw_string(&disp, 104, 36, 1, buf);

		sprintf(buf,  "%lu", cmin);
		ssd1306_draw_string(&disp, 104, 56, 1, buf);

		ssd1306_show(&disp);
	} else
	if (dispMode == 1) {
		ssd1306_clear(&disp);

		ssd1306_draw_char(&disp, 0, 0, 1, 'M');
		ssd1306_draw_char(&disp, 0, 8, 1, 'A');

		if (uptime < 300 + 60) {
			sprintf(buf, "[NV]");
			ssd1306_draw_string(&disp, 8, 4, 1, buf);
		}

		sprintf(buf,  "%.2f", avrg);
		ssd1306_draw_string(&disp, 96 - 24, 7, 1, buf + strlen(buf) - 3);
		buf[strlen(buf) - 3] = 0;
		ssd1306_draw_string(&disp, 96 - strlen(buf) * 16 - 24 + 4, 0, 2, buf);

		sprintf(buf,  "CPM");
		ssd1306_draw_string(&disp, 98, 4, 1, buf);

		sprintf(buf, "%.4f", avrg / gamma_sensitivity);
		if(strlen(buf) > 6) {
			sprintf(buf, "%.3f", (float)avrg / gamma_sensitivity);
		}
		ssd1306_draw_string(&disp, 0, 18, 2, buf);

		sprintf(buf, "uSv");
		ssd1306_draw_string(&disp, 98, 16, 1, buf);

		sprintf(buf, "/h");
		ssd1306_draw_string(&disp, 102, 24, 1, buf);

		int k;
		for (k = 0; k < 128; k++) {
			if (k % 4 == 0) {
				ssd1306_draw_pixel(&disp, k, 38);
			}
		}
		for (k = 38; k < 64; k++) {
			if (k % 3 == 0) {
				ssd1306_draw_pixel(&disp, 92, k);
			}
		}

		if (uptime < 300 + 60) {
			sprintf(buf, "MAX: UD");
		} else {
			sprintf(buf, "MAX:%.4f", max_avrg / gamma_sensitivity);
		}
		ssd1306_draw_string(&disp, 0, 44, 1, buf);

		if (uptime < 300 + 60) {
			sprintf(buf, "MIN: UD");
		} else {
			sprintf(buf, "MIN:%.4f", min_avrg / gamma_sensitivity);
		}
		ssd1306_draw_string(&disp, 0, 56, 1, buf);

		sprintf(buf, "N=");
		ssd1306_draw_string(&disp, 98, 44, 1, buf);

		sprintf(buf, "300");
		ssd1306_draw_string(&disp, 98, 56, 1, buf);

		ssd1306_show(&disp);
	} else
	if (dispMode == 2) {
		ssd1306_clear(&disp);

		sprintf(buf,  "FWV:%s", FW_VERSION);
		ssd1306_draw_string(&disp, 0, 0, 1, buf);

		sprintf(buf,  "TCN:%lu", total_cnt);
		ssd1306_draw_string(&disp, 0, 9, 1, buf);

		sprintf(buf,  "CH1:%lu", ch1_cnt);
		ssd1306_draw_string(&disp, 0, 18, 1, buf);

		sprintf(buf,  "CH2:%lu", ch2_cnt);
		ssd1306_draw_string(&disp, 0, 27, 1, buf);

		sprintf(buf,  "OTS:%lu", uptime);
		ssd1306_draw_string(&disp, 0, 36, 1, buf);

		result = adc_read();

		sprintf(buf, "VCC:%f", result * conversion_factor);
		ssd1306_draw_string(&disp, 0, 45, 1, buf);

		ssd1306_show(&disp);
	}

	// CPM output
	if (sout) {
		printf("%d\n", tmp);
	}
}

void software_reset() {
	watchdog_enable(1, 1);
	while(1);
}

void eeprom_write_byte (uint16_t addr, uint8_t val) {
	uint8_t src[1 + 1];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val;

	switch(i2c_write_blocking(i2c_default, 0x50, src, 2, false)) {
		case PICO_ERROR_GENERIC:
			printf("[%s] addr not acknowledged!\n");
			 break;
		case PICO_ERROR_TIMEOUT:
			printf("[%s] timeout!\n");
			break;
		default:
			break;
	}
	restore_interrupts(flags);

	sleep_ms(5);
}

void eeprom_write_word (uint16_t addr, uint16_t val) {
	uint8_t src[1 + 2];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val  & 0xFF;
	src[2] = val >> 8;

	switch(i2c_write_blocking(i2c_default, 0x50, src, 3, false)) {
		case PICO_ERROR_GENERIC:
			printf("[%s] addr not acknowledged!\n");
			 break;
		case PICO_ERROR_TIMEOUT:
			printf("[%s] timeout!\n");
			break;
		default:
			break;
	}
	restore_interrupts(flags);

	sleep_ms(5);
}

void eeprom_write_long (uint16_t addr, uint32_t val) {
	uint8_t src[1 + 4];
	uint32_t flags = save_and_disable_interrupts();

	src[0] = addr & 0xFF;
	src[1] = val  & 0xFF;
	src[2] = (val >>  8) & 0xFF;
	src[3] = (val >> 16) & 0xFF;
	src[4] = (val >> 24) & 0xFF;

	switch(i2c_write_blocking(i2c_default, 0x50, src, 5, false)) {
		case PICO_ERROR_GENERIC:
			printf("[%s] addr not acknowledged!\n");
			 break;
		case PICO_ERROR_TIMEOUT:
			printf("[%s] timeout!\n");
			break;
		default:
			break;
	}
	restore_interrupts(flags);

	sleep_ms(5);
}

int eeprom_read_byte(int addr, uint8_t *p)
{
	uint8_t src;
    uint8_t rx;
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_blocking(i2c_default, 0x50, &src, 1, true);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_blocking(i2c_default, 0x50, &rx, 1, false);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

	*p = rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_word(int addr, uint16_t *p)
{
	uint8_t src;
    uint8_t rx[2];
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_blocking(i2c_default, 0x50, &src, 1, true);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_blocking(i2c_default, 0x50, rx, 2, false);
    if (ret != 2) {
		restore_interrupts(flags);
		return ret;
    }

	*p = *(uint16_t*)rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_long(int addr, uint32_t *p)
{
	uint8_t src;
    uint8_t rx[4];
    int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_blocking(i2c_default, 0x50, &src, 1, true);
    if (ret != 1) {
		restore_interrupts(flags);
		return ret;
    }

    ret = i2c_read_blocking(i2c_default, 0x50, rx, 4, false);
    if (ret != 4) {
		restore_interrupts(flags);
		return ret;
    }

	*p = *(uint32_t*)rx;

	restore_interrupts(flags);

	return PICO_OK;
}

int eeprom_read_page(int addr, uint8_t *p, size_t len)
{
	uint8_t src;
	uint8_t rx[16];
	int ret;
	uint32_t flags = save_and_disable_interrupts();

	src = (uint8_t)addr;

	ret = i2c_write_blocking(i2c_default, 0x50, &src, 1, true);
	if (ret != 1) {
		restore_interrupts(flags);
		return ret;
	}

	ret = i2c_read_blocking(i2c_default, 0x50, rx, len, false);
	if (ret != len) {
		restore_interrupts(flags);
		return ret;
	}

	memcpy(p, rx, len);

	restore_interrupts(flags);

	return PICO_OK;
}

void SerCmdExec(void) {
	char* ptr;
	char buf[8];

	if (memcmp((char*)CmdBuf, "stop", 4) == 0) {
		sout = false;
	} else
	if (memcmp((char*)CmdBuf, "go", 2) == 0) {
		sout = true;
	} else
	if (memcmp((char*)CmdBuf, "set ", 4) == 0) {
		ptr = (char*)CmdBuf + 4;
		if (memcmp(ptr, "gms=", 4) == 0) {
			ptr += 4;
			gamma_sensitivity = atoi(ptr);
		} else
		if (memcmp(ptr, "atc=", 4) == 0) {
			ptr += 4;
			alarm_trigger_cpm = atoi(ptr);
		} else
		if (memcmp(ptr, "hvg=", 4) == 0) {
			ptr += 4;
			hvg_pulsewidth = atoi(ptr);
			pwm_set_chan_level(hvgPwmSlice, PWM_CHAN_A, hvg_pulsewidth);
		} else
		if (memcmp(ptr, "bps=", 4) == 0) {
			uint32_t bps;
			ptr += 4;
			bps = atol(ptr);
			uart_set_baudrate(uart1, bps);
		} else
		if (memcmp(ptr, "cfg=", 4) == 0) {
			ptr += 4;
			tone = atoi(ptr);
			pwm_set_clkdiv(bzsPwmSlice, tone);
		} else
		if (memcmp(ptr, "snd=", 4) == 0) {
			ptr += 4;
			if (memcmp(ptr, "on", 2) == 0) {
				is_sound_enabled = true;
			} else
			if (memcmp(ptr, "off", 3) == 0) {
				is_sound_enabled = false;
			}
		} else
		if (memcmp(ptr, "dpd=", 4) == 0) {
			ptr += 4;
			if (memcmp(ptr, "on", 2) == 0) {
				ssd1306_poweron(&disp);
			} else
			if (memcmp(ptr, "off", 3) == 0) {
				ssd1306_poweroff(&disp);
			}
		}
	} else
	if (memcmp((char*)CmdBuf, "save", 4) == 0) {
		eeprom_write_word(OFS_GMS, gamma_sensitivity);
		eeprom_write_word(OFS_ALARM, alarm_trigger_cpm);
		eeprom_write_word(OFS_PWM_WIDTH, hvg_pulsewidth);
		eeprom_write_byte(OFS_SOUND, (is_sound_enabled == true)? 1 : 0);
	} else
	if (memcmp((char*)CmdBuf, "show", 4) == 0) {
		printf("ttc: ");
		sprintf(buf, "%lu", total_cnt);
		printf(buf);
		printf("\r\n");

		printf("gms: ");
		sprintf(buf, "%u", gamma_sensitivity);
		printf(buf);
		printf("\r\n");

		printf("atc: ");
		sprintf(buf, "%u", alarm_trigger_cpm);
		printf(buf);
		printf("\r\n");

		printf("hvg: ");
		sprintf(buf, "%u", hvg_pulsewidth);
		printf(buf);
		printf("\r\n");
	} else
	if (memcmp((char*)CmdBuf, "reboot", 6) == 0) {
		software_reset();
	} else
	if (memcmp((char*)CmdBuf, "dfu", 3) == 0) {
		ssd1306_clear(&disp);
		sprintf(buf, "DFU MODE");
		ssd1306_draw_string(&disp, 0, 0, 2, buf);
		ssd1306_show(&disp);

		reset_usb_boot(0, 0);
	} else
	if (memcmp((char*)CmdBuf, "ver", 3) == 0) {
		uint8_t pmn[8];
		uint8_t rev[2];

		eeprom_read_page(OFS_MODELNAME, pmn, 8);
		eeprom_read_word(OFS_BDREV, (uint16_t*)rev);

		printf("%s\n", pmn);
		printf("%02X%02X\n", rev[1],rev[0]);

		sprintf(buf, "%s", FW_VERSION);
		printf(buf);
		printf("\r\n");
	}
}

void lineart(void) {

	//60 67
	ssd1306_draw_line(&disp, 60, 0, 60, 63);
	ssd1306_draw_line(&disp, 67, 0, 67, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//56 71
	ssd1306_draw_line(&disp, 56, 0, 56, 63);
	ssd1306_draw_line(&disp, 71, 0, 71, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//y 28, 35  
	ssd1306_draw_line(&disp, 0, 28, 127, 28);
	ssd1306_draw_line(&disp, 0, 35, 127, 35);
	ssd1306_show(&disp);
	sleep_ms(2);

	//52 75
	ssd1306_draw_line(&disp, 52, 0, 52, 63);
	ssd1306_draw_line(&disp, 75, 0, 75, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//48 79
	ssd1306_draw_line(&disp, 48, 0, 48, 63);
	ssd1306_draw_line(&disp, 79, 0, 79, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//y 24  39
	ssd1306_draw_line(&disp, 0, 24, 127, 24);
	ssd1306_draw_line(&disp, 0, 39, 127, 39);
	ssd1306_show(&disp);
	sleep_ms(2);

	//44 83
	ssd1306_draw_line(&disp, 44, 0, 44, 63);
	ssd1306_draw_line(&disp, 83, 0, 83, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//40 87
	ssd1306_draw_line(&disp, 40, 0, 40, 63);
	ssd1306_draw_line(&disp, 87, 0, 87, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 20 43
	ssd1306_draw_line(&disp, 0, 20, 127, 20);
	ssd1306_draw_line(&disp, 0, 43, 127, 43);
	ssd1306_show(&disp);
	sleep_ms(2);

	//36 91
	ssd1306_draw_line(&disp, 36, 0, 36, 63);
	ssd1306_draw_line(&disp, 91, 0, 91, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//28 99
	ssd1306_draw_line(&disp, 28, 0, 28, 63);
	ssd1306_draw_line(&disp, 99, 0, 99, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//24 103
	ssd1306_draw_line(&disp, 24, 0, 24, 63);
	ssd1306_draw_line(&disp, 103, 0, 103, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 12 51
	ssd1306_draw_line(&disp, 0, 12, 127, 12);
	ssd1306_draw_line(&disp, 0, 51, 127, 51);
	ssd1306_show(&disp);
	sleep_ms(2);

	//20 107
	ssd1306_draw_line(&disp, 20, 0, 20, 63);
	ssd1306_draw_line(&disp, 107, 0, 107, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//16 111
	ssd1306_draw_line(&disp, 16, 0, 16, 63);
	ssd1306_draw_line(&disp, 111, 0, 111, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 8 55
	ssd1306_draw_line(&disp, 0, 8, 127, 8);
	ssd1306_draw_line(&disp, 0, 55, 127, 55);
	ssd1306_show(&disp);
	sleep_ms(2);

	//12 115
	ssd1306_draw_line(&disp, 12, 0, 12, 63);
	ssd1306_draw_line(&disp, 115, 0, 115, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//8 119
	ssd1306_draw_line(&disp, 8, 0, 8, 63);
	ssd1306_draw_line(&disp, 119, 0, 119, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 4 59
	ssd1306_draw_line(&disp, 0, 4, 127, 4);
	ssd1306_draw_line(&disp, 0, 59, 127, 59);
	ssd1306_show(&disp);
	sleep_ms(2);

	//4 123
	ssd1306_draw_line(&disp, 4, 0, 4, 63);
	ssd1306_draw_line(&disp, 123, 0, 123, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	//0 127
	ssd1306_draw_line(&disp, 0, 0, 0, 63);
	ssd1306_draw_line(&disp, 127, 0, 127, 63);
	ssd1306_show(&disp);
	sleep_ms(2);

	// y 0 63
	ssd1306_draw_line(&disp, 0, 0, 127, 0);
	ssd1306_draw_line(&disp, 0, 63, 127, 63);
	ssd1306_show(&disp);
}


void title() {
	ssd1306_clear(&disp);

	lineart();
	sleep_ms(180);

	ssd1306_clear(&disp);

	ssd1306_draw_string(&disp, 6, 0, 1, "NetIO Devices");

	ssd1306_bmp_show_image(&disp, gc10nx_bmp, gc10nx_bmp_len);
	ssd1306_show(&disp);

	sleep_ms(220);

	ssd1306_draw_string(&disp, 6, 54, 1, "FW:");
	ssd1306_draw_string(&disp, 30, 54, 1, FW_VERSION);

	ssd1306_show(&disp);
	sleep_ms(740);

	ssd1306_clear(&disp);
}

int main() {
	static repeating_timer_t msecTimer;
	static repeating_timer_t secTimer;

	set_sys_clock_48mhz();

	stdio_init_all();

	// Futer Use (battery)
	adc_init();
	adc_gpio_init(29);
	adc_select_input(3);

	sleep_ms(400);

	i2c_init(i2c0, 400000); // FOR EEPROM
	gpio_set_function(20, GPIO_FUNC_I2C);
	gpio_set_function(21, GPIO_FUNC_I2C);
	gpio_pull_up(20);
	gpio_pull_up(21);
	bi_decl(bi_2pins_with_func(20, 21, GPIO_FUNC_I2C));

	// UART
	//uart_set_baudrate(uart1, 9600);
	irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
	irq_set_enabled(UART1_IRQ, true);
	uart_set_irq_enables(uart1, true, false);

	// OLED
	i2c_init(i2c1, 400000);
	gpio_set_function(14, GPIO_FUNC_I2C);
	gpio_set_function(15, GPIO_FUNC_I2C);
	gpio_pull_up(14);
	gpio_pull_up(15);
	bi_decl(bi_2pins_with_func(14, 15, GPIO_FUNC_I2C));
	disp.external_vcc = false;
	ssd1306_init(&disp, 128, 64, 0x3C, i2c1);

	title();

	// eeprom
	uint8_t soundMode;

	eeprom_read_byte(OFS_SOUND, &soundMode);

	is_sound_enabled = (soundMode & 0x01) ? true : false;

	eeprom_read_word(OFS_PWM_WIDTH, &hvg_pulsewidth);

	gpio_init(LED_PIN);		// DETECTION INDICATOR LED
	gpio_set_dir(LED_PIN, GPIO_OUT);

	gpio_set_function(10, GPIO_FUNC_PWM); // GPIO10
	gpio_set_function(16, GPIO_FUNC_PWM); // GPIO16

	// HV gen pulse
	hvgPwmSlice = pwm_gpio_to_slice_num(10);	// GPIO10 for HV gen pulse
	pwm_set_clkdiv(hvgPwmSlice, 6);
	pwm_set_wrap(hvgPwmSlice, 2047);
	pwm_set_chan_level(hvgPwmSlice, PWM_CHAN_A, hvg_pulsewidth);
	pwm_set_output_polarity(hvgPwmSlice, true, true);
	pwm_set_enabled(hvgPwmSlice, true);

	// Buzzer
	bzsPwmSlice = pwm_gpio_to_slice_num(16);	// GPIO16 for BZ
	pwm_set_clkdiv(bzsPwmSlice, 25);
	pwm_set_wrap(bzsPwmSlice, 999);
	pwm_set_chan_level(bzsPwmSlice, PWM_CHAN_A, 0);
	pwm_set_enabled(bzsPwmSlice, true);

	// DETECTION
	gpio_init(22);			// DETECTION PULSE INT CH1
	gpio_set_dir(22, GPIO_IN);
	gpio_pull_up(22);

	gpio_set_irq_enabled_with_callback(22, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

	gpio_init(2);			// DETECTION PULSE INT CH2
	gpio_set_dir(2, GPIO_IN);
	gpio_pull_up(2);

	gpio_set_irq_enabled(2, GPIO_IRQ_EDGE_FALL, true);

	add_repeating_timer_ms(-1, &msecTimer_callback, NULL, &msecTimer);
	add_repeating_timer_ms(-1000, &secTimer_callback, NULL, &secTimer);

	// LOAD EEPROM
	eeprom_read_word(OFS_GMS, &gamma_sensitivity);
	eeprom_read_word(OFS_ALARM, &alarm_trigger_cpm);
	eeprom_read_byte(OFS_DISPMODE, &dispMode);

	cmax = 0;
	cptr = 0;
	uptime = 0;

	int c;

	while(1) {

		if (get_bootsel_button()) {
			if (is_button_pressed) {
			}
			lp++;
			is_button_pressed = true;
		} else {
			if (lp > 20) {
				if (is_sound_enabled == true) {
					is_sound_enabled = false;
					eeprom_write_byte(OFS_SOUND, 0);
					lp = 0;
				} else {
					is_sound_enabled = true;
					eeprom_write_byte(OFS_SOUND, 1);
					lp = 0;
				}
			}

			if (lp <= 20 &&  lp > 0) {
				dispMode++;
				if (dispMode > MODEMAX) {
					dispMode = 0;
				}
				pd = true;
			}

			is_button_pressed = false;
			lp = 0;
		}

		sleep_ms(25);

		if (tud_cdc_connected()) {
			while ((c = tud_cdc_read_char()) != -1) {
				SerCmdProc(c);
			}
		}
	}

	return 0;
}
