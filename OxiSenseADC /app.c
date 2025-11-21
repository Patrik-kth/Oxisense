/* app.c — DFRobot MAX30102 V2.0 (SEN0344) via UART/Modbus-RTU, show SPO2 & BPM on LCD
 *
 * Sensor wiring (UART mode):
 *   Sensor 3V3 -> 3.3V
 *   Sensor GND -> GND
 *   Sensor TX  -> PA10 (USART0 RX)
 *   Sensor RX  -> PA9  (USART0 TX)
 *   INT/RST not used
 *
 * Sensor protocol (from DFRobot wiki):
 *   Baud: 9600 8N1
 *   Start measuring:  addr=0x20, FC=0x06, reg=0x0010, val=0x0001, CRC16(Modbus)
 *   Read SPO2/BPM:    addr=0x20, FC=0x03, reg=0x0006, qty=0x0004,  CRC16(Modbus)
 *   Response (read):  0x20 0x03 0x08 [SPO2][res][HB3][HB2][HB1][HB0][res][res] CRC
 *
 * References: DFRobot SEN0344 wiki (pinout, baud/protocol). 
 */

#include "gd32vf103.h"
#include "delay.h"
#include "lcd.h"
#include <stdint.h>

/* -------------------- LCD helpers -------------------- */
static void lcd_init_screen(void){
    Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);
    //LCD_ShowString(0, 0,  (const u8*)"SEN0344 (UART)", YELLOW);
    LCD_ShowString(0, 16, (const u8*)"SPO2:", WHITE);
    LCD_ShowString(0, 32, (const u8*)"BPM:", WHITE);
}

/* ------------ USART0 on PA9 (TX) / PA10 (RX) ------------ */
static void usart0_init_9600(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /* PA9: TX (AF push-pull), PA10: RX (input floating) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP,     GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 9600U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

static void usart0_putc(uint8_t c)
{
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    usart_data_transmit(USART0, c);
}

static int usart0_getc_timeout(uint8_t *c, uint32_t timeout_ms)
{
    while (timeout_ms--) {
        if (SET == usart_flag_get(USART0, USART_FLAG_RBNE)) {
            *c = (uint8_t)usart_data_receive(USART0);
            return 1;
        }
        delay_1ms(1);
    }
    return 0;
}

/* ------------- Modbus-RTU helpers (CRC16/A001) ------------- */
static uint16_t mb_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc; /* low byte first on the wire */
}

static void mb_send(const uint8_t *p, int n)
{
    for (int i = 0; i < n; ++i) usart0_putc(p[i]);
}

/* Receive exactly n bytes (with overall timeout) */
static int uart_read_exact(uint8_t *dst, int n, uint32_t timeout_ms)
{
    uint8_t c;
    for (int i = 0; i < n; ++i) {
        if (!usart0_getc_timeout(&c, timeout_ms)) return 0;
        dst[i] = c;
    }
    return 1;
}

/* ------------- Sensor high-level commands ------------- */
#define DEV_ADDR   0x20  /* module's Modbus address (DFRobot wiki) */

/* Send: Write Single Register: reg, val (both 16-bit). Returns 1 on success. */
static int sensor_write_reg(uint16_t reg, uint16_t val)
{
    uint8_t P[8] = {
        DEV_ADDR, 0x06,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(val >> 8), (uint8_t)(val & 0xFF),
        0x00, 0x00 /* CRC placeholder */
    };
    uint16_t crc = mb_crc16(P, 6);
    P[6] = (uint8_t)(crc & 0xFF);      /* CRC lo */
    P[7] = (uint8_t)(crc >> 8);        /* CRC hi */
    mb_send(P, 8);

    /* Echoed response is 8 bytes; verify CRC & contents */
    uint8_t R[8];
    if (!uart_read_exact(R, 8, 200)) return 0;
    uint16_t rcrc = (uint16_t)R[6] | ((uint16_t)R[7] << 8);
    if (mb_crc16(R, 6) != rcrc) return 0;
    if (R[0] != DEV_ADDR || R[1] != 0x06) return 0;
    return 1;
}

/* Read holding registers: starting reg and quantity (qty). Read into buf (bytes). */
static int sensor_read_regs(uint16_t reg, uint16_t qty, uint8_t *data, int data_len)
{
    uint8_t P[8] = {
        DEV_ADDR, 0x03,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(qty >> 8), (uint8_t)(qty & 0xFF),
        0x00, 0x00
    };
    uint16_t crc = mb_crc16(P, 6);
    P[6] = (uint8_t)(crc & 0xFF);
    P[7] = (uint8_t)(crc >> 8);
    mb_send(P, 8);

    /* Response header is 3 bytes, then N data bytes, then 2 CRC */
    uint8_t hdr[3];
    if (!uart_read_exact(hdr, 3, 200)) return 0;
    if (hdr[0] != DEV_ADDR || hdr[1] != 0x03) return 0;
    uint8_t nbytes = hdr[2];
    if (nbytes > data_len) return 0;

    if (!uart_read_exact(data, nbytes, 200)) return 0;
    uint8_t crcbuf[3+255];
    for (int i=0;i<3;i++) crcbuf[i] = hdr[i];
    for (int i=0;i<nbytes;i++) crcbuf[3+i] = data[i];

    uint8_t crcraw[2];
    if (!uart_read_exact(crcraw, 2, 200)) return 0;
    uint16_t rcrc = (uint16_t)crcraw[0] | ((uint16_t)crcraw[1] << 8);
    if (mb_crc16(crcbuf, 3+nbytes) != rcrc) return 0;
    return nbytes;
}

/* Start the sensor’s measurement state machine */
static int sensor_start_collect(void)
{
    /* Write reg 0x0010 = 0x0001 (start) */
    return sensor_write_reg(0x0010, 0x0001);
}

/* Optional: stop collection (not used here) */
// static int sensor_stop_collect(void){ return sensor_write_reg(0x0010, 0x0002); }

/* Read SPO2 and BPM; returns 1 on success, fills *spo2 (0..100) and *bpm */
static int sensor_read_spo2_bpm(int *spo2, int *bpm)
{
    uint8_t D[16];
    int n = sensor_read_regs(0x0006, 0x0004, D, sizeof(D));
    if (n != 8) return 0;

    /* D layout per wiki: [0]=SPO2, [1]=reserved, [2..5]=Heartbeat(4 bytes), [6..7]=reserved */
    *spo2 = (int)D[0];

    uint32_t hb = ((uint32_t)D[2] << 24) | ((uint32_t)D[3] << 16) | ((uint32_t)D[4] << 8) | (uint32_t)D[5];
    *bpm = (int)hb;  /* values are reported in beats per minute */

    return 1;
}

/* ---------------------------- main ---------------------------- */
int main(void)
{
    lcd_init_screen();
    usart0_init_9600();
    delay_1ms(200);

    while (!sensor_start_collect()) {
        /* Re-sync UART in case of framing/garbage, then wait a bit */
        usart0_init_9600();
        delay_1ms(1000);
    }
    
    delay_1ms(1500);

    int spo2 = 0, bpm = 0;
    int last_spo2 = -1, last_bpm = -1;   // track last drawn values

    while (1) {
        int bpm_changed = 0;
        int spo2_changed = 0;

        if (sensor_read_spo2_bpm(&spo2, &bpm)) {
            /* Detect changes first */
            spo2_changed = (spo2 != last_spo2);
            bpm_changed  = (bpm  != last_bpm);

            /* Draw the markers FIRST if something changed */
            if (spo2_changed) LCD_ShowString(120, 16, (const u8*)"!", WHITE);
            if (bpm_changed)  LCD_ShowString(120, 32, (const u8*)"!", WHITE);

            /* Now update numbers */
            if (spo2_changed) {
                LCD_ShowNum(48, 16, spo2, 3, GREEN);
                last_spo2 = spo2;
            }
            if (bpm_changed) {
                LCD_ShowNum(48, 32, bpm,  3, GREEN);
                last_bpm = bpm;
            }

            /* If nothing changed this cycle, clear the markers */
            if (!spo2_changed) LCD_ShowString(120, 16, (const u8*)" ", WHITE);
            if (!bpm_changed)  LCD_ShowString(120, 32, (const u8*)" ", WHITE);
        }
        /* Poll a bit faster to reduce quantization delay (optional) */
        delay_1ms(500);  // try 250–500 ms instead of 1000 ms
    }
}
