/* 
  USB interface for temperature probe with DS18X20 from Dallas Semiconductor
*/

#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include "onewire/onewire.h"
#include "usbdrv/usbdrv.h"

/* State mashine states */
typedef enum { DETECT, CONFIG, MEASURE, ACQUIRE, IDLE } sensor_state_t;

/* Onboard LED */
#define LED_G_PIN	PC0
#define LED_R_PIN	PC1
#define LED_OUT		PORTC
#define LED_DDR		DDRC

/* One wire configuration */
#define OW_PIN	PB3
#define OW_IN	&PINB
#define OW_OUT	&PORTB
#define OW_DDR	&DDRB

/* DS18X20 configuration */
#define DS18X20_SP_SIZE		9
#define DS18X20_C_SIZE		3
#define DS18X20_CONVERT_T	0x44
#define DS18X20_READ_T		0xbe
#define DS18X20_WRITE_C		0x4e
#define DS18X20_SAVE_C		0x48
#define DS18X20_CONFIG_REG	4

#define DS18S20_ID	0x10
#define DS18B20_ID	0x28

/* USB commands */
#define USB_MEASURE		1
#define USB_READ_TEMP	2
#define USB_READ_NUMBER	3
#define USB_READ_ROM	4
#define USB_DETECT		5
#define USB_PRECISION	6
#define USB_SETMODE		7
#define USB_GETMODE		8
#define USB_POKE		9
#define USB_PEEK		10

/* Main loop delay */
#define MAIN_DELAY_MS		2

/* Repeat measurements every 10 seconds. */
#define IDLE_INTERVAL_MS	10000
/* Wait 750 ms before acquiring. */
#define ACQUIRE_INTERVAL_MS	750

#define MAXSENSORS 5

/* Storage */
uint8_t id_sensor[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t sp_sensor[MAXSENSORS][DS18X20_SP_SIZE];

/* Number of attached sensors */
uint8_t nr_sensors;
uint8_t cur_sensor;

/* Current operating mode */
uint8_t mode;

/* State mashine */
sensor_state_t state;

/* USB return message */
static uchar replyBuf[DS18X20_SP_SIZE + 1] = { 0 };

/* USB HID report descriptor */
PROGMEM const char usbHidReportDescriptor[22] = {
    0x05, 0x0c,
    0x0a, 0x05, 0x01,
    0xa1, 0x01,
    0x0a, 0x05, 0x01,
    0x15, 0x00,
    0x26, 0xff, 0x00,
    0x75, 0x08,
    0x95, 0x10,
    0xb1, 0x02,
    0xc0
};

/* == Procedures == */

inline void led_G_On(void) {
  LED_OUT &= ~_BV(LED_G_PIN);
}

inline void led_G_Off(void) {
  LED_OUT |= _BV(LED_G_PIN);
}

inline void led_R_On(void) {
  LED_OUT &= ~_BV(LED_R_PIN);
}

inline void led_R_Off(void) {
  LED_OUT |= _BV(LED_R_PIN);
}

inline void schedule_detection(void) {
  state = DETECT;
  /* Skip part A */
  nr_sensors = 0;
}

inline void schedule_reconfiguration(uint8_t sensor_id) {
  state = CONFIG;
  /* Go to part A */
  cur_sensor = sensor_id;
}

uint8_t detect_sensors(void) {
  /* Detect connected sensors, save their ROM to id_sensor and return number of detected sensors. */

  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t cur_sensor;
  uint8_t diff;

  cur_sensor = 0;
  for ( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && cur_sensor < MAXSENSORS; ) {
    diff = ow_rom_search( diff, &id[0] );
    if ( diff == OW_PRESENCE_ERR || diff == OW_DATA_ERR )
      break;
    if ( id[0] == DS18B20_ID || id[0] == DS18S20_ID ) {
      /* Count only DS18X20 sensors. */
      for (i = 0; i < OW_ROMCODE_SIZE; i++ )
        /* Copy ROM. */
	    id_sensor[cur_sensor][i] = id[i];
      cur_sensor++;
    }
  }

  return cur_sensor;
}

USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {

  uint8_t sensor_id;
  uint8_t i;
  usbRequest_t *rq = (void *)data;
  usbMsgPtr = (usbMsgPtr_t)replyBuf;

  if ( (rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR ) {
  /* Handle our requests. */

  switch ( rq->bRequest ) {

	case USB_MEASURE:
	  if ( state == IDLE ) {
	    state = MEASURE;
	    replyBuf[0] = 1;
	  }
	  else
	    replyBuf[0] = 0;
	  return 1;

	case USB_READ_TEMP:
	  sensor_id = rq->wValue.bytes[0];
	  if ( sensor_id >= nr_sensors )
	    return 0;
	  for ( i = 0; i < DS18X20_SP_SIZE; i++ )
	    /* Copy scratchpad. */
	    replyBuf[i] = sp_sensor[sensor_id][i];
	  return DS18X20_SP_SIZE;

	case USB_READ_NUMBER:
	  replyBuf[0] = nr_sensors;
	  return 1;

	case USB_READ_ROM:
	  sensor_id = rq->wValue.bytes[0];
	  if ( sensor_id >= nr_sensors )
	    return 0;
	  for ( i = 0; i < OW_ROMCODE_SIZE; i++ )
	    /* Copy ROM. */
	    replyBuf[i] = id_sensor[sensor_id][i];
	  return OW_ROMCODE_SIZE;

	case USB_DETECT:
	  schedule_detection();
	  replyBuf[0] = 1;
	  return 1;

	case USB_PRECISION:
	  sensor_id = rq->wValue.bytes[0] & 0x1f;
	  if ( sensor_id >= nr_sensors )
	    return 0;
	  sp_sensor[sensor_id][DS18X20_CONFIG_REG] = ((rq->wValue.bytes[0] >> 1) & 0x60) | 0x1f;
	  schedule_reconfiguration(sensor_id);
	  replyBuf[0] = sp_sensor[sensor_id][DS18X20_CONFIG_REG];
	  return 1;

	case USB_SETMODE:
	  mode = rq->wValue.bytes[0] & 0x01;
	case USB_GETMODE:
	  replyBuf[0] = mode;
	  return 1;

    case USB_POKE:
      sensor_id = rq->wValue.bytes[0];
	  if ( sensor_id >= nr_sensors )
	    return 0;
      sp_sensor[sensor_id][2] = rq->wIndex.bytes[0];
      sp_sensor[sensor_id][3] = rq->wIndex.bytes[1];
      schedule_reconfiguration(sensor_id);

    case USB_PEEK:
      sensor_id = rq->wValue.bytes[0];
	  if ( sensor_id >= nr_sensors )
	    return 0;
      replyBuf[0] = sp_sensor[sensor_id][2];
      replyBuf[1] = sp_sensor[sensor_id][3];
      return 2;

  }

  }
  else if ( (rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS ) {

    if ( rq->bRequest == USBRQ_HID_GET_REPORT ) {

      for ( i = 0; i < 2; i++ )
        replyBuf[i] = sp_sensor[0][i];
      return 2;

    }

  }

  return 0;
}

#define COUNTER_RESET \
  delay_counter = 0; \
  cur_sensor = 0;

int main() {

  uint8_t i;
  uint32_t delay_counter;

  /* Enable watchdog timer with 1 second. */
  wdt_enable(WDTO_1S);

  /* Set LED pin as output. */
  LED_DDR = _BV(LED_G_PIN) | _BV(LED_R_PIN);
  led_G_Off();
  led_R_Off();

  /* Initialize one wire interface. */
  ow_set_bus(OW_IN, OW_OUT, OW_DDR, OW_PIN);

  /* Initialize the driver. */
  usbInit();

  usbDeviceDisconnect();
  for ( i = 0; i < 250; i++ ) {
    wdt_reset();
    _delay_ms(2);
  }
  usbDeviceConnect();

  /* Enable global interrupts. */
  sei();

  /* Starting state. */
  schedule_detection();
  COUNTER_RESET
  mode = 1;

  while (1) {

    /* Reset the watchdog timer. */
    wdt_reset();

    /* Poll USB data. */
    usbPoll();

/*
    if ( usbInterruptIsReady() ) {
      usbSetInterrupt((void *)&sp_sensor[0][0], 2);
    }
*/

    if ( cur_sensor < nr_sensors ) {

      /* PART A */
      switch ( state ) {

#ifdef SEPARATE_MEASURE
        /* state MEASURE */
        case ACQUIRE:
          ow_command(DS18X20_CONVERT_T, &id_sensor[cur_sensor][0]);
          break;
#endif

        /* state ACQUIRE */
        case IDLE:

          _delay_ms(1);
          delay_counter += 1;

          ow_command(DS18X20_READ_T, &id_sensor[cur_sensor][0]);
          for ( i = 0; i < DS18X20_SP_SIZE; i++ )
            /* Copy scratchpad */
            sp_sensor[cur_sensor][i] = ow_byte_rd();

          break;

        case CONFIG:

          ow_command(DS18X20_WRITE_C, &id_sensor[cur_sensor][0]);
          for ( i = 0; i < DS18X20_C_SIZE; i++ )
            ow_byte_wr(sp_sensor[cur_sensor][i + 2]);
          ow_command(DS18X20_SAVE_C, &id_sensor[cur_sensor][0]);

          /* Skip others. */
          cur_sensor = nr_sensors;
          /* Set next state */
          state = IDLE;

          break;

        default:
          break;

      }
      cur_sensor++;

    }
    else {

      /* PART B */
      if ( state == DETECT ) {

        /* Search for connected sensors. */
        led_R_On();
        nr_sensors = detect_sensors();
        led_R_Off();

        /* Set next state. */
        state = mode ? IDLE : MEASURE;

        /* Skip part A. */
        cur_sensor = nr_sensors;

        /* Do not apply delay. */
        continue;

      }
      else if ( state == MEASURE ) {

        /* Start measuring. */
        led_G_On();

#ifndef SEPARATE_MEASURE
        ow_command(DS18X20_CONVERT_T, 0);
#else
        /* Go to part A. */
        cur_sensor = 0;
#endif

        /* Set next state. */
        state = ACQUIRE; 
        delay_counter = 0;

      }
      else if ( state == ACQUIRE && delay_counter > ACQUIRE_INTERVAL_MS ) {

        /* Start acquiring data. */
        led_G_Off();

        /* Set next state. */
        state = IDLE;
        COUNTER_RESET
        
      }
      else if ( state == IDLE && delay_counter > IDLE_INTERVAL_MS ) {

        /* Restart. */
        state = MEASURE;
        delay_counter = 0;

      }

      /* In mode 1 stay in IDLE state */
      if ( !mode || state != IDLE )
        delay_counter += MAIN_DELAY_MS;

      /* Apply delay */
      _delay_ms(MAIN_DELAY_MS); 

    }

  }

  return 0;
}
