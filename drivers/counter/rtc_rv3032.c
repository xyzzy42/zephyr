/* Micro Crystal RV3032 RTC
  
   Copyright (C) 2023 Igor Institute, Inc.
   Author: Trent Piepho <trent.piepho@igorinstitute.com>
   
   This driver supports a number of functions of the Micro Crystal RV3032 RTC.

   Zephyr has limited support for RTCs.  This primarily uses the Counter API,
   but the mapping from Counter functions to the RTC is not perfect.

   Counter
   =======

   The counter will return the time as an unsigned 32-bit POSIX time_t.  This
   will not roll-over until 2106.  The RV3032 itself will roll-over in 2100. 
   While the RV3032 provides time in hundredth seconds, it's not possible to get
   this precision from the 32-bits allowed in the Zephyr's counter API.  Newer
   versions of Zephyr address this with 64-bit counters.

   The counter has three alarms:
	Alarm 0.  
	An absolute date/time alarm.  The COUNTER_ALARM_CFG_ABSOLUTE should be
	set.  A 32-bit time_t, as above, should be supplied as the alarm ticks
	value.  The alarm only supports minutes resolution, this effectively
	means the time supplied will be rounded down to the minute.

	This alarm will re-trigger every month which matches the supplied value.
	E.g, a time_t representing 2023/2/26 12:34:56 will trigger on 2023/1/26
	12:34:00, 2023/2/26 12:34:00, 2023/3/26 12:34:00, and so on.

	The required monthly triggering is a hardware limitation.  It is not
	possible to select daily, hourly, minute-ly, etc. triggering either,
	which is an API limitation.

	Alarm 1:
	This is a periodic countdown alarm.  The alarm ticks value should be a
	time in seconds.  It will trigger every this many seconds, starting from
	the time it is set.

	Different frequencies than 1 Hz, such as 1/60 Hz, 64 Hz, and 4096 Hz are
	not supported due to API limitations.

	Alarm 2:
	This is a periodic 1 Hz or 1/60 Hz alarm, i.e. every second or every
	minute.  The alarm ticks value should be 1 or 60.  The 1 Hz alarm is
	triggered when fractional seconds are 0, and the 1/60 Hz alarm when
	seconds are zero too.  This is unlike alarm 1, where the fractional
	seconds will be the value when the alarm was set.

   The alarm callback will be passed the counter value from when the alarm was
   triggered.  This is not the same as the ticks value specified when the alarm
   was set.  The alarm trigger time is not captured by hardware and so may be
   delayed due to Zephyr scheduling latency.

   The Counter API provides no way to set the counter.  So while the RTC can be
   read this way, it can not be set.

   Custom RTC API
   ==============

   Custom device specific functions support additional features.  These are:
	int rv3032_rtc_set_time(const struct device*, time_t unix_time);
	int rv3032_rtc_set_time_tm(const struct device*, const struct tm*);
	int rv3032_rtc_get_time(const struct device*, time_t *unix_time);
	int rv3032_rtc_get_time_tm(const struct device*, struct tm*, uint8_t *hsec);
	int rv3032_rtc_get_time_ts(const struct device*, struct timespec *);

   These allow setting and reading the RTC.  rv3032_rtc_set/get_time_tm(), which
   a broken down struct tm for the time and an additional hundredth seconds
   field available only when reading the time, is the closest to the native
   format of the RTC.  The other functions are wrappers around the struct tm
   based functions.

   If the RTC has not been set, the "get" functions will return -ENODATA.  They
   will do this until the RTC is set.  The unset state is not lost by an MCU
   restart.

   Sensor API
   ==========

   The temperature sensor is represented using the Sensor API.  It has three
   channels.  One is the standard DIE_TEMP channel, while the other two are
   device specific channels for Tlow and Thigh time-stamps.

   The attributes SENSOR_ATTR_LOWER_THRESH and SENSOR_ATTR_UPPER_THRESH can be
   set to enable the RV3023's temperature alarms.  The trigger
   SENSOR_TRIG_THRESHOLD can be used to trigger on the resulting interrupts. 
   These values are not reset when the RTC driver initializes.

   The SENSOR_TRIG_DATA_READY trigger is not supported as there is no hardware
   temperature update interrupt.  The Sensor API provides no way to disable a
   trigger once enabled.

   The device specific sensor channels SENSOR_CHAN_RV3032_TLOWER_TS and
   SENSOR_CHAN_RV3032_TUPPER_TS will return the time of the first temperature
   threshold event, which is cleared when retrieved.  The val1 field will be an
   unsigned time_t, as in the counter.  val2 is not sub-seconds, as the
   timestamp do not have sub-second precision, but will instead be the count of
   events.  This count is reset by the handler, but can accumulate while the MCU
   is off.

   The temperature alarms will remain active while the RV3032 is on battery
   backup power.  If an alarm occurred on backup power, or before
   SENSOR_TRIG_THRESHOLD is enabled, it will not be lost.  In this case, the
   call to sensor_trigger_set() will immediately trigger the handler and the
   time-stamp information can be retrieved.  The temperature that triggered the
   alarm is not saved by the hardware.  Only the current temperature is
   available on channel SENSOR_CHAN_DIE_TEMP.

   EEPROM API
   ==========

   The RV3032 has 16 bytes of user NVRAM, which requires backup power to
   preserve, and 32 bytes of EEPROM, which does not.  These are both provided
   under the EEPROM API, as two different devices.

   The EEPROM will be stored as each byte is written.
   
   Limitations (beyond those mentioned above)
   ==========================================

   The EEPROM password feature is not supported.

   There is no support for time-stamping the external event pin.

   The CLKOUT pin frequency can not be changed at run time.

   There is no support for configuring the CLKOUT enable-on-event feature.

   The temperature offset can not be set, but it can be read.

   The crystal calibration can not be set.

   It's not possible to get an accurate timestamp of the alarm ISR.  It will be
   many microseconds, if not milliseconds, later when the alarm callback was
   called.  This prevents accurate calibration using the PPS alarm or other use
   cases that need an accurate interrupt signal.
 */

#define DT_DRV_COMPAT microcrystal_rv3032

#include <device.h>
#include <drivers/counter.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/timeutil.h>
#include <sys/util.h>
#include <time.h>

#include <drivers/rtc/rv3032.h>
#include "rtc_rv3032_regs.h"

LOG_MODULE_REGISTER(RV3032, CONFIG_COUNTER_LOG_LEVEL);

struct rv3032_config {
	// This must be first, the Zephyr counter API functions assume it!
	struct counter_config_info generic;
	const struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpio;
	const struct gpio_dt_spec evi_gpio;
	/* CLKOUT and charger */
	uint8_t pmu;
	uint8_t clkout1;
	uint8_t clkout2;
	bool	clken;
};

struct rv3032_data {
	struct k_mutex lock;
	struct gpio_callback irq;
	struct k_work irq_work;
	const struct device *dev;
	const struct device *sensor_dev;
	bool unset;

	// For Timer callbacks
	counter_alarm_callback_t alarm_callback[3];
	void *alarm_user_data[3];

	// For Sensor callback (with sensor_temp)
	sensor_trigger_handler_t sensor_callback;
	const struct sensor_trigger *sensor_trigger;
	// Sensor Data
	int16_t sensor_temp;

	// Cache of control registers
	uint8_t ctrl1;
	uint8_t ctrl2;
	uint8_t ctrl3;
};

static int reg_read(const struct device *dev, uint8_t addr)
{
	uint8_t val;
	const struct rv3032_config *cfg = dev->config;
	int ret = i2c_reg_read_byte_dt(&cfg->i2c, addr, &val);

	return ret < 0 ? ret : val;
}

static int reg_read_n(const struct device *dev, uint8_t addr,
			     uint8_t* vals, uint32_t count)
{
	const struct rv3032_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, addr, vals, count);
}

static int reg_write(const struct device *dev, uint8_t addr, uint8_t val)
{
	const struct rv3032_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, addr, val);
}

static int reg_write_n(const struct device *dev, uint8_t addr,
			      const uint8_t vals[], uint32_t count)
{
	const struct rv3032_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, addr, vals, count);
}

// Clear status bits set in status
static int reg_status_clear(const struct device *dev, uint8_t status)
{
	return reg_write(dev, RV3032_STATUS, (uint8_t)~status);
}

static int eeprom_wait(const struct device *dev)
{
	int timeout = 100;
	int tlsb;

	do {
		tlsb = reg_read(dev, RV3032_TLSB);
		if (tlsb < 0) {
			LOG_ERR("EEPROM write I2C error: %d", tlsb);
			return tlsb;
		}
		if (!(tlsb & RV3032_TLSB_EEBUSY)) {
			LOG_DBG("EEBusy for ~%d ms", 100 - timeout);
			return tlsb;
		}
		k_sleep(K_MSEC(1));
	} while (--timeout);
	LOG_WRN("EEPROM write timeout");
	return -ETIMEDOUT;
}

static int reg_eeprom_write(const struct device *dev, uint8_t addr, uint8_t val)
{
	int ret;

	LOG_DBG("EEPROM Write %02x = %02x", addr, val);
	ret = eeprom_wait(dev);
	if (ret < 0) {
		return ret;
	} else if (ret & RV3032_TLSB_EEF) {
		LOG_DBG("Clearing EEPROM Write Fault");
		reg_write(dev, RV3032_TLSB, (uint8_t)~RV3032_TLSB_EEF);
	}

	const struct eeprom_regs regs = { .addr = addr, .data = val, .cmd = RV3032_EEPROM_CMD_WRITE };
	reg_write_n(dev, RV3032_EEPROM_ADDR, &regs.addr, sizeof(regs));

	ret = eeprom_wait(dev);
	if (ret < 0) {
		return ret;
	} else if (ret & RV3032_TLSB_EEF) {
		LOG_WRN("EEPROM Write Fault at %02x", addr);
		reg_write(dev, RV3032_TLSB, (uint8_t)~RV3032_TLSB_EEF);
		return -EPERM;
	}

	return 0;
}

static int reg_eeprom_read(const struct device *dev, uint8_t addr, uint8_t *val)
{
	int ret;

	LOG_DBG("EEPROM Read %02x", addr);
	ret = eeprom_wait(dev);
	if (ret < 0) {
		return ret;
	}

	const struct eeprom_regs regs = { .addr = addr, .data = 0, .cmd = RV3032_EEPROM_CMD_READ };
	reg_write_n(dev, RV3032_EEPROM_ADDR, &regs.addr, sizeof(regs));

	ret = eeprom_wait(dev);
	if (ret < 0) {
		return ret;
	}

	ret = reg_read(dev, RV3032_EEPROM_DATA);
	if (ret < 0) {
		return ret;
	}
	*val = (uint8_t)ret;
	return 0;
}


static void decode_tm(const struct time_regs *regs, struct tm *tm)
{
	tm->tm_sec = bcd2bin(regs->clock.sec);
	tm->tm_min = bcd2bin(regs->clock.min);
	tm->tm_hour = bcd2bin(regs->clock.hour);
	tm->tm_wday = bcd2bin(regs->cal.weekday);
	tm->tm_mday = bcd2bin(regs->cal.day);
	tm->tm_mon = bcd2bin(regs->cal.month) - 1;
	tm->tm_year = bcd2bin(regs->cal.year) + 100;
	tm->tm_isdst = 0;
}

static void encode_tm(struct time_regs *regs, const struct tm *tm)
{
	regs->clock.sec = bin2bcd(tm->tm_sec);
	regs->clock.min = bin2bcd(tm->tm_min);
	regs->clock.hour = bin2bcd(tm->tm_hour);
	regs->cal.weekday = bin2bcd(tm->tm_wday);
	regs->cal.day = bin2bcd(tm->tm_mday);
	regs->cal.month = bin2bcd(tm->tm_mon + 1);
	regs->cal.year = bin2bcd(tm->tm_year - 100);
}

int rv3032_rtc_set_time_tm(const struct device *dev, const struct tm *tm)
{
	struct rv3032_data *data = dev->data;
	struct time_regs regs;

	if (tm->tm_year < 100 || tm->tm_year > 199) {
		return -EINVAL;
	}

	encode_tm(&regs, tm);

	k_mutex_lock(&data->lock, K_FOREVER);

	int ret = reg_write_n(dev, RV3032_SEC, &regs.clock.sec, sizeof(regs) - 1);

	if (!ret && data->unset) {
		// Clock is now set
		data->unset = false;
		reg_status_clear(dev, RV3032_STATUS_VLF | RV3032_STATUS_PORF);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

int rv3032_rtc_set_time(const struct device *dev, time_t unix_time)
{
	struct tm tm;

	gmtime_r(&unix_time, &tm);
	return rv3032_rtc_set_time_tm(dev, &tm);
}

int rv3032_rtc_get_time_tm(const struct device *dev, struct tm *tm, uint8_t *hsec)
{
	struct rv3032_data *data = dev->data;
	struct time_regs regs;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = data->unset ? -ENODATA :
		reg_read_n(dev, RV3032_HSEC, &regs.clock.hsec, sizeof(regs));
	k_mutex_unlock(&data->lock);

	if (!ret) {
		decode_tm(&regs, tm);
		if (hsec) {
			*hsec = bcd2bin(regs.clock.hsec);
		}
	}

	return ret;
}

int rv3032_rtc_get_time(const struct device *dev, time_t *unix_time)
{
	struct tm tm;
	int ret = rv3032_rtc_get_time_tm(dev, &tm, NULL);

	if (!ret) {
		*unix_time = mktime(&tm);
	}

	return ret;
}

int rv3032_rtc_get_time_ts(const struct device *dev, struct timespec *ts)
{
	struct tm tm;
	uint8_t hsec;
	int ret = rv3032_rtc_get_time_tm(dev, &tm, &hsec);

	if (!ret) {
		ts->tv_sec = mktime(&tm);
		ts->tv_nsec = hsec * (NSEC_PER_SEC/100);
	}

	return ret;
}

static int rv3032_counter_start(const struct device *dev)
{
	struct rv3032_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	// Clear STOP bit
	data->ctrl2 &= ~RV3032_CTRL2_STOP;
	ret = reg_write(dev, RV3032_CTRL2, data->ctrl2);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_counter_stop(const struct device *dev)
{
	struct rv3032_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	// Set STOP bit
	data->ctrl2 |= RV3032_CTRL2_STOP;
	ret = reg_write(dev, RV3032_CTRL2, data->ctrl2);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_counter_get_value(const struct device *dev, uint32_t *ticks)
{
	time_t t;

	int ret = rv3032_rtc_get_time(dev, &t);
	if (!ret) {
		*ticks = t & 0xffffffff;
	}
	return ret;
}

static uint32_t rv3032_counter_get_top_value(const struct device *dev)
{
	return UINT32_MAX;
}

static int rv3032_counter_set_top_value(const struct device *dev,
					const struct counter_top_cfg *cfg)
{
	return -ENOTSUP;
}

// Set alarm.  Only supporting absolute times.  This will repeat monthly.
// Should check if alarm is in the past, but doesn't.
static int set_alarm(const struct device *dev, const struct counter_alarm_cfg *alarm_cfg)
{
	struct rv3032_data *data = dev->data;
	struct tm tm;
	struct alarm_regs regs;

	/* Absolute time only alarm */
	if (!(alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE)) {
		return -EINVAL;
	}

	const time_t t = alarm_cfg->ticks;
	gmtime_r(&t, &tm);

	k_mutex_lock(&data->lock, K_FOREVER);

	// Save alarm callback info
	data->alarm_callback[0] = alarm_cfg->callback;
	data->alarm_user_data[0] = alarm_cfg->user_data;

	if (data->ctrl2 & RV3032_CTRL2_AIE) {
		reg_write(dev, RV3032_CTRL2, data->ctrl2 & ~RV3032_CTRL2_AIE);
	}

	regs.min = bin2bcd(tm.tm_min);
	regs.hour = bin2bcd(tm.tm_hour);
	regs.day = bin2bcd(tm.tm_mday);
	reg_write_n(dev, RV3032_ALARM_MIN, &regs.min, sizeof(regs));

	reg_status_clear(dev, RV3032_STATUS_AF);
	data->ctrl2 |= RV3032_CTRL2_AIE;
	reg_write(dev, RV3032_CTRL2, data->ctrl2);

	k_mutex_unlock(&data->lock);

	LOG_DBG("Alarm set to 20xx-xx-%02d %02d:%02d:00", tm.tm_mday, tm.tm_hour, tm.tm_min);

	return 0;
}

// Set the periodic countdown timer.  There's no way to configure the clock frequency.
static int set_timer(const struct device *dev, const struct counter_alarm_cfg *alarm_cfg)
{
	struct rv3032_data *data = dev->data;
	uint8_t timer[2];

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		return -EINVAL;
	}

	if (alarm_cfg->ticks > 0xfffff) {
		return -ERANGE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	// Save alarm callback info
	data->alarm_callback[1] = alarm_cfg->callback;
	data->alarm_user_data[1] = alarm_cfg->user_data;

	timer[0] = alarm_cfg->ticks & 0xff;
	timer[1] = (alarm_cfg->ticks >> 8) & 0xf;

	reg_write_n(dev, RV3032_TIMER0, timer, sizeof(timer));

	reg_status_clear(dev, RV3032_STATUS_TF);
	data->ctrl2 |= RV3032_CTRL2_TIE;
	reg_write(dev, RV3032_CTRL2, data->ctrl2);

	data->ctrl1 |= RV3032_CTRL1_TE;
	data->ctrl1 = (data->ctrl1 & ~RV3032_CTRL1_TD_MSK) | RV3032_CTRL1_TD_1;
	reg_write(dev, RV3032_CTRL1, data->ctrl1);

	k_mutex_unlock(&data->lock);

	return 0;
}

// Set the periodic alarm.
static int set_update(const struct device *dev, const struct counter_alarm_cfg *alarm_cfg)
{
	struct rv3032_data *data = dev->data;

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		return -EINVAL;
	}

	if (alarm_cfg->ticks != 1 && alarm_cfg->ticks != 60) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	// Save alarm callback info
	data->alarm_callback[2] = alarm_cfg->callback;
	data->alarm_user_data[2] = alarm_cfg->user_data;

	if (alarm_cfg->ticks == 1) {
		data->ctrl1 &= ~RV3032_CTRL1_USEL;
	} else {
		data->ctrl1 |= RV3032_CTRL1_USEL;
	}
	reg_write(dev, RV3032_CTRL1, data->ctrl1);

	reg_status_clear(dev, RV3032_STATUS_UF);
	data->ctrl2 |= RV3032_CTRL2_UIE;
	reg_write(dev, RV3032_CTRL2, data->ctrl2);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rv3032_counter_set_alarm(const struct device *dev, uint8_t alarm_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	switch (alarm_id) {
	case 0:
		return set_alarm(dev, alarm_cfg);
	case 1:
		return set_timer(dev, alarm_cfg);
	case 2:
		return set_update(dev, alarm_cfg);
	default:
		return -ENOTSUP;
	}
}

static int rv3032_counter_cancel_alarm(const struct device *dev, uint8_t alarm_id)
{
	struct rv3032_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (alarm_id) {
	case 0:
		data->ctrl2 &= ~RV3032_CTRL2_AIE;
		reg_write(dev, RV3032_CTRL2, data->ctrl2);
		// Turn off alarm registers too? How? Set day to 0? Set all ignore bits?
		break;
	case 1:
		data->ctrl1 &= ~RV3032_CTRL1_TE;
		data->ctrl2 &= ~RV3032_CTRL2_TIE;
		reg_write_n(dev, RV3032_CTRL1, &data->ctrl1, 2);
		break;
	case 2:
		data->ctrl2 &= ~RV3032_CTRL2_UIE;
		reg_write(dev, RV3032_CTRL2, data->ctrl2);
		break;
	default:
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	data->alarm_callback[alarm_id] = NULL;
	data->alarm_user_data[alarm_id] = 0;

	k_mutex_unlock(&data->lock);

	return 0;
}

static uint32_t rv3032_counter_get_pending_int(const struct device *dev)
{
	struct rv3032_data *data = dev->data;
	uint32_t pending = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	int status = reg_read(dev, RV3032_STATUS);
	if (status < 0) {
		LOG_ERR("Unable to read status (%d)", status);
		status = 0;
	}

	if (status & RV3032_STATUS_AF)
		pending |= BIT(0);
	if (status & RV3032_STATUS_TF)
		pending |= BIT(1);
	if (status & RV3032_STATUS_UF)
		pending |= BIT(2);

	// Clear status bits returned
	reg_status_clear(dev, status & (RV3032_STATUS_AF|RV3032_STATUS_TF|RV3032_STATUS_UF));

	k_mutex_unlock(&data->lock);

	return pending;
}

static void rv3032_irq(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct rv3032_data *data = CONTAINER_OF(gpio_cb, struct rv3032_data, irq);

	ARG_UNUSED(pins);

	k_work_submit(&data->irq_work);
}

static void rv3032_work_handler(struct k_work *work)
{
	struct rv3032_data *data = CONTAINER_OF(work, struct rv3032_data, irq_work);
	const struct device *dev = data->dev;
	struct time_regs regs;
	struct tm tm;

	k_mutex_lock(&data->lock, K_FOREVER);

	reg_read_n(dev, RV3032_SEC, &regs.clock.sec, sizeof(regs) - 1);
	int status = reg_read(dev, RV3032_STATUS);

	LOG_DBG("IRQ status %02x", status);
	
	uint8_t clear = 0;
	if (data->alarm_callback[0]) {
		clear |= RV3032_STATUS_AF;
	}
	if (data->alarm_callback[1]) {
		clear |= RV3032_STATUS_TF;
	}
	if (data->alarm_callback[2]) {
		clear |= RV3032_STATUS_UF;
	}
	if (data->sensor_callback) {
		clear |= RV3032_STATUS_THF|RV3032_STATUS_TLF;
	}

	// Clear only status bits we'll handle
	reg_status_clear(dev, status & clear);

	k_mutex_unlock(&data->lock);

	decode_tm(&regs, &tm);
	uint32_t ticks = mktime(&tm);
	// Call handlers
	if ((status & RV3032_STATUS_AF) && data->alarm_callback[0]) {
		data->alarm_callback[0](dev, 0, ticks, data->alarm_user_data[0]);
	}
	if ((status & RV3032_STATUS_TF) && data->alarm_callback[1]) {
		data->alarm_callback[1](dev, 1, ticks, data->alarm_user_data[1]);
	}
	if ((status & RV3032_STATUS_UF) && data->alarm_callback[2]) {
		data->alarm_callback[2](dev, 2, ticks, data->alarm_user_data[2]);
	}
	if ((status & (RV3032_STATUS_THF|RV3032_STATUS_TLF)) && data->sensor_callback) {
		data->sensor_callback(data->sensor_dev, data->sensor_trigger);
	}
}

static int rv3032_init(const struct device *dev)
{
	struct rv3032_data *data = dev->data;
	const struct rv3032_config *cfg = dev->config;
	unsigned eeprom_update = 0;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}
	if (cfg->int_gpio.port != NULL && !device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("GPIO controller %s is not ready", cfg->int_gpio.port->name);
		return -ENODEV;
	}

	uint8_t status;
	ret = i2c_reg_read_byte_dt(&cfg->i2c, RV3032_STATUS, &status);
	if (ret < 0) {
		LOG_ERR("Error reading status register (%d)", ret);
		return ret;
	}
	LOG_DBG("STATUS %02x", status);
	if (status & (RV3032_STATUS_VLF | RV3032_STATUS_PORF)) {
		LOG_INF("Power On Reset or Voltage Low");
		data->unset = true;
	}

	k_mutex_init(&data->lock);
	k_mutex_lock(&data->lock, K_FOREVER);
	data->dev = dev;

	// Initialize ctrl1,2,3 cache
	reg_read_n(dev, RV3032_CTRL1, &data->ctrl1, 3);

	// Disable all interrupts
	data->ctrl2 &= ~RV3032_CTRL2_ALLIE;
	reg_write(dev, RV3032_CTRL2, data->ctrl2);

	/* Disable EEPROM auto-refresh */
	if (!(data->ctrl1 & RV3032_CTRL1_EERD)) {
		data->ctrl1 |= RV3032_CTRL1_EERD;
		reg_write(dev, RV3032_CTRL1, data->ctrl1);
	}

	/* Configure clock out pin */
	if (cfg->clken) {
		uint8_t clkout[2];
		reg_read_n(dev, RV3032_CLKOUT1, clkout, sizeof(clkout));
		LOG_DBG("CLKOUT %02x %02x", clkout[0], clkout[1]);

		if (clkout[0] != cfg->clkout1 || clkout[1] != cfg->clkout2) {
			reg_write_n(dev, RV3032_CLKOUT1, &cfg->clkout1, sizeof(clkout));
			eeprom_update |= BIT(0);
		}
	}
	int pmu = reg_read(dev, RV3032_PMU);
	if (!(pmu & RV3032_PMU_NCLKE) != cfg->clken) {
		pmu ^= RV3032_PMU_NCLKE;
		reg_write(dev, RV3032_PMU, pmu);
		eeprom_update |= BIT(1);
	}

	/* Setup charger config */
	if ((pmu & ~RV3032_PMU_NCLKE) != cfg->pmu) {
		pmu = (pmu & RV3032_PMU_NCLKE) | cfg->pmu;
		reg_write(dev, RV3032_PMU, pmu);
		eeprom_update |= BIT(1);
	}

	if (cfg->int_gpio.port != NULL) {
		k_work_init(&data->irq_work, rv3032_work_handler);
		gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		gpio_init_callback(&data->irq, rv3032_irq, BIT(cfg->int_gpio.pin));
		gpio_add_callback(cfg->int_gpio.port, &data->irq);
		gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}

	if (eeprom_update) {
		LOG_INF("Updating EEPROM:%s%s",
		        eeprom_update & BIT(0) ? " CLKOUT" : "",
		        eeprom_update & BIT(1) ? " PMU" : "");

		if (eeprom_update & BIT(0)) {
			reg_eeprom_write(dev, RV3032_CLKOUT1, cfg->clkout1);
			reg_eeprom_write(dev, RV3032_CLKOUT2, cfg->clkout2);
		}
		if (eeprom_update & BIT(1)) {
			reg_eeprom_write(dev, RV3032_PMU, pmu);
		}
	}

	k_mutex_unlock(&data->lock);

	struct tm tm;
	uint8_t hsec;
	ret = rv3032_rtc_get_time_tm(dev, &tm, &hsec);
	if (ret == 0) {
		LOG_INF("RV3032 RTC: %4d-%02d-%02dT%02d:%02d:%02d.%02d",
			tm.tm_year + 1900, tm.tm_mon+1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, hsec);
	} else if (ret == -ENODATA) {
		LOG_INF("RV3032 RTC: Time not set");
	} else {
		LOG_ERR("Failed to read current time (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct counter_driver_api rv3032_api = {
	.start = rv3032_counter_start,
	.stop = rv3032_counter_stop,
	.get_value = rv3032_counter_get_value,
	.set_alarm = rv3032_counter_set_alarm,
	.cancel_alarm = rv3032_counter_cancel_alarm,
	.set_top_value = rv3032_counter_set_top_value,
	.get_pending_int = rv3032_counter_get_pending_int,
	.get_top_value = rv3032_counter_get_top_value,
};

#include <drivers/eeprom.h>

static int rv3032_child_init(const struct device *dev)
{
	LOG_DBG("child init %s", dev->name);
	return 0;
}

static int rv3032_eeprom_read(const struct device *dev, off_t offset,
                              void *buf, size_t len)
{
	struct rv3032_data *data = dev->data;
	uint8_t *ibuf = buf;
	int ret = 0;

	if (len + offset > 32) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < len && !ret; i++) {
		ret = reg_eeprom_read(dev, RV3032_EEPROM_USER + offset + i, &ibuf[i]);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_eeprom_write(const struct device *dev, off_t offset,
                               const void *buf, size_t len)
{
	struct rv3032_data *data = dev->data;
	const uint8_t *ibuf = buf;
	int ret = 0;

	if (len + offset > 32) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < len && !ret; i++) {
		ret = reg_eeprom_write(dev, RV3032_EEPROM_USER + offset + i, ibuf[i]);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t rv3032_eeprom_size(const struct device *dev)
{
	return 32;
}

static const struct eeprom_driver_api rv3032_eeprom_api __unused = {
	.read = rv3032_eeprom_read,
	.write = rv3032_eeprom_write,
	.size = rv3032_eeprom_size,
};

static int rv3032_nvram_read(const struct device *dev, off_t offset,
                             void *buf, size_t len)
{
	struct rv3032_data *data = dev->data;

	if (len + offset > 16) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = reg_read_n(dev, RV3032_RAM1 + offset, buf, len);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_nvram_write(const struct device *dev, off_t offset,
                              const void *buf, size_t len)
{
	struct rv3032_data *data = dev->data;
	int ret;

	if (len + offset > 16) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = reg_write_n(dev, RV3032_RAM1 + offset, buf, len);
	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t rv3032_nvram_size(const struct device *dev)
{
	return 16;
}

static const struct eeprom_driver_api rv3032_nvram_api __unused = {
	.read = rv3032_nvram_read,
	.write = rv3032_nvram_write,
	.size = rv3032_nvram_size,
};


static int rv3032_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct rv3032_data *data = dev->data;
	uint8_t temp[2];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = reg_read_n(dev, RV3032_TLSB, temp, sizeof(temp));
	if (ret == 0) {
		LOG_DBG("TLSB,TMSB %02x %02x", temp[0], temp[1]);
		data->sensor_temp = (temp[0] >> 4) | (temp[1] << 4);
		// Sign-extend 12 bit value
		data->sensor_temp = (data->sensor_temp ^ 0x800) - 0x800;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
	struct rv3032_data *data = dev->data;

	switch ((int)chan) {
	case SENSOR_CHAN_DIE_TEMP:
		k_mutex_lock(&data->lock, K_FOREVER);
		val->val1 = data->sensor_temp / 16;
		val->val2 = (data->sensor_temp % 16) * 62500;
		k_mutex_unlock(&data->lock);
		break;
	case SENSOR_CHAN_RV3032_TLOWER_TS:
	case SENSOR_CHAN_RV3032_TUPPER_TS:
	{
		bool lower = (int)chan == SENSOR_CHAN_RV3032_TLOWER_TS;
		struct {
			uint8_t count;
			uint8_t sec, min, hour;
			uint8_t day, month, year;
		} regs;
		struct tm tm;

		k_mutex_lock(&data->lock, K_FOREVER);
		reg_read_n(dev, lower ? RV3032_TSLOW_COUNT : RV3032_TSHIGH_COUNT,
			   (uint8_t*)&regs, sizeof(regs));
		reg_write(dev, RV3032_TS_CTRL, lower ? RV3032_TS_CTRL_TLR : RV3032_TS_CTRL_THR);
		k_mutex_unlock(&data->lock);

		if (regs.day == 0) { // Timestamp not set
			return -ENODATA;
		}

		struct time_regs tregs = {
			.clock.sec = regs.sec,
			.clock.min = regs.min,
			.clock.hour = regs.hour,
			.cal.day = regs.day,
			.cal.month = regs.month,
			.cal.year = regs.year,
		};
		decode_tm(&tregs, &tm);
		val->val1 = mktime(&tm);
		val->val2 = regs.count;
		break;
	}
	default:
                return -ENOTSUP;
        }

        return 0;
}

static int rv3032_attr_set(const struct device *dev, enum sensor_channel chan,
		           enum sensor_attribute attr, const struct sensor_value *val)
{
	struct rv3032_data *data = dev->data;
	int ret = 0;

        if (chan != SENSOR_CHAN_DIE_TEMP) {
                return -ENOTSUP;
        }

	switch (attr) {
	case SENSOR_ATTR_LOWER_THRESH:
	case SENSOR_ATTR_UPPER_THRESH:
		if (val->val1 < -128 || val->val1 > 127) {
			return -ERANGE;
		}
		k_mutex_lock(&data->lock, K_FOREVER);
		ret = reg_write(dev, attr == SENSOR_ATTR_LOWER_THRESH ?
			             RV3032_TLOW : RV3032_THIGH, (int8_t)val->val1);
		k_mutex_unlock(&data->lock);
		break;
	case SENSOR_ATTR_ALERT:
	case SENSOR_ATTR_OFFSET:
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static int rv3032_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	struct rv3032_data *data = dev->data;
	int ret = 0;

        if (chan != SENSOR_CHAN_DIE_TEMP) {
                return -ENOTSUP;
        }

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = 1;
		val->val2 = 0;
		break;
	case SENSOR_ATTR_LOWER_THRESH:
	case SENSOR_ATTR_UPPER_THRESH:
		k_mutex_lock(&data->lock, K_FOREVER);
		ret = reg_read(dev, attr == SENSOR_ATTR_LOWER_THRESH ?
				    RV3032_TLOW : RV3032_THIGH);
		k_mutex_unlock(&data->lock);
		LOG_DBG("TLOW|HIGH %02x", ret);
		if (ret >= 0) {
			val->val1 = (int8_t)ret;
			val->val2 = 0;
		}
		break;
	case SENSOR_ATTR_ALERT:
		break;
	case SENSOR_ATTR_OFFSET:
	{
		uint8_t tref[2];

		k_mutex_lock(&data->lock, K_FOREVER);
		ret = reg_read_n(dev, RV3032_TREF0, tref, sizeof(tref));
		k_mutex_unlock(&data->lock);
		LOG_DBG("TREF %02x %02x", tref[0], tref[1]);
		if (ret >= 0) {
			int t = (int16_t)(tref[0] | (tref[1] << 8)) - 64;
			val->val1 = t / 128;
			val->val2 = ((t % 128) * 15625) / 2;
		}
		break;
	}
	default:
		return -ENOTSUP;
	}

	return ret;
}

static int rv3032_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	struct rv3032_data *data = dev->data;
	const uint8_t tflags = RV3032_STATUS_THF | RV3032_STATUS_TLF;
	int ret;

        if (trig->chan != SENSOR_CHAN_DIE_TEMP) {
                return -ENOTSUP;
        }
	if (trig->type != SENSOR_TRIG_THRESHOLD) {
                return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->sensor_callback = handler;
	data->sensor_trigger = trig;

	// Trigger if thigh/tlow has already happened
	int status = reg_read(dev, RV3032_STATUS);
	if (status < 0) {
		k_mutex_unlock(&data->lock);
		return status;
	}
	
	if (status & tflags) {
		LOG_DBG("Pending temperature event when trigger enabled (%02x)", status);
		k_work_submit(&data->irq_work);
	}

	// Enable events and interrupt, might already be enabled
	data->ctrl3 |= RV3032_CTRL3_THE | RV3032_CTRL3_THIE |
		       RV3032_CTRL3_TLE | RV3032_CTRL3_TLIE;
 	ret = reg_write(dev, RV3032_CTRL3, data->ctrl3);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int rv3032_sensor_init(const struct device *dev)
{
	struct rv3032_data *data = dev->data;

	rv3032_child_init(dev);
	data->sensor_dev = dev;

	return 0;
}

static const struct sensor_driver_api rv3032_sensor_api __unused = {
	.attr_set = rv3032_attr_set,
	.attr_get = rv3032_attr_get,
	.sample_fetch = rv3032_sample_fetch,
	.channel_get = rv3032_channel_get,
	.trigger_set = rv3032_trigger_set,
};

#ifndef DT_INST_CHILD
#define DT_INST_CHILD(inst, child) DT_CHILD(DT_DRV_INST(inst), child)
#endif

// Value of HFD field, derived from frequency in Hz
#define HFD_VAL(inst)	(DT_INST_PROP_OR(inst, clkout_hf_frequency, 8192u) / 8192u - 1u)

// clkout2 EEPROM value, based on clkout frequency and mode
#define CLKOUT2(inst) (									\
	(DT_INST_NODE_HAS_PROP(inst, clkout_hf_frequency) ? RV3032_CLKOUT2_OS : 0) |	\
	(DT_INST_ENUM_IDX_OR(inst, clkout_xtal_frequency, 0) << RV3032_CLKOUT2_FD_SHIFT) | \
	((HFD_VAL(inst) >> 8) & RV3032_CLKOUT2_HFD_MSK))

// PMU EEPROM value, except for clkout disable bit
#define PMU(inst) (									\
	(DT_INST_ENUM_IDX(inst, backup_mode) << RV3032_PMU_BSM_SHIFT) |			\
	(DT_INST_ENUM_IDX(inst, trickle_resistor_ohms) << RV3032_PMU_TCR_SHIFT) |	\
	(DT_INST_PROP(inst, trickle_enable) ?						\
		(DT_INST_ENUM_IDX(inst, backup_mode) == 1 ? 0x1 :			\
			DT_INST_ENUM_IDX_OR(inst, trickle_voltage_millivolt, -1) + 1)	\
		: 0) )

// Create a child device, for the node "node", with parent device instance "inst", using
// the specified init and api pointers.  The data and config will be the same objects
// used for the parent inst.
#define _INST_CHILD(node, inst, init, api)						\
	DEVICE_DT_DEFINE(node, init, NULL,						\
		&rv3032_data_##inst,							\
		&rv3032_config_##inst,							\
		POST_KERNEL,								\
		CONFIG_COUNTER_INIT_PRIORITY,						\
		&api)

#define MATCH_CHILD(node, inst, child, init, api)					\
		COND_CODE_1(DT_NODE_HAS_COMPAT(node, microcrystal_rv3032_##child),	\
			(_INST_CHILD(node, inst, init, api)), ())

// Create an EEPROM, NVRAM, or SENSOR device.  Expands to nothing if the child node
// isn't present.
#define INST_CHILD(inst, child, init, api)						\
	DT_INST_FOREACH_CHILD_VARGS(inst, MATCH_CHILD, inst, child, init, api)

#define INST_DT_RV3032(inst)								\
	static struct rv3032_data rv3032_data_##inst;					\
	static const struct rv3032_config rv3032_config_##inst = {			\
		.generic = {								\
			.max_top_value = UINT32_MAX,					\
			.freq = 1,							\
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,				\
			.channels = 3,							\
		},									\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),		\
		.evi_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, evi_gpios, {0}),		\
		.clken = DT_INST_NODE_HAS_PROP(inst, clkout_hf_frequency) || 		\
		         DT_INST_NODE_HAS_PROP(inst, clkout_xtal_frequency),		\
		.pmu = PMU(inst),							\
		.clkout1 = HFD_VAL(inst) & 0xff,					\
		.clkout2 = CLKOUT2(inst),						\
	};										\
											\
	INST_CHILD(inst, eeprom, rv3032_child_init, rv3032_eeprom_api);			\
	INST_CHILD(inst, nvram, rv3032_child_init, rv3032_nvram_api);			\
	INST_CHILD(inst, sensor, rv3032_sensor_init, rv3032_sensor_api);		\
	DEVICE_DT_INST_DEFINE(inst, rv3032_init, NULL,					\
		    &rv3032_data_##inst,						\
		    &rv3032_config_##inst,						\
		    POST_KERNEL,							\
		    CONFIG_COUNTER_INIT_PRIORITY,					\
		    &rv3032_api);

DT_INST_FOREACH_STATUS_OKAY(INST_DT_RV3032);
