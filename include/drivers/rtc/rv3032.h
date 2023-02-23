/* Copyright (c) 2023 Igor Institute, Inc.
 * Author: Trent Piepho <trent.piepho@igorinstitute.com>
 *
 * Extended API for RV3032 */
#pragma once

#include <drivers/sensor.h>
#include <time.h>

enum sensor_channel_rv3032 {
        /** Timestamp of lower threshold event, as time_t */
        SENSOR_CHAN_RV3032_TLOWER_TS = SENSOR_CHAN_PRIV_START,
        /** Timestamp of lower threshold event, as time_t */
        SENSOR_CHAN_RV3032_TUPPER_TS,
};

#ifdef CONFIG_COUNTER_MICROCRYSTAL_RV3032

extern int rv3032_rtc_set_time(const struct device *dev, time_t unix_time);
extern int rv3032_rtc_set_time_tm(const struct device *dev, const struct tm*);

extern int rv3032_rtc_get_time(const struct device *dev, time_t *unix_time);
extern int rv3032_rtc_get_time_tm(const struct device *dev, struct tm*, uint8_t *hsec);
extern int rv3032_rtc_get_time_ts(const struct device *dev, struct timespec *);

#else

static inline int rv3032_rtc_set_time(const struct device *dev, time_t unix_time) { return -ENOSYS; }
static inline int rv3032_rtc_set_time_tm(const struct device *dev, const struct tm *tm) { return -ENOSYS; }
static inline int rv3032_rtc_get_time(const struct device *dev, time_t *unix_time) { return -ENOSYS; }
static inline int rv3032_rtc_get_time_tm(const struct device *dev, struct tm *tm, uint8_t *hsec) { return -ENOSYS; }
static inline int rv3032_rtc_get_time_ts(const struct device *dev, struct timespec *ts) { return -ENOSYS; }

#endif
