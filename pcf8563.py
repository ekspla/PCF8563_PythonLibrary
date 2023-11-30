'''
NOV. 2023, modified by ekspla.

The code was modified to read/write mutiple date & time registers at once.

CAUTION:
  From the datasheets and application notes (e.g. AN10652), it is clear that 
the date/time registers should be read/written in one single access (e.g. 
before stop) and within 1 second for internal watchdog compensation to work; 
the compensation is not useful for the lost two 1-Hz ticks.

NOTE:
  The definitions of weekday in machine.RTC depends on ports; e.g. a range 
of [0,6] from MON to SUN in ESP32 while [1,7] in STM32 (Pyboard).  Adjust the 
range appropriately.


MIT License

Copyright (c) 2019 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

pcf8563.py - MicroPython library for NXP PCF8563 Real-time clock/calendar
Created by Lewis he on September 17, 2019.
github:https://github.com/lewisxhe/PCF8563_PythonLibrary
'''
import time
from micropython import const

_SLAVE_ADDRESS = const(0x51)

_STAT1_REG = const(0x00)
_STAT2_REG = const(0x01)

_SEC_REG = _DATETIME_REG = const(0x02)
_MIN_REG = const(0x03)
_HR_REG = const(0x04)
_DATE_REG = const(0x05)
_WEEKDAY_REG = const(0x06)
_MONTH_REG = const(0x07)
_YEAR_REG = const(0x08)

_ALARM_MINUTES_REG = _ALARMS_REG = const(0x09)
_ALARM_HOURS_REG = const(0x0A)
_ALARM_DATE_REG = const(0x0B)
_ALARM_WEEKDAY_REG = const(0x0C)

_SQW_REG = const(0x0D)

_TIMER1_REG = _TIMERS_REG = const(0x0E)
_TIMER2_REG = const(0x0F)

_VOL_LOW_MASK = const(0x80)
_minuteS_MASK = const(0x7F)
_HOUR_MASK = const(0x3F)
_WEEKDAY_MASK = const(0x07)
_YEAR_MASK = const(0xff)
_CENTURY_MASK = const(0x80)
_DATE_MASK = const(0x3F)
_MONTH_MASK = const(0x1F)
_TIMER_CTL_MASK = const(0x03)

_STOP = const(0x20)
_ALARM_AF = const(0x08)
_TIMER_TF = const(0x04)
_ALARM_AIE = const(0x02)
_TIMER_TIE = const(0x01)
_TIMER_TE = const(0x80)
_TIMER_TD10 = const(0x03)
_NO_ALARM = const(0xFF)
_ALARM_ENABLE = const(0x80)
_CLK_ENABLE = const(0x80)

CLK_OUT_FREQ_32_DOT_768KHZ = const(0x80)
CLK_OUT_FREQ_1_DOT_024KHZ = const(0x81)
CLK_OUT_FREQ_32_HZ = const(0x82)
CLK_OUT_FREQ_1_HZ = const(0x83)
CLK_HIGH_IMPEDANCE = const(0x0)


class PCF8563:
    def __init__(self, i2c, address=_SLAVE_ADDRESS, en_subsecond=False):
        """Initialization needs to be given an initialized I2C port
        """
        self.i2c = i2c
        self.address = address
        self._buffer = bytearray(16)
        self._bytebuf = bytearray(1)
        self._mv = memoryview(self._buffer)
        self._mv_datetime = self._mv[_SEC_REG:_YEAR_REG + 1]
        self._mv_timers = self._mv[_TIMER1_REG:_TIMER2_REG + 1]
        self._mv_alarms = self._mv[_ALARM_MINUTES_REG:_ALARM_WEEKDAY_REG + 1]
        self._DATETIME_MASK = bytes((
            _minuteS_MASK, 
            _minuteS_MASK, 
            _HOUR_MASK, 
            _DATE_MASK, 
            _WEEKDAY_MASK, 
            _MONTH_MASK, 
            _YEAR_MASK))
        self._en_subsecond = en_subsecond

    def __write_byte(self, reg, val):
        self._bytebuf[0] = val & 0xff
        self.__write_bytes(reg, self._bytebuf)

    def __read_byte(self, reg):
        self.__read_bytes(reg, self._bytebuf)
        return self._bytebuf[0]

    def __write_bytes(self, reg, buffer):
        self.i2c.writeto_mem(self.address, reg, buffer)

    def __read_bytes(self, reg, buffer):
        self.i2c.readfrom_mem_into(self.address, reg, buffer)

    def __bcd2dec(self, bcd):
        return (((bcd & 0xf0) >> 4) * 10 + (bcd & 0x0f))

    def __dec2bcd(self, dec):
        tens, units = divmod(dec, 10)
        return (tens << 4) + units

    def __get_weekday(self, date, month, year):
        if month < 3:
            month += 12
            year -= 1
        weekday = (
            (-1 + date + (13 * month + 8) // 5 + year + year // 4 
            - year // 100 + year // 400)
            % 7)
        return weekday

    def datetime(self):
        """Return a tuple such as (year, month, date, weekday, hours, minutes,
        seconds, subseconds[63,0]).
        """
        if self._en_subsecond:
            self.__read_bytes(_TIMERS_REG, self._mv_timers)
        self.__read_bytes(_DATETIME_REG, self._mv_datetime)

        seconds, minutes, hours, date, weekday, month, year = (
            self.__bcd2dec(a & b) for a, b in zip(
            self._mv_datetime, self._DATETIME_MASK))

        if self._en_subsecond:
            return (year, month, date, weekday, hours, minutes, seconds, 
                self._buffer[_TIMER2_REG] & 0x7f)
        return (year, month, date, weekday, hours, minutes, seconds)

    def write_all(self, seconds=None, minutes=None, hours=None, weekday=None,
                  date=None, month=None, year=None):
        """Direct write un-none value.
        Range: seconds [0,59], minutes [0,59], hours [0,23],
               weekday [0,6], date [1,31], month [1,12], year [0,99].
        """
        if (seconds is None) or seconds < 0 or seconds > 59:
            raise ValueError('Seconds is out of range [0,59].')
        if (minutes is None) or minutes < 0 or minutes > 59:
            raise ValueError('Minutes is out of range [0,59].')
        if (hours is None) or hours < 0 or hours > 23:
            raise ValueError('Hours is out of range [0,23].')
        if (date is None) or date < 1 or date > 31:
            raise ValueError('Date is out of range [1,31].')
        if (month is None) or month < 1 or month > 12:
            raise ValueError('Month is out of range [1,12].')
        if (year is None) or year < 0 or year > 99:
            raise ValueError('Years is out of range [0,99].')
        if weekday is None:
            weekday = self.__get_weekday(date, month, year + 2000)
        elif weekday < 0 or weekday > 6:
            raise ValueError('Day is out of range [0,6].')

        self._buffer[_SEC_REG] = self.__dec2bcd(seconds)
        self._buffer[_MIN_REG] = self.__dec2bcd(minutes)
        self._buffer[_HR_REG] = self.__dec2bcd(hours) # no 12 hour mode
        self._buffer[_DATE_REG] = self.__dec2bcd(date)
        self._buffer[_WEEKDAY_REG] = self.__dec2bcd(weekday)
        self._buffer[_MONTH_REG] = self.__dec2bcd(month)
        self._buffer[_YEAR_REG] = self.__dec2bcd(year)

        if self._en_subsecond:
            self.stop()
            self.__write_bytes(_DATETIME_REG, self._mv_datetime)
            self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG) & ~_TIMER_TIE
            self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG]) # TIE disable
            self._buffer[_TIMER1_REG] = _TIMER_TE | b'\01' # TE enable, source clk 64 Hz
            self._buffer[_TIMER2_REG] = b'\x40' # Set Countdown Timer to 64
            self.__write_bytes(_TIMERS_REG, self._mv_timers)
            self.start()

        else:
            # STOP/START not necessary because of the compensation by internal watchdog.
            self.__write_bytes(_DATETIME_REG, self._mv_datetime)

    def stop(self):
        """Stop divided clocks below 4096 Hz in frequency"""
        self._buffer[_STAT1_REG] = self.__read_byte(_STAT1_REG) | _STOP
        self.__write_byte(_STAT1_REG, self._buffer[_STAT1_REG])

    def start(self):
        """Start divided clocks below 4096 Hz in frequency"""
        self._buffer[_STAT1_REG] = self.__read_byte(_STAT1_REG) & ~_STOP
        self.__write_byte(_STAT1_REG, self._buffer[_STAT1_REG])

    def set_datetime(self, dt):
        """Input a tuple such as (year, month, date, weekday, hours, minutes,
        seconds).
        """
        self.write_all(dt[5], dt[4], dt[3],
                       dt[6], dt[2], dt[1], dt[0] % 100)

    def write_now(self):
        """Write the current system time to PCF8563
        """
        self.set_datetime(time.localtime())

    def set_clk_out_frequency(self, frequency=CLK_OUT_FREQ_1_HZ):
        """Set the clock output pin frequency
        """
        self._buffer[_SQW_REG] = frequency
        self.__write_byte(_SQW_REG, self._buffer[_SQW_REG])

    def check_if_alarm_on(self):
        """Read the register to get the alarm enabled
        """
        return bool(self.__read_byte(_STAT2_REG) & _ALARM_AF)

    def turn_alarm_off(self):
        """Should not affect the alarm interrupt state.
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG) & ~_ALARM_AF
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def clear_alarm(self):
        """Clear status register.
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG)
        self._buffer[_STAT2_REG] &= ~_ALARM_AF
        self._buffer[_STAT2_REG] |= _TIMER_TF

        self._buffer[_ALARM_MINUTES_REG] = self._buffer[_ALARM_HOURS_REG] \
            = self._buffer[_ALARM_DATE_REG] = self._buffer[_ALARM_WEEKDAY_REG] \
            = _ALARM_ENABLE

        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])
        self.__write_bytes(_ALARMS_REG, self._mv_alarms)

    def check_for_alarm_interrupt(self):
        """check for alarm interrupt,is alram int return True
        """
        return bool(self.__read_byte(_STAT2_REG) & _ALARM_AIE)

    def enable_alarm_interrupt(self):
        """Turn on the alarm interrupt output to the interrupt pin
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG)
        self._buffer[_STAT2_REG] &= ~_ALARM_AF
        self._buffer[_STAT2_REG] |= (_TIMER_TF | _ALARM_AIE)
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def disable_alarm_interrupt(self):
        """Turn off the alarm interrupt output to the interrupt pin
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG)
        self._buffer[_STAT2_REG] &= ~(_ALARM_AF | _ALARM_AIE)
        self._buffer[_STAT2_REG] |= _TIMER_TF
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def set_daily_alarm(self, hours=None, minutes=None, date=None, weekday=None):
        """Set alarm match, allow sometimes, minute, day, week
        """
        if minutes is None:
            self._buffer[_ALARM_MINUTES_REG] = _ALARM_ENABLE
        else:
            if minutes < 0 or minutes > 59:
                raise ValueError('Minutes is out of range [0,59].')
            self._buffer[_ALARM_MINUTES_REG] = self.__dec2bcd(minutes) & _minuteS_MASK

        if hours is None:
            self._buffer[_ALARM_HOURS_REG] = _ALARM_ENABLE
        else:
            if hours < 0 or hours > 23:
                raise ValueError('Hours is out of range [0,23].')
            self._buffer[_ALARM_HOURS_REG] = self.__dec2bcd(hours) & _HOUR_MASK

        if date is None:
            self._buffer[_ALARM_DATE_REG] = _ALARM_ENABLE
        else:
            if date < 1 or date > 31:
                raise ValueError('date is out of range [1,31].')
            self._buffer[_ALARM_DATE_REG] = self.__dec2bcd(date) & _DATE_MASK

        if weekday is None:
            self._buffer[_ALARM_WEEKDAY_REG] = _ALARM_ENABLE
        else:
            if weekday < 0 or weekday > 6:
                raise ValueError('weekday is out of range [0,6].')
            self._buffer[_ALARM_WEEKDAY_REG] = self.__dec2bcd(weekday) & _WEEKDAY_MASK

        self.__write_bytes(_ALARMS_REG, self._mv_alarms)
