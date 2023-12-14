'''
DEC. 2023, the modified PCF8563 code was adapted to PCF2129 by ekspla.
Functions such as timers, watchdogs, timestamps, 12 hour mode, etc. not supported.


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
_STAT3_REG = const(0x02)

_SEC_REG = _DATETIME_REG = const(0x03)
_MIN_REG = const(0x04)
_HR_REG = const(0x05)
_DATE_REG = const(0x06)
_WEEKDAY_REG = const(0x07)
_MONTH_REG = const(0x08)
_YEAR_REG = const(0x09)

_ALARM_SECONDS_REG = _ALARMS_REG = const(0x0A)
_ALARM_MINUTES_REG = const(0x0B)
_ALARM_HOURS_REG = const(0x0C)
_ALARM_DATE_REG = const(0x0D)
_ALARM_WEEKDAY_REG = const(0x0E)

_SQW_REG = const(0x0F)

_TIMER1_REG = _TIMERS_REG = const(0x10)
_TIMER2_REG = const(0x11)

# From 0x12 to 0x18: timestamp registers 

_AGING_REG = const(0x19)  # From 0 (+8) to 15 (-7 ppm); default is 8 (0 ppm).

_OSC_STOP_FLAG = const(0x80)
_minuteS_MASK = const(0x7F)
_HOUR_MASK = const(0x3F)
_WEEKDAY_MASK = const(0x07)
_YEAR_MASK = const(0xff)
_DATE_MASK = const(0x3F)
_MONTH_MASK = const(0x1F)

_STOP = const(0x20)
_POR_OVRD = const(0x08)
#_H12 = const(0x04)  # 12 hour (am./pm.) mode currently not supported.
_MI = const(0x02)
_SI = const(0x01)

_MSF = const(0x80)
_WDTF = const(0x40)
_TSF2 = const(0x20)
_ALARM_AF = const(0x10)
_TIMESTAMP_TIE = const(0x04)
_ALARM_AIE = const(0x02)

_BATTERY_LOW_BLF = const(0x04)
_BATTERY_LOW_INT_BLIE = const(0x01)

_ALARM_ENABLE = const(0x80)

_TCR_MASK = const(0xC0)
_OTP = const(0x20)
CLK_OUT_FREQ_32_DOT_768KHZ = const(0x00)  # Default
CLK_OUT_FREQ_16_DOT_384KHZ = const(0x01)
CLK_OUT_FREQ_8_DOT_192KHZ = const(0x02)
CLK_OUT_FREQ_4_DOT_096KHZ = const(0x03)
CLK_OUT_FREQ_2_DOT_048KHZ = const(0x04)
CLK_OUT_FREQ_1_DOT_024KHZ = const(0x05)
CLK_OUT_FREQ_1_HZ = const(0x06)
CLK_HIGH_IMPEDANCE = const(0x07)

_TI_TP = const(0x20)

class PCF2129:
    def __init__(self, i2c, address=_SLAVE_ADDRESS):
        """Initialization needs to be given an initialized I2C port
        """
        self.i2c = i2c
        self.address = address
        self._buffer = bytearray(_AGING_REG - _STAT1_REG + 1)
        self._bytebuf = bytearray(1)
        self._mv = memoryview(self._buffer)
        self._mv_datetime = self._mv[_SEC_REG:_YEAR_REG + 1]
        self._mv_timers = self._mv[_TIMER1_REG:_TIMER2_REG + 1]
        self._mv_alarms = self._mv[_ALARM_SECONDS_REG:_ALARM_WEEKDAY_REG + 1]
        self._DATETIME_MASK = bytes((
            _minuteS_MASK, 
            _minuteS_MASK, 
            _HOUR_MASK, 
            _DATE_MASK, 
            _WEEKDAY_MASK, 
            _MONTH_MASK, 
            _YEAR_MASK))
        if self.check_osc_stop():
            self.otp_refresh()
        self.set_clk_out_frequency(CLK_HIGH_IMPEDANCE)
        self.__poweron_reset_override(False)

    def __write_byte(self, reg, val):
        self._bytebuf[0] = val & 0xff
        self.__write_bytes(reg, self._bytebuf)

    def __read_byte(self, reg):
        self.__read_bytes(reg, self._bytebuf)
        return self._bytebuf[0]

    def __write_bytes(self, reg, buffer):
        self.i2c.writeto_mem(self.address, reg, buffer)

    def __read_bytes(self, reg, buffer):
        #self.i2c.readfrom_mem_into(self.address, reg, buffer)  # This does not work on PCF2129.
        self._bytebuf[0] = reg
        self.i2c.writeto(self.address, self._bytebuf)
        self.i2c.readfrom_into(self.address, buffer)

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

    def __poweron_reset_override(self, flag=False):
        self._buffer[_STAT1_REG] = self.__read_byte(_STAT1_REG)
        if flag:
            self._buffer[_STAT1_REG] |= _POR_OVRD
        else:
            self._buffer[_STAT1_REG] &= ~_POR_OVRD
        self.__write_byte(_STAT1_REG, self._buffer[_STAT1_REG])

    def datetime(self):
        """Return a tuple such as (year, month, date, weekday, hours, minutes,
        seconds, subseconds[63,0]).
        """
        self.__read_bytes(_DATETIME_REG, self._mv_datetime)

        seconds, minutes, hours, date, weekday, month, year = (
            self.__bcd2dec(a & b) for a, b in zip(
            self._mv_datetime, self._DATETIME_MASK))

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
        self._buffer[_HR_REG] = self.__dec2bcd(hours) # 12 hour mode currently not supported
        self._buffer[_DATE_REG] = self.__dec2bcd(date)
        self._buffer[_WEEKDAY_REG] = self.__dec2bcd(weekday)
        self._buffer[_MONTH_REG] = self.__dec2bcd(month)
        self._buffer[_YEAR_REG] = self.__dec2bcd(year)

        self.__write_bytes(_DATETIME_REG, self._mv_datetime)

    def temp_control(self, value=None):
        """Read/Set temperature control register, from 0 (4 min) to 3 (0.5 min); default is 0 (4 min).
        """
        self._buffer[_SQW_REG] = self.__read_byte(_SQW_REG)
        if value is None:
            return (self._buffer[_SQW_REG] & _TCR_MASK) >> 6
        self._buffer[_SQW_REG] &= ~_TCR_MASK
        self._buffer[_SQW_REG] |= ((value & 0x03) << 6)
        self.__write_byte(_SQW_REG, self._buffer[_SQW_REG])

    def check_osc_stop(self):
        return bool(self.__read_byte(_SEC_REG) & _OSC_STOP_FLAG)

    def otp_refresh(self):
        self._buffer[_SQW_REG] = self.__read_byte(_SQW_REG) & ~_OTP
        self.__write_byte(_SQW_REG, self._buffer[_SQW_REG])
        self._buffer[_SQW_REG] = self.__read_byte(_SQW_REG) | _OTP
        self.__write_byte(_SQW_REG, self._buffer[_SQW_REG])

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
        """Write the current system time to PCF2129
        """
        self.set_datetime(time.localtime())

    def minute_int(self, flag=None, pulsed=True):
        """Set/reset minute interrupt
        """
        return self.__mi_si(_MI, flag, pulsed)

    def second_int(self, flag=None, pulsed=True):
        """Set/reset second interrupt
        """
        return self.__mi_si(_SI, flag, pulsed)

    def __mi_si(self, mi_si_bit, flag, pulsed):
        self._buffer[_STAT1_REG] = self.__read_byte(_STAT1_REG)
        if flag is None:
            return bool(self._buffer[_STAT1_REG] & mi_si_bit)
        elif flag:
            self._buffer[_STAT1_REG] |= mi_si_bit
        else:
            self._buffer[_STAT1_REG] &= ~mi_si_bit
        self.pulsed_mi_si(flag=pulsed)
        self.minute_second_flag(False)  # Clear MSF, maybe useful for polling
        self.__write_byte(_STAT1_REG, self._buffer[_STAT1_REG])

    def minute_second_flag(self, flag=None):
        """Read/clear the minute second interrupt flag
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG)
        if flag is None:
            return bool(self._buffer[_STAT2_REG] & _MSF)
        self._buffer[_STAT2_REG] &= ~_MSF
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def pulsed_mi_si(self, flag=None):
        """Read/set/reset TI_TP flag
        
        INT is pulsed/permanent for True/False.
        """
        self._buffer[_TIMER1_REG] = self.__read_byte(_TIMER1_REG)
        if flag is None:
            return bool(self._buffer[_TIMER1_REG] & _TI_TP)
        elif flag:
            self._buffer[_TIMER1_REG] |= _TI_TP
        else:
            self._buffer[_TIMER1_REG] &= ~_TI_TP
        self.__write_byte(_TIMER1_REG, self._buffer[_TIMER1_REG])

    def set_clk_out_frequency(self, frequency=CLK_OUT_FREQ_1_HZ):
        """Set the clock output pin frequency
        """
        self._buffer[_SQW_REG] = frequency
        self.__write_byte(_SQW_REG, self._buffer[_SQW_REG])

    def aging_offset(self, value=None):
        """Read/Set clock offset, from 0 (+8 ppm) to 15 (-7 ppm); default is 8 (0 ppm).
        """
        if value is None:
            self._buffer[_AGING_REG] = self.__read_byte(_AGING_REG)
            return self._buffer[_AGING_REG]
        self._buffer[_AGING_REG] = value & 0x0F
        self.__write_byte(_AGING_REG, self._buffer[_AGING_REG])

    def battery_low_flag(self):
        """Read battery low flag (BLF)
        """
        self._buffer[_STAT3_REG] = self.__read_byte(_STAT3_REG)
        return bool(self._buffer[_STAT3_REG] & _BATTERY_LOW_BLF)

    def battery_low_int(self, value=None):
        """Read/Set battery low interrupt (BLIE)
        """
        if value is None:
            return bool(self.__read_byte(_STAT3_REG) & _BATTERY_LOW_INT_BLIE)
        elif value:
            self._buffer[_STAT3_REG] = self.__read_byte(_STAT3_REG) | _BATTERY_LOW_INT_BLIE
        else:
            self._buffer[_STAT3_REG] = self.__read_byte(_STAT3_REG) & ~_BATTERY_LOW_INT_BLIE
        self.__write_byte(_STAT3_REG, self._buffer[_STAT3_REG])

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
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG) & ~_ALARM_AF

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
        self._buffer[_STAT2_REG] |= _ALARM_AIE
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def disable_alarm_interrupt(self):
        """Turn off the alarm interrupt output to the interrupt pin
        """
        self._buffer[_STAT2_REG] = self.__read_byte(_STAT2_REG)
        self._buffer[_STAT2_REG] &= ~(_ALARM_AF | _ALARM_AIE)
        self.__write_byte(_STAT2_REG, self._buffer[_STAT2_REG])

    def set_daily_alarm(self, hours=None, minutes=None, seconds=None, date=None, weekday=None):
        """Set alarm match, allow sometimes, minute, day, week
        """
        if seconds is None:
            self._buffer[_ALARM_SECONDS_REG] = _ALARM_ENABLE
        else:
            if seconds < 0 or seconds > 59:
                raise ValueError('Seconds is out of range [0,59].')
            self._buffer[_ALARM_SECONDS_REG] = self.__dec2bcd(seconds) & _minuteS_MASK

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
