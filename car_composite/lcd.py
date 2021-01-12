'''
   Copyright 2021 Kerry Johnson

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
'''
import platform
import os
import rclpy.node
import rcl_interfaces.msg
import time
from car_composite import utils

class ROS_Lcd(rclpy.node.Node):

    DEFAULT_ROTATE_RATE_S   = 0.2
    DEFAULT_SCROLL_RATE_S   = 5.0

    def __init__(self, address=0x27, rows=4, cols = 20):
        super(ROS_Lcd, self).__init__('ROS_lcd')
        '''
        Constructor
        '''
        self.declare_parameter("use_mock_lcd",
                               False,
                               rcl_interfaces.msg.ParameterDescriptor(type         = rcl_interfaces.msg.ParameterType.PARAMETER_BOOL,
                                                                      description  = 'True LCD can only be used on the SoC.  Otherwise, a mock is used.'))
        
        self.declare_parameter('rotate_rate_ms',
                               ROS_Lcd.DEFAULT_ROTATE_RATE_S,
                               rcl_interfaces.msg.ParameterDescriptor(type         = rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                                                                      description  = 'LCD marquee update rate in fractional seconds.  Default: {d}s'.format(d = ROS_Lcd.DEFAULT_ROTATE_RATE_S)))

        self.declare_parameter('scroll_rate_ms',
                               ROS_Lcd.DEFAULT_SCROLL_RATE_S,
                               rcl_interfaces.msg.ParameterDescriptor(type         = rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE,
                                                                      description  = 'LCD scroll update rate in fractional seconds.  Default: {d}s'.format(d = ROS_Lcd.DEFAULT_SCROLL_RATE_S)))

        lcd_constructor = _PCF8574_LCD if not self.get_parameter('use_mock_lcd').get_parameter_value().bool_value else _TerminalLCD
        self.device     = lcd_constructor(address = address, cols = cols, rows = rows)
        self.timer      = self.create_timer(self.get_parameter('rotate_rate_ms').get_parameter_value().double_value,
                                            self.tick)
        
        self.device.set_fixed_lines(1)
        self.device.set_scroll_rate(self.get_parameter('scroll_rate_ms').get_parameter_value().double_value)
        self.device.show_welcome()

    def on(self):
        self.device.on()
        
    def off(self):
        self.device.off()

    def clear(self):
        self.device.clear()
        
    def write_line(self, row, value):
        self.device.write_line(row, value)
        
    def tick(self):
        self.device.tick()
        

class _LCD(object):
    RIGHT_ALIGN     = '>'
    CENTER_ALIGN    = '^'
    LEFT_ALIGN      = '<'
    
    def __init__(self, cols = 20, rows = 2):
        self.rows           = rows
        self.cols           = cols
        self.buffer         = []
        self.num_fixed      = 1
        self.scroll_rate    = 0
        self.scroll_ts      = time.time()
        
        self.clear()
    
    def show_welcome(self):
        self.write('^{version}'.format(version = os.environ.get('ROS_DONKEY_VERSION', 'development')))
        self.write('OS   {os}'.format(os = platform.system()))
        self.write('Mach {mt}'.format(mt = platform.machine()))
        self.write('IP   {ip}'.format(ip = utils.ip()))

    def clear(self):
        self.buffer = []
        self.redraw()

    def set_fixed_lines(self, fixed):
        self.num_fixed = fixed
        
    def set_scroll_rate(self, value):
        self.scroll_rate = value

    def _format(self, value):
        if value is None or len(value) < 1:
            value = ' '
        
        # Determine if the caller wants to format the string:
        format_type = value[0]
        if format_type in [_LCD.LEFT_ALIGN, _LCD.CENTER_ALIGN, _LCD.RIGHT_ALIGN]:
            # The user supplied a format character...strip it off
            value = value[1:]
        else:
            # The user didn't supply a format character... use the default
            format_type = _LCD.LEFT_ALIGN

        delta   = self.cols - len(value)

        if delta > 0:
            # Value fits within a row, apply optional padding:
            pad = ' ' * delta
            
            # Right-align
            if format_type == _LCD.RIGHT_ALIGN:
                return pad + value
                
            if format_type == _LCD.LEFT_ALIGN:
                return value + pad
                
            if format_type == _LCD.CENTER_ALIGN:
                padLeft     = pad[:delta // 2]
                padRight    = pad[len(padLeft):]
                return padLeft + value + padRight
        
        else:
            # Value doesn't fit within a row... add some trailing space for
            # the marquee effect
            return value + (' ' * (self.cols // 2))

    def _rotate(self):
        for r in range(self.rows):
            if len(self.buffer) > r:
                if len(self.buffer[r]) > self.cols:
                    self.buffer[r] = self.buffer[r][1:] + self.buffer[r][0]

    def write(self, value):
        # Update the buffer
        self.buffer.append(self._format(value))
        
    def _scroll(self):
        if len(self.buffer) > self.rows:
            del(self.buffer[self.num_fixed])

    def write_line(self, row, value):
        while len(self.buffer) < row:
            self.buffer.append(self._format(None))
        
        # Update the buffer
        self.buffer.append(self._format(value))
        
        # Redraw the 'LCD'
        self.redraw(row)

    def _get_value_at(self, row):
        if row < self.rows:
            return self.buffer[row][:self.cols] if len(self.buffer) > row else ' ' * self.cols
        raise RuntimeError('Row out of range for LCD: ' + str(row))

    def tick(self):
        time_now    = time.time()

        self._rotate()
        
        if self.scroll_rate > 0 and (time_now - self.scroll_ts) >= self.scroll_rate:
            self._scroll()
            self.scroll_ts = time_now
            
        self.redraw()

class _PCF8574_LCD(_LCD):
    
    def __init__(self, address = None, cols = 20, rows = 2, backlight_enabled = True):
        from RPLCD.i2c import CharLCD
        self.device = CharLCD(i2c_expander='PCF8574', address=address, cols=cols, rows=rows, backlight_enabled=backlight_enabled)

        super(_PCF8574_LCD, self).__init__(cols = cols, rows = rows)

    def on(self):
        self.device.backlight_enabled = True
        
    def off(self):
        self.device.backlight_enabled = False

    def redraw(self, row = None):
        self.device.home()
        for r in range(self.rows):
            if row is None or row == r:
                value = self._get_value_at(r)
                self.device.write_string(value)
                self.device.crlf()
                


class _TerminalLCD(_LCD):
    
    def __init__(self, _address = None, cols = 20, rows = 2, backlight_enabled = True):
        import blessed

        self.term   = blessed.Terminal()
        self.backlt = backlight_enabled

        super(_TerminalLCD, self).__init__(cols = cols, rows = rows)


    def on(self):
        self.backlt = True
        self.redraw()
        
    def off(self):
        self.backlt = False
        self.redraw()
            
    def redraw(self, _ = None):
        with self.term.location():
            # Draw header row:
            print(self.term.move_xy(0, 0))
            print(self.term.blue_reverse(' ' + ''.join([str(x % 10) for x in range(self.cols)])))
            
            # Draw row contents, including column labels
            for r in range(self.rows):
                print(self.term.move_xy(0, r + 1))
                print(self.term.blue_reverse(str(r)), end='')
                
                value = self._get_value_at(r)
                if self.backlt:
                    print(self.term.green_reverse(value))
                else:
                    print(self.term.green(value))



def main(args=None):
    import rclpy
    rclpy.init(args=args)

    display = ROS_Lcd()

    rclpy.spin(display)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
