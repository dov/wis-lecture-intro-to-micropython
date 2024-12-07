######################################################################
#  A micropython program for the M5 ATOM that outputs a uart message
#  everytime the button is pressed.
#
#  2024-12-06 Fri
#  Dov Grobgeld <dov.grobgeld@gmail.com>
######################################################################

import utime
import machine
from machine import Pin
from machine import UART

# Button g39
button = Pin(39, Pin.IN)
led = Pin(26, Pin.OUT)

button_presses = 0 # the count of times the button has been pressed
last_time = 0 # the last time we pressed the button

# IRQ handler for the pin irq
def button_pressed_handler(pin):
    global button_presses, last_time
    new_time = utime.ticks_ms()
    # Debounce trigger = 200 ms
    if new_time - last_time > 200: 
        button_presses +=1
        last_time = new_time

# Register the handler function when the button is pressed
button.irq(trigger=Pin.IRQ_FALLING, handler=button_pressed_handler)

# Init uart on the yellow (G26) and white (G32) wires from the
# cable
uart = UART(1, 115200)
uart.init(115200, bits=8, tx=26, rx=32)

old_presses = 0
while True:
    # Only print on change in the button_presses value
    if button_presses != old_presses:
        msg = f'Hello {button_presses}'
        print(msg)
        uart.write(msg + '\n')  
        old_presses = button_presses
        
