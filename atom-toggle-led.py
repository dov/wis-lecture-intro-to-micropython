######################################################################
#  A micropython program for the M5 ATOM that outputs a uart message
#  everytime the button is pressed.
######################################################################

import utime
import machine
from machine import Pin

# Mapping
#  Yellow G26 -> tx
#  White G32 -> rx

# Button g39
button = Pin(39, Pin.IN)
led = Pin(26, Pin.OUT)

button_presses = 0 # the count of times the button has been pressed
last_time = 0 # the last time we pressed the button

# This function gets called every time the button is pressed.  The parameter "pin" is not used.
def button_pressed_handler(pin):
    global button_presses, last_time
    new_time = utime.ticks_ms()
    # if it has been more that 1/5 of a second since the last event, we have a new event
    if (new_time - last_time) > 200: 
        button_presses +=1
        last_time = new_time

# now we register the handler function when the button is pressed
button.irq(trigger=machine.Pin.IRQ_FALLING, handler = button_pressed_handler)

# This is for only printing when a new button press count value happens
old_presses = 0
while True:
    # only print on change in the button_presses value
    if button_presses != old_presses:
        print(button_presses)
        led.value(1-led.value())
        old_presses = button_presses
