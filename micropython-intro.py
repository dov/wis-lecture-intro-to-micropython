#+STARTUP: hidestars showall 
#+OPTIONS: ^:nil toc:nil num:nil html-postamble:nil
#+HTML_HEAD: <link rel="stylesheet" type="text/css" href="dov-org.css" />
#+AUTHOR: Dov Grobgeld
#+TITLE: micropython and rp2040-presentation
#+DATE: 2024-11-19 Tue

* RP2XXX Board

  - Boards developed by the Raspberry Pi foundation
  - First version was 2040
  - Current version 2350
  - Very good and detailed documentation
  - Free to use for third party - lots of boards

* RP2040 spec

  - Dual Arm Cortex-M0+, flexible clock up to 133 MHz
  - 264kB on-chip SRAM
  - 2 × UART, 2 × SPI controllers, 2 × I2C controllers, 16 × PWM channels
  - 1 × USB 1.1 controller and PHY, with host and device support
  - 8 × Programmable I/O (PIO) state machines for custom peripheral support
  - Price: ~5 USD

* RP2350 improvements

  - Arm Cortex-M33
  - RISC-V support
  - 512kB memory
  - More PIO
  - Less power consumption

* Programming (burning)

  - Press button BOOTSEL and copy an UF2 file
  - Reboot without BOOTSEL and the program is running

* MicroPython

  - A simplified version of python for microcontrollers
  - Install by copying the UF2 file
  - Scripts are kept on local file system
  - Special script ~main.py~ is run on reboot
  - Provides an interactive shell

* Other "standard" board firmware

  - "No firmware" - Program in Arduino or C/C++ SDK
  - program in assembly
  - Tasmota (ESP8266 and ESP32 only) - MQTT based firmware. Great for IOT
  - Circuit python - Fork of micropython by AdaFruit
  - MicroLua - Standard lua

* Thonny python editor

  - Simple editor for learners of python
  - Comes with micropython integration
  - Need to configure port and bord

* Terminal - Turn on the built in led

#+begin_src python
import machine
pin = machine.Pin(0, machine.Pin.OUT)
pin.value(0)
pin.value(1)
#+end_src
* main.py script

  - Scripts can be stored on the RP2040 file system
  - Libraries can be written and stored
  - ~main.py~ is automatically run on boot
  - Can stop the script with ~Control-C~
  - Blink led on boot:

  #+begin_src python
from machine import Pin
import time

pin = Pin(0)  # Output by default
i = 0
while True:
  pin.value(1-pin.value())
  i+= 1
  time.sleep(0.2)
  #+end_src

* pimoroni explorer board

  - Comes with several IO's connected and other hardware
  - Comes with custom micropython with additional hardware support
  - Graphics hello world:
  #+begin_src python
from picographics import PicoGraphics, DISPLAY_PICO_EXPLORER
display = PicoGraphics(display = DISPLAY_PICO_EXPLORER)

width, height = display.get_bounds()
WHITE= display.create_pen(255,255,255)

display.set_pen(WHITE)
display.text('Hello world', 3,3,0,3)
display.update()
  #+end_src

* Communications protocols

  - Predefined libraries for common micro controller protocols:
    - UART - Serial
    - I2C 
    - SPI

* Example read UART input

#+begin_src python
from machine import UART, Pin
import time

uart0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
rxData = bytes()
while uart0.any() > 0:
    rxData += uart0.read(1)

print(rxData.decode('utf-8'))
#+end_src

  - Will send UART communication from host dongle
  - Host green: TX
  - Host orange: RX
  - Program: on host 
  #+begin_src sh
minicom -b 9600 -o -D /dev/ttyUSB0
  #+end_src

* Servo motor

  - Example of included library
#+begin_src python
from servo import Servo
import time

sm = Servo(pin=0)

for i in range(10):
    sm.value(90*(i%2))
    time.sleep(1)
#+end_src

* LED panel

  - Another included library neopixel
#+begin_src python
import neopixel
from machine import Pin
import time

ws_pin = 0  # GP0
led_num = 64
BRIGHTNESS = 0.2  # Adjust the brightness (0.0 - 1.0)

neoPanel = neopixel.NeoPixel(Pin(ws_pin), led_num)

neoPanel.fill((0,10,0))
neoPanel.write()

for pixel in (0,7,63,63-7):
  neoPanel[pixel] = (255,0,0)
neoPanel.write()
#+end_src

* wifi

  - Example on the d1 mini (6.5 NIS)
#+begin_src python
import network, ubinascii
sta_if = network.WLAN(network.STA_IF)
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)
sta_if.active(True)
sta_if.connect(XJET3D-Guest','Xjet2015')
sta_if.isconnected()

# Interactive
print(ubinascii.hexlify(network.WLAN().config('mac'),':').decode())
sta_if.ifconfig()
#+end_src

* http queries

  - Almost like in standard python
  #+begin_src python
import urequests

# Query a public api
astronauts = urequests.get("http://api.open-notify.org/astros.json").json()
number = astronauts['number']
print('There are', number, 'astronauts in space.')
for i in range(number):
    print(i+1, astronauts['people'][i]['name'])
  #+end_src

* PIO

  - State machine for high resolution - accurate timed protocols
  - Simplest example blink
  - Can set frequency, slowest frequency is 2000 Hz
  - Need lots of delays to slow it down to 1Hz
  - Assembly code described in python syntax

  #+begin_src python
import time
from machine import Pin
import rp2

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    # Cycles: 1 + 1 + 6 + 32 * (30 + 1) = 1000
    irq(rel(0))
    set(pins, 1)
    set(x, 31)                  [5]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    set(pins, 0)
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")


# Create the StateMachine with the blink_1hz program, outputting on GP0.
sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(0))

# Set the IRQ handler to print the millisecond timestamp.
sm.irq(lambda p: print(time.ticks_ms()))

# Start the StateMachine.
sm.active(1)
  #+end_src

* PIO for ws2812b

  - More complex example
  - Let's do ws2812 on our own!
  - One channel protocol with accurate timing
  - ws2812b protocol description: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
  - pio example:
#+begin_src python
import array, time
from machine import Pin
import rp2

# Configure the number of WS2812 LEDs.
PANEL_NUM_COLUMNS=8
NUM_LEDS = 8*PANEL_NUM_COLUMNS

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]  # 0.375us off
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]  # 0.25us on
    jmp("bitloop")          .side(1)    [T2 - 1]  # bit 1 -> 0.625 on
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]  # bit 0 -> 0.625 off
    wrap()


# Create the StateMachine with the ws2812 program, outputting on Pin(22).
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(0))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Display a pattern on the LEDs via an array of LED RGB values.
ar = array.array("I", [0 for _ in range(NUM_LEDS)])

sleep_time_in_ms = 200

smiley = [
0,0,1,1,1,1,0,0,
0,1,0,0,0,0,0,0,
1,0,1,0,0,1,0,1,
1,0,0,0,0,0,0,1,
1,0,1,0,0,1,0,1,
1,0,0,1,1,0,0,1,
0,1,0,0,0,0,1,0,
0,0,1,1,1,1,0,0]

ar = array.array('I', 
  [sm*0x0f0000 for sm in smiley])
  
while True:
  sm.put(ar)
  time.sleep_ms(sleep_time_in_ms)
#+end_src

* Encoder

- file:encoder_explorer_move_fwd_back.png
#+begin_src python
# A+ B+ encoder forward motion. This carries out four steps. The four pins are
# 
#  A    0b0001 = 1
#  B    0b0010 = 2
#  ¬A   0b0100 = 4
#  ¬B   0b1000 = 8
@rp2.asm_pio(set_init=(rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW,rp2.PIO.OUT_HIGH, rp2.PIO.OUT_HIGH))
def encoder_fwd():
    pull()                      # Get number of encoder cycles (4 steps). 
                                # pull() stores in to the OSR
    mov(y, osr)

    # Cycles: 1 + 1 + 6 + 32 * (30 + 1) = 1000

    # A¬B
    jmp(y_dec, "step")          # Do y--
    label("step")
    set(pins, 1+8)                          
    set(x, 31)                  [6]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    #  A B
    set(pins, 3)                            
    set(x, 31)                  [6]
    label("delay_high1")
    nop()                       [29]
    jmp(x_dec, "delay_high1")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    # ¬Α B
    set(pins, 2+4)                          
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")

    # ¬A¬B
    # Cycles: 1 + 7 + 31 * (30 + 1) + (30 + 1) = 1000
    set(pins, 4+8)                          
    set(x, 30)                  [6]
    label("delay_low1")
    nop()                       [29]
    jmp(x_dec, "delay_low1")
    nop()                       [29]

    jmp(y_dec, "step")

:
pos = 0
def move_fwd(req_pos = 0, freq = 1000):
  global pos
  sm = rp2.StateMachine(0, encoder_fwd, freq=1000*freq, set_base=Pin(0))
  sm.active(1)
  sm.put(round((req_pos - pos))//4)
#+end_src
