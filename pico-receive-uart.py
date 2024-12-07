from machine import UART

uart = UART(1, 115200)
uart.init(115200, bits=8, tx=3,rx=2)

while True:
  line = uart.readline()
  print(f'Got: "{line}"')
  
