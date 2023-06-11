import busio
import digitalio
import board
import time
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P6)

pins = [AnalogIn(mcp, MCP.P0), AnalogIn(mcp, MCP.P1), AnalogIn(mcp, MCP.P2), AnalogIn(mcp, MCP.P3), AnalogIn(mcp, MCP.P4), AnalogIn(mcp, MCP.P5), AnalogIn(mcp, MCP.P6), AnalogIn(mcp, MCP.P7)]

print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
print('-' * 57)


while True:
    values = [0]*8
    for i, channel in enumerate(pins):
        values[i] = channel.value
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    time.sleep(1)
