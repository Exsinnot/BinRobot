import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X

# Initialize the I2C bus
i2c = board.I2C()

# Define the shutdown (SHDN) pins for each VL53L0X sensor
xshut = [
    DigitalInOut(board.D20),  # First sensor shutdown pin
    DigitalInOut(board.D21)   # Second sensor shutdown pin
]

# Ensure the SHDN pins are set as outputs and turn off all sensors
for power_pin in xshut:
    power_pin.switch_to_output(value=False)

# Initialize the list to store VL53L0X sensor objects
vl53 = []

# Power on the first sensor and change its address
xshut[0].value = True  # Turn on the first sensor
time.sleep(0.1)        # Allow time for the sensor to initialize
vl53.append(VL53L0X(i2c))  # Initialize the first sensor
vl53[0].set_address(0x30)  # Change the address to 0x30 (or any address between 0x30-0x34)

# Power off the first sensor
xshut[0].value = False

# Power on the second sensor and change its address
xshut[1].value = True  # Turn on the second sensor
time.sleep(0.1)        # Allow time for the sensor to initialize
vl53.append(VL53L0X(i2c))  # Initialize the second sensor
vl53[1].set_address(0x2A)  # Change the address to 0x31 (or any address between 0x30-0x34)

# Now both sensors have unique addresses and can be used simultaneously
xshut[0].value = True  # Turn on the first sensor again
xshut[1].value = True  # Keep the second sensor on
