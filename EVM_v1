from machine import Pin, ADC, PWM
from time import sleep

# ----------------------
# Pins
# ----------------------
motor_in1 = Pin(26, Pin.OUT)
motor_in2 = Pin(27, Pin.OUT)
motor_enable = PWM(Pin(25))  # PWM speed control
motor_enable.freq(1000).p

current_pin = ADC(Pin(34))  # ACS712 output
current_pin.atten(ADC.ATTN_11DB)  # 0-3.3V full scale

# ----------------------
# ACS712 parameters
# ----------------------
sensitivity = 0.1  # 100 mV/A for 20A sensor
samples_calibration = 50
samples_smooth = 10
overcurrent_limit = 0.210  # A

# ----------------------
# Functions
# ----------------------
def calibrate_offset():
    total = 0
    for _ in range(samples_calibration):
        total += current_pin.read()
    avg = total / samples_calibration
    return (avg / 4095) * 3.3  # convert ADC to voltage

def read_current():
    raw = current_pin.read()
    voltage = (raw / 4095) * 3.3
    current = (voltage - ACSoffset) / sensitivity
    return -current  # flip sign if needed

def read_current_smooth():
    total = 0
    for _ in range(samples_smooth):
        total += read_current()
    return total / samples_smooth

# ----------------------
# Calibration
# ----------------------
print("Calibrating ACS712 offset... keep motor OFF")
sleep(2)
ACSoffset = calibrate_offset()
print("Calibrated ACS offset:", round(ACSoffset, 3), "V")

# ----------------------
# Motor control (start forward)
# ----------------------
motor_in1.value(1)
motor_in2.value(0)
motor_enable.duty_u16(65535)  # full speed

# ----------------------
# Main loop with overcurrent check
# ----------------------
while True:
    current = read_current_smooth()
    print("Motor current: {:.3f} A".format(current))
    
    if current > overcurrent_limit:
        print("Overcurrent detected! Motor stopped.")
        motor_enable.duty_u16(0)   # stop motor
        motor_in1.value(0)
        motor_in2.value(0)
        break  # exit loop
    
    sleep(0.5)

