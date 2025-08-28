from machine import Pin, PWM, ADC
import time

# Pin mapping (change if needed)
in1 = Pin(26, Pin.OUT)
in2 = Pin(27, Pin.OUT)
enA = PWM(Pin(25), freq=1000)   # motor enable with PWM
buttonPin = Pin(2, Pin.IN, Pin.PULL_UP)
ledPin = Pin(13, Pin.OUT)
currentSensePin = ADC(Pin(34))  # ESP32 analog input (use pin 36/39/34 only!)

# Button debounce
lastButtonState = 1
currentButtonState = 1
lastDebounceTime = 0
debounceDelay = 50

# PWM duty (0–1023 for ESP32 in MicroPython)
openPWM = 920   # ~235/255 scaled
closePWM = 1000 # ~250/255 scaled

# Current detection tuning
emaAlpha = 0.20
minRunningCurrent = 0.3
openSpikeRate = 0.35
closeSpikeRate = 0.45
ignoreSpikeTime = 250  # ms

filteredCurrent = 0.0
previousFiltered = 0.0

direction = True  # True=open, False=close


def millis():
    return time.ticks_ms()


def setMotor(pwm_val, dir_open):
    if dir_open:
        in1.value(1)
        in2.value(0)
    else:
        in1.value(0)
        in2.value(1)
    enA.duty(int(pwm_val))


def stopMotor():
    enA.duty(0)


def readCurrent():
    raw = currentSensePin.read()  # 0–4095
    voltage = (raw / 4095.0) * 3.3
    current = (voltage - 2.5) / 0.100  # adjust for your ACS712
    return abs(current)


def runMotorUntilSpike():
    global direction, filteredCurrent, previousFiltered

    ledPin.value(1)

    # Run motor
    if direction:
        setMotor(openPWM, True)
    else:
        setMotor(closePWM, False)

    startTime = millis()
    previousFiltered = readCurrent()
    filteredCurrent = previousFiltered

    while True:
        time.sleep_ms(20)
        current = readCurrent()
        previous = filteredCurrent
        filteredCurrent = emaAlpha * current + (1 - emaAlpha) * filteredCurrent
        elapsed = millis() - startTime
        rate = (filteredCurrent - previous) / 0.020
        overMinCurrent = filteredCurrent > minRunningCurrent
        timeOk = elapsed > ignoreSpikeTime

        if timeOk and overMinCurrent:
            if direction and rate > openSpikeRate:
                break
            if not direction and rate > closeSpikeRate:
                break

        print("Current:", filteredCurrent, "| Rate:", rate)

    stopMotor()
    ledPin.value(0)
    direction = not direction


# Main loop
print("Filtered Current (A)")
while True:
    reading = buttonPin.value()
    if reading != lastButtonState:
        lastDebounceTime = millis()

    if (millis() - lastDebounceTime) > debounceDelay:
        if reading != currentButtonState:
            currentButtonState = reading
            if currentButtonState == 0:  # button pressed
                runMotorUntilSpike()

    lastButtonState = reading
    rawCurrent = readCurrent()
    filteredCurrent = emaAlpha * rawCurrent + (1 - emaAlpha) * filteredCurrent
    print(filteredCurrent)
    time.sleep_ms(100)
