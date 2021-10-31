
from machine import Pin, ADC, Timer, RTC, TouchPad, deepsleep, I2C, SoftI2C, UART
import esp32
from mqtt import MQTTClient 
import utime
import ssd1306
import webrepl
import time  # needed for ntptime
import network
import ubinascii
import anytemp
import mhz19b

# Pins and pin configs
sdaPin = 21
sclPin = 22
sdaPinSoft = 18
sclPinSoft = 19
uartRx = 16
uartTx = 17
touchPin = 15
touchMaxValue = 250
led = Pin(2,Pin.OUT)  # for onboard LED blink
doDisplay = False
doPowerOn = False

# Wifi object
wlan = network.WLAN(network.STA_IF)

# MQTT
client_id = ubinascii.hexlify(machine.unique_id())
topic_sub = b'home/%s/cmd' % (dev_name)
topic_pub = b'home/%s/metrics' % (dev_name)

# metric variables
message_interval = 300  # duration of deep sleep
signal = 0
temperature_string = ""
humidity_string = ""
pressure_string = ""
co2_string = ""
status = ""

# I2C
# 60 (0x3c) = ssd1306, 118 (0x76) = bme280, 56 (0x38) = aht10
i2c = I2C(1, scl=Pin(sclPin), sda=Pin(sdaPin), freq=400000)
i2c_s = SoftI2C(sda=Pin(sdaPinSoft), scl=Pin(sclPinSoft))
# print(i2c.scan())  # to debug I2C
# print(i2c_s.scan())  # to debug SoftI2C

# Create dispay object using I2C
display = ssd1306.SSD1306_I2C(128, 64, i2c)
display.contrast(50)

# Create AnyTemp object (abstraction for different temp sensors)
tempSensor = anytemp.AnyTemp(i2c_s, temp_sensor_model)

def wifi_connect(wifi_ssid,wifi_passwd):
  wlan.active(True)
  if not wlan.isconnected():
    print('\nConnecting to network', end='')
    wlan.connect(wifi_ssid, wifi_passwd)
    while not wlan.isconnected():
      print('.', end='')
      utime.sleep(0.5)
      pass
  print()
  print("Interface's MAC: ", ubinascii.hexlify(network.WLAN().config('mac'),':').decode()) # print the interface's MAC
  print("Interface's IP/netmask/gw/DNS: ", wlan.ifconfig(),"\n") # print the interface's IP/netmask/gw/DNS addresses

def sub_cb(topic, msg):
  # print((topic, msg))
  last_receive = utime.time()
  print('%s: received message on topic %s with msg: %s' % (last_receive, topic, msg))
  if topic == topic_sub and msg == b'ping':
    client.publish(topic_pub, b'pong')
    print('sent pong')

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub
  client = MQTTClient(client_id, mqtt_server, port=1883, user=mqtt_user, password=mqtt_password)
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  utime.sleep(10)
  machine.reset()

def blink():
    led.on()
    utime.sleep_ms(500)
    led.off()
    utime.sleep_ms(500)

def draw_display():
  display.fill(0)  # clear display by filling with black
  display.rect(0, 0, 128, 64, 1)
  display.hline(0, 50, 128, 1)
  display.text(status, 2, 54, 1)
  if temperature_string:
    temperature_display = temperature_string + ' F'
    display.text(temperature_display, 2, 4, 1)
  if humidity_string:
    humidity_display = humidity_string + ' %'
    display.text(humidity_display, 2, 18, 1)
  if pressure_string:
    display.text(pressure_string, 2, 32, 1)
  if co2_string:
    display.text(co2_string, 2, 32, 1)
  retry = 3
  while retry > 0:
    try:
      display.show()
      break
    except:
      print("retry display (usually I2C timeout when waking from capacitive touch")
      utime.sleep(0.5)
      retry -= 1
      continue

def wait_for_sensor(sleep_sec):
  global status
  print('wait %s seconds on start' % (sleep_sec))
  while sleep_sec > 0:
    status = 's={0}, sleep {1}'.format(signal, sleep_sec)
    # draw_status(status)
    draw_display()
    utime.sleep(1)
    sleep_sec -= 1

def get_metrics():

  # variables used in display (TODO: pass w/ kwargs)
  global temperature_string
  global humidity_string
  global pressure_string
  global co2_string
  global status
  global signal

  tempSensor.read()
  temperature_val = tempSensor.temperature
  humidity_val = tempSensor.humidity
  pressure_string = tempSensor.pressure

  temperature_string = "{:0.1f}".format(round(temperature_val, 1))
  humidity_string = "{:0.1f}".format(round(humidity_val, 1))

  print(temperature_string)
  print(humidity_string)
  print(pressure_string)

  # TODO: Should put a soft-reboot in here if CO2 sensor fails.  Avoid looping forever (implement retry counter in module)
  co2_string = str(co2Sensor.measure()[0])
  print(co2_string)

def display_metrics():
  status = 's={0}'.format(signal)
  display.poweron()
  draw_display()
  utime.sleep(8)

def send_metrics(mqtt_client):
  if pressure_string:
    # msg = b'{{"s":"{0}","t":"{1}","h":"{2}","p":"{3}"}}'.format(signal, temperature_string, humidity_string, pressure_string)
    msg = b'{{"s":"{0}","t":"{1}","h":"{2}"}}'.format(signal, temperature_string, humidity_string)
  elif co2_string:
    msg = b'{{"s":"{0}","t":"{1}","h":"{2}","c":"{3}"}}'.format(signal, temperature_string, humidity_string, co2_string)
  else:
    msg = b'{{"s":"{0}","t":"{1}","h":"{2}"}}'.format(signal, temperature_string, humidity_string)
  blink()
  mqtt_client.publish(topic_pub, msg)
  print('MQTT: published metrics')

def do_deepsleep():
  # to calibrate or debug TouchPad (determine touchMaxValue to use for wakeup)
  # t = TouchPad(Pin(touchPin))
  # print(t.read()) # smaller number when touched

  wake = Pin(touchPin, mode = Pin.IN)
  touch = TouchPad(wake)
  touch.config(touchMaxValue)
  esp32.wake_on_touch(True)
  print('deep sleep for {0}s'.format(message_interval))
  deepsleep(1000 * message_interval)  # ms to sleep

# check how the ESP32 was started up (mainly by touch sensor, hard power on, soft reboot)
boot_reason = machine.reset_cause()
if boot_reason == machine.DEEPSLEEP_RESET:
  print('woke from a deep sleep')  # constant = 4
  wake_reason = machine.wake_reason()
  print("Device running for: " + str(utime.ticks_ms()) + "ms")
  print("wake_reason: " + str(wake_reason))
  if wake_reason == machine.PIN_WAKE:
      print("Woke up by external pin (external interrupt)")
  elif wake_reason == 4:  # machine.RTC_WAKE, but constant doesn't exist
      print("Woke up by RTC (timer ran out)")
  elif wake_reason == 5:  # machine.ULP_WAKE, but constant doesn't match
      print("Woke up capacitive touch")
      doDisplay = True
elif boot_reason == machine.SOFT_RESET:
  print('soft reset detected')  # constant = 5
elif boot_reason == machine.PWRON_RESET:
  print('power on detected') # constant = 1
  # This is used for 2 main reasons:
  # 1. Safety net in case there are issues with deep sleep that makes it difficult to re-upload
  # 2. Often the sensors need a few seconds to get accurate readings when power is first applied
  #    except deep sleep should cut power on the regulated 3.3V pin, but not 5Vin
  doPowerOn = True
  signal = 'NA'
  # wait_for_sensor(5)
  doDisplay = True
elif boot_reason == machine.WDT_RESET:
  print('WDT_RESET detected') # constant = 3
  # This also seems to indicate a hard power on
  doPowerOn = True
  signal = 'NA'
  # wait_for_sensor(5)
  doDisplay = True
else:
  print('boot_reason={0}'.format(boot_reason))

# Give extra time on first boot in case webrepl is needed and initial power-on wait is too short
# CO2 sensor needs extra time on initial startup sending initializing UART
if doPowerOn:
  wait_for_sensor(30)

# Create CO2 sensor object
# needs a few seconds on bootup (move line below to after wait_for_sensor)
co2Sensor = mhz19b.Mhz19b(uartRx, uartTx) # be sure to wire Rx -> Tx and Tx -> Rx

get_metrics()
if doDisplay:
  display_metrics()

wifi_connect(ssid, password)
signal = wlan.status('rssi')
webrepl.start()

# Connect to MQTT
try:
  client = connect_and_subscribe()
except OSError as e:
  restart_and_reconnect()

client.check_msg()
send_metrics(client)
print("Device running before sleep: " + str(utime.ticks_ms()) + "ms")

display.fill(0)  # clear display by filling with black
display.poweroff() # power off the display, pixels persist in memory
do_deepsleep()