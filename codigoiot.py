import os
import sys
import paho.mqtt.client as mqtt
import json
import random
import psutil
import string
import smbus
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import serial
import adafruit_mlx90614
#Credenciales para enviar datos a PUJ SERVER RACK2
THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = '8bt6LQljXrbDhGeIQrVy'
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, gain=2)
chan = AnalogIn(ads, ADS.P0, ADS.P1)
GPIO.setmode(GPIO.BCM)
triage1 = 17
GPIO.setup(triage1, GPIO.OUT)
triage2 = 27
GPIO.setup(triage2, GPIO.OUT)
triage3 = 22
GPIO.setup(triage3, GPIO.OUT)
triage4 = 23
GPIO.setup(triage4, GPIO.OUT)
# Configuración del intervalo de tiempo de generacion de datos y subida a TB
INTERVAL=1
# Dirección I2C y registros del MAX30100

ser = serial.Serial('/dev/ttyACM0', 115200)  # Reemplaza '/dev/ttyUSB0' con el puerto serie correcto
ser.flushInput()

sensor_data = {'EMG': 0, 'OXIGENO': 0,'TEMPERATURA': 0}

next_reading = time.time() 

client = mqtt.Client()

# Configuracion del username (en el caso de TB es el ACCESS TOKEN)
client.username_pw_set(ACCESS_TOKEN)

# Conexion a ThingsBoard usando MQTT con 60 segundos de duracion de la sesion (keepalive interval)
client.connect(THINGSBOARD_HOST, 1883, 60)

client.loop_start()
data = ser.readline().decode('utf-8').rstrip()
sensor = adafruit_mlx90614.MLX90614(i2c)
try:
    while True:
        #Generacion de los valores de temperatura y humedad (simulacion de valores de los sensores)
        if ser.inWaiting() > 0:
           data = float(ser.readline().decode('utf-8').rstrip())
           print("Datos recibidos:", data)
           #data = float(data)
        temp_objeto = sensor.object_temperature
        temp_ambiente = sensor.ambient_temperature
        print("Temperatura del objeto: {} C".format(temp_objeto))
        print("Temperatura ambiente: {} C".format(temp_ambiente))
        lectura = chan.value
        lectura=lectura*(3.3/65536)
        print('Valor de lectura ADC:', lectura)
        if (lectura > 130) and data<80 and temp_objeto>40 :
         
           print("Encendiendo el LED")
           GPIO.output(triage1, GPIO.HIGH)
         
        else:
           GPIO.output(triage1, GPIO.LOW) 
           
        if (130>lectura > 121) and 89>data>80 and temp_objeto>40:
         
           print("Encendiendo el LED")
           GPIO.output(triage2, GPIO.HIGH)
         
        else:
           GPIO.output(triage2, GPIO.LOW)
        if (120>lectura > 111) and 94>data>90 and 40>temp_objeto>37:
         
           print("Encendiendo el LED")
           GPIO.output(triage3, GPIO.HIGH)
         
        else:
           GPIO.output(triage3, GPIO.LOW)
        if temp_objeto>30 and temp_objeto<37 and lectura>-5 and data>93 and lectura<110 and data<100  :
         
           print("Encendiendo el LED")
           GPIO.output(triage4, GPIO.HIGH)
         
        else:
           GPIO.output(triage4, GPIO.LOW)
         
        
        sensor_data['EMG'] = lectura
        sensor_data['OXIGENO'] = data
        sensor_data['TEMPERATURA'] = temp_objeto

        # Enviando los datos de humidity and temperature a ThingsBoard
        client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)
        print("Entre a TB y revise el campo LATEST TELEMETRY en su dispositivo")
        print(sensor_data)
        next_reading += INTERVAL
        sleep_time = next_reading-time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
            
except KeyboardInterrupt:
    pass
    ser.close()

client.loop_stop()
client.disconnect()