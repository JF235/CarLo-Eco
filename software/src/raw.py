import serial
from serial.tools.list_ports import comports
import threading
from time import sleep

BAUDRATE = 9600
PARITY = 'N'
STOPBITS = 1

running = True

def reciever():
  global running

  ser_in = serial.Serial("COM20", baudrate=BAUDRATE, parity=PARITY, stopbits=STOPBITS, timeout=2)
  while running:
    if ser_in.in_waiting:
      print(ser_in.read_all().decode())
    sleep(1)

ser = serial.Serial()

def main():
  global running

  for p in comports():
    print(f"{p.device} {p.name} {p.location} {p.description}")
  port = input()

  ser.port = port
  ser.baudrate = BAUDRATE
  ser.parity = PARITY
  ser.stopbits = STOPBITS
  ser.timeout = 2
  ser.open()

  t = threading.Thread(target=reciever)
  t.start()

  if not ser.writable():
    print('Serial not writable')
    return

  while running:
    command = input()
    if command == 'exit':
      running = False
      break
    ser.write(bytes(command, 'utf-8'))
  
  t.join()

if __name__=='__main__':
  main()