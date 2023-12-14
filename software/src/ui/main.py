import pygame
import serial
from time import sleep
import threading

IN_COM = 'COM20'
OUT_COM = 'COM21'
BAUDRATE = 9600
PARITY = 'N'
STOPBITS = 1

class Game:
  def __init__(self):
    self.s_input = serial.Serial(
      port = IN_COM,
      baudrate = BAUDRATE,
      parity = PARITY,
      stopbits = STOPBITS
    )
    self.s_output = serial.Serial(
      port = OUT_COM,
      baudrate = BAUDRATE,
      parity = PARITY,
      stopbits = STOPBITS
    )

    self.clock = pygame.time.Clock()
    self.window = pygame.display.set_mode((400, 400))
    pygame.font.init()
    self.font = pygame.font.Font(pygame.font.get_default_font(), 36)
    self.input_buffer = ''
  
  def run(self):
    pygame.init()
    self.running = True
    self.keypress = False
    while self.running:
      # self.window.fill((0, 0, 0))
      for event in pygame.event.get():
        self.keypress = False
        if event.type == pygame.QUIT:
          self.running = False
          break
        if event.type == pygame.KEYDOWN:
          self.keypress = True
        command = self.handleKeypress(pygame.key.get_pressed())
        # if self.keypress:
        #   self.handleInput()
        self.s_output.write(bytes(command, 'utf-8'))
        sleep(.1)
      pygame.display.flip()
    pygame.quit()

  def handleKeypress(self, keys):
    if keys[pygame.K_w]:
      return 'w'
    if keys[pygame.K_a]:
      return 'a'
    if keys[pygame.K_s]:
      return 's'
    if keys[pygame.K_d]:
      return 'd'
    if keys[pygame.K_7]:
      return '7'
    if keys[pygame.K_8]:
      return '8'
    if keys[pygame.K_0]:
      return '0'
    if keys[pygame.K_e]:
      return 'e'
    return 'q'
  
  def handleInput(self):
    # if self.s_input.in_waiting > 0:
      self.input_buffer = self.s_input.readline(1).decode()
      self.window.fill((0, 0, 0))
      text = self.font.render(self.input_buffer, True, (255, 255, 255))
      self.window.blit(text, (10, 10))
    # self.font.render_to(self.window, (10, 10), self.input_buffer, (0, 0, 0))

def main():
  game = Game()
  game.run()

if __name__=='__main__':
  main()