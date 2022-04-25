from random import randint

import framebuf
import utime
from machine import Pin, SPI, PWM

#################################################
# PARAMETER
BL = 13
DC = 8
RST = 12
MOSI = 11
SCK = 10
CS = 9

BLOCK_SIZE = 10
FIELD_WIDTH = 10
FIELD_HEIGHT = 20

FIELD_X = 20
FIELD_Y = 20

CYCLE_FRAME = 60  # FRAME of 1 CYCLE (1 FRAME = 10msec sleep + calculating)
DETECT_FRAME = 5  # FRAME of 1 display update and 1 btn detection

DUTY_MAX = 10


#################################################
# I/O
class LCD(framebuf.FrameBuffer):
    def __init__(self):
        self.width = 240
        self.height = 240

        self.cs = Pin(CS, Pin.OUT)
        self.rst = Pin(RST, Pin.OUT)

        self.cs(1)
        self.spi = SPI(1)
        self.spi = SPI(1, 1000_000)
        self.spi = SPI(1, 100000_000, polarity=0, phase=0, sck=Pin(SCK), mosi=Pin(MOSI), miso=None)
        self.dc = Pin(DC, Pin.OUT)
        self.dc(1)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)
        self.init_display()

    def write_cmd(self, cmd):
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([buf]))
        self.cs(1)

    def init_display(self):
        """Initialize dispaly"""
        self.rst(1)
        self.rst(0)
        self.rst(1)

        self.write_cmd(0x36)
        self.write_data(0x70)

        self.write_cmd(0x3A)
        self.write_data(0x05)

        self.write_cmd(0xB2)
        self.write_data(0x0C)
        self.write_data(0x0C)
        self.write_data(0x00)
        self.write_data(0x33)
        self.write_data(0x33)

        self.write_cmd(0xB7)
        self.write_data(0x35)

        self.write_cmd(0xBB)
        self.write_data(0x19)

        self.write_cmd(0xC0)
        self.write_data(0x2C)

        self.write_cmd(0xC2)
        self.write_data(0x01)

        self.write_cmd(0xC3)
        self.write_data(0x12)

        self.write_cmd(0xC4)
        self.write_data(0x20)

        self.write_cmd(0xC6)
        self.write_data(0x0F)

        self.write_cmd(0xD0)
        self.write_data(0xA4)
        self.write_data(0xA1)

        self.write_cmd(0xE0)
        self.write_data(0xD0)
        self.write_data(0x04)
        self.write_data(0x0D)
        self.write_data(0x11)
        self.write_data(0x13)
        self.write_data(0x2B)
        self.write_data(0x3F)
        self.write_data(0x54)
        self.write_data(0x4C)
        self.write_data(0x18)
        self.write_data(0x0D)
        self.write_data(0x0B)
        self.write_data(0x1F)
        self.write_data(0x23)

        self.write_cmd(0xE1)
        self.write_data(0xD0)
        self.write_data(0x04)
        self.write_data(0x0C)
        self.write_data(0x11)
        self.write_data(0x13)
        self.write_data(0x2C)
        self.write_data(0x3F)
        self.write_data(0x44)
        self.write_data(0x51)
        self.write_data(0x2F)
        self.write_data(0x1F)
        self.write_data(0x1F)
        self.write_data(0x20)
        self.write_data(0x23)

        self.write_cmd(0x21)

        self.write_cmd(0x11)

        self.write_cmd(0x29)

    def show(self):
        self.write_cmd(0x2A)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xef)

        self.write_cmd(0x2B)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xEF)

        self.write_cmd(0x2C)

        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)


#################################################
# GAME

# -----------------------------------------------
# enum
class Direction():
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3


class Color:
    WHITE = 0XFFFF
    BLACK = 0X0001

    def calc_color(self, r, g, b):
        color = r << 11 | g << 5 | b
        color = color << 8 & 2 ** 16 - 1 | color >> 8
        return color


# -----------------------------------------------
# class

class Mino:
    I = 0
    O = 1
    S = 2
    Z = 3
    J = 4
    L = 5
    T = 6

    def __init__(self):
        self.type = randint(0, 6)

        center = FIELD_WIDTH // 2
        if self.type == self.I:
            self.block_coordinates = [[center, -1], [center, -2], [center, -3], [center, -4]]
        if self.type == self.O:
            self.block_coordinates = [[center, -1], [center - 1, -1], [center, -2], [center - 1, -2]]
        if self.type == self.S:
            self.block_coordinates = [[center - 1, -1], [center, -1], [center, -2], [center + 1, -2]]
        if self.type == self.Z:
            self.block_coordinates = [[center - 1, -2], [center, -1], [center, -2], [center + 1, -1]]
        if self.type == self.J:
            self.block_coordinates = [[center - 1, -1], [center, -1], [center, -2], [center, -3]]
        if self.type == self.L:
            self.block_coordinates = [[center + 1, -1], [center, -1], [center, -2], [center, -3]]
        if self.type == self.T:
            self.block_coordinates = [[center, -1], [center - 1, -2], [center, -2], [center + 1, -2]]
        # TODO 
        self.color = Color.BLACK

    def get_downed_block_coordinates(self):
        return [[coor[0], coor[1] + 1] for coor in self.block_coordinates]

    def down(self):
        for coor in self.block_coordinates:
            coor[1] += 1

    def renderer(self, LCD):
        for coor in self.block_coordinates:
            if coor[1] >= 0:
                LCD.fill_rect(FIELD_X + coor[0] * BLOCK_SIZE, FIELD_Y + coor[1] * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE,
                              self.color)


class Field:
    def __init__(self, LCD):
        self.LCD = LCD
        self.test = 0xFFFF
        self.blocks = [[0] * FIELD_WIDTH for i in range(FIELD_HEIGHT)]
        self.dropping_mino = Mino()

    def update(self):
        can_down = True
        for coor in self.dropping_mino.get_downed_block_coordinates():
            if coor[1] >= 0 and (coor[1] >= FIELD_HEIGHT or self.blocks[coor[1]][coor[0]] != 0):
                can_down = False
                break

        if can_down:
            self.dropping_mino.down()
        else:
            for coor in self.dropping_mino.block_coordinates:
                self.blocks[coor[1]][coor[0]] = Color.BLACK
            self.dropping_mino = Mino()

    def renderer(self):
        self.LCD.line(FIELD_X, FIELD_Y,
                      FIELD_X + BLOCK_SIZE * FIELD_WIDTH, FIELD_Y,
                      Color.BLACK)
        self.LCD.line(FIELD_X, FIELD_Y,
                      FIELD_X, FIELD_Y + BLOCK_SIZE * FIELD_HEIGHT,
                      Color.BLACK)
        self.LCD.line(FIELD_X + BLOCK_SIZE * FIELD_WIDTH, FIELD_Y,
                      FIELD_X + BLOCK_SIZE * FIELD_WIDTH, FIELD_Y + BLOCK_SIZE * FIELD_HEIGHT,
                      Color.BLACK)
        self.LCD.line(FIELD_X, FIELD_Y + BLOCK_SIZE * FIELD_HEIGHT,
                      FIELD_X + BLOCK_SIZE * FIELD_WIDTH, FIELD_Y + BLOCK_SIZE * FIELD_HEIGHT,
                      Color.BLACK)
        for i in range(len(self.blocks)):
            for j in range(len(self.blocks[i])):
                if self.blocks[i][j] != 0:
                    self.LCD.fill_rect(FIELD_X + j * BLOCK_SIZE, FIELD_Y + i * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE,
                                       self.blocks[i][j])
        self.dropping_mino.renderer(self.LCD)

    def move_left(self):
        self.LCD.fill(0x0000)

    def move_right(self):
        return False

    def move_down(self):
        return False

    def rotate_right(self):
        return False

    def rotate_left(self):
        return False

    def is_gameover(self):
        return False


class Button:
    def __init__(self):
        self.keyA = Pin(15, Pin.IN, Pin.PULL_UP)
        self.keyB = Pin(17, Pin.IN, Pin.PULL_UP)
        self.keyX = Pin(19, Pin.IN, Pin.PULL_UP)
        self.keyY = Pin(21, Pin.IN, Pin.PULL_UP)

        self.up = Pin(2, Pin.IN, Pin.PULL_UP)
        self.dowm = Pin(18, Pin.IN, Pin.PULL_UP)
        self.left = Pin(16, Pin.IN, Pin.PULL_UP)
        self.right = Pin(20, Pin.IN, Pin.PULL_UP)
        self.ctrl = Pin(3, Pin.IN, Pin.PULL_UP)

    def is_pressed_left(self):
        return not self.left.value()

    def is_pressed_right(self):
        return not self.right.value()

    def is_pressed_up(self):
        return not self.up.value()

    def is_pressed_down(self):
        return not self.dowm.value()

    def is_pressed_A(self):
        return not self.keyA.value()

    def is_pressed_B(self):
        return not self.keyB.value()

    def is_pressed_X(self):
        return not self.keyX.value()

    def is_pressed_Y(self):
        return not self.keyY.value()


class Game:
    def __init__(self):
        self.LCD = LCD()
        self.field = Field(self.LCD)
        self.btn = Button()
        self.duty = 0.1 * DUTY_MAX
        self.pwm = PWM(Pin(BL))
        self.pwm.freq(100000)

    def start(self):
        f_cnt = 0
        while True:
            self.LCD.fill(0xFFFF)
            self.pwm.duty_u16(int(self.duty / DUTY_MAX * 65535))

            # detect button and move mino
            if f_cnt % DETECT_FRAME == 0:
                if self.btn.is_pressed_left():
                    self.field.move_left()
                # if self.btn.is_pressed_right():
                #     self.field.move_right()
                # if self.btn.is_pressed_down():
                #     self.field.move_down()
                # if self.btn.is_pressed_A():
                #     self.field.rotate_right()
                # if self.btn.is_pressed_B():
                #     self.field.rotate_left()
                if self.btn.is_pressed_X() and self.duty < DUTY_MAX:
                    self.duty += 1
                if self.btn.is_pressed_Y() and self.duty > 0:
                    self.duty -= 1

            # drop mino
            if f_cnt in [x * 5 for x in range(12)]:
                self.field.update()
                if self.field.is_gameover():
                    break

            self.field.renderer()
            self.LCD.text(f"{f_cnt}", 210, 10, 0)
            self.LCD.show()

            utime.sleep(0.01)
            f_cnt = (f_cnt + 1) % CYCLE_FRAME


#################################################
# MAIN
if __name__ == '__main__':
    game = Game()
    game.start()
