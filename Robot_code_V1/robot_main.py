from utils.Robot import Robot
from utils import *
import os
from pynput import mouse, keyboard

#global
robot = None

def on_press(key):
    print(f"{key} pressed")
    if str(key) == "Key.up":
        robot.wheels.forward()
    if str(key) == "Key.down":
        robot.wheels.backward()
    if str(key) == "Key.right":
        robot.wheels.turn_right()
    if str(key) == "Key.left":
        robot.wheels.turn_left()
    if str(key) == 's':
        print(robot.env_detection.sweep())

def main():
    global robot
    robot = Robot()
    
    with keyboard.Listener(on_press=on_press) as k_listener:
        k_listener.join()

if __name__ == "__main__":
    main()
    
