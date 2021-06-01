#!/usr/bin/env python

import keyboard  # using module keyboard

while True:  # making a loop
    if keyboard.is_pressed('q'):
        print('You Pressed Q Key!')
    if keyboard.is_pressed('d'):
        print('You Pressed D Key!')