import keyboawsrd
import time
time = 0.1
while True:
    if keyboard.is_pressed('w'):  # Press 'w' for forward
        print('w is pressed')
        time.sleep(time)

    elif keyboard.is_pressed('a'):  # Press 'a' for left rotation
        print('a is pressed')
        time.sleep(time)

    elif keyboard.is_pressed('d'):  # Press 'd' for right rotation
        print('d is pressed')
        time.sleep(time)

    elif keyboard.is_pressed('s'):  # Press 's' to stop
        print('s is pressed')
        time.sleep(time)

    if keyboard.is_pressed('q'):  # Press 'q' to quit
        break