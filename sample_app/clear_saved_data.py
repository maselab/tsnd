from tsnd import TSND151
from queue import Queue
import numpy as np
import time
import sys


with open('serial_port.txt', 'r') as f:
    path_to_serial_port = f.readline()[0:-1] # remove \n


with TSND151.open(path_to_serial_port) as tsnd151:
    # init
    tsnd151.stop_recording()
    tsnd151.set_time()

    total_entry_num = tsnd151.get_saved_entry_num() 
    print(f'saved entry num: {total_entry_num}')
    print('clear saved data? [y/N]')

    line = sys.stdin.readline()
    cmd = line.rstrip().lower()
    if cmd == 'y' or cmd == 'yes':
        tsnd151.clear_saved_entry()
        print('clear')
    else:
        print('do nothing')
