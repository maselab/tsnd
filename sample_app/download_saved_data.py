from tsnd import TSND151
from queue import Queue
import numpy as np
import sys
import time


with open('serial_port.txt', 'r') as f:
    path_to_serial_port = f.readline()[0:-1] # remove \n


with TSND151.open(path_to_serial_port) as tsnd151:
    # init
    tsnd151.stop_recording()
    tsnd151.set_time()

    total_entry_num = tsnd151.get_saved_entry_num() 
    print(f'saved entry num: {total_entry_num}')
    start_times = [start_time = tsnd151.get_saved_entry_start_date(i+1, remove_hms=False) for i in range(total_entry_num)]

    while True:
        for i, start_time in enumerate(start_times):
            start_time.append(start_time)
            print(f'{i:2}: {start_time}')

        cmd = input("[all/end/num(e.g., 1)]: ", end='')
        targets = []

        if cmd.lower()  == "end":
            break
        elif cmd.lower()  == "all":
            targets = [i in range(total_entry_num)]
        else:
            try:
                _ix = int(cmd)
                if _ix < 0 or len(start_times) <= _ix:
                    raise ValueError("invalid input")
                targets.append(_ix)
            except e:
                print(f"invalid input {cmd}")
                continue

        def conv(q_data, start_time):
            ms, acc, gyro = TSND151.parse_acc_gyro(q_data)
            res = [ms / 1000 + start_time, ]
            res.extend(acc)
            res.extend(gyro)
            return np.array(res)

        for i in targets:
            q = Queue()
            tsnd151.set_response_queue('acc_gyro_data', q)  # it should be arranged for the used recording setting
            start_time = start_times[i]
            start_timestamp = start_time.replace(hour=0, minute=0, second=0, microsecond=0).timestamp() 

            print(f'{i+1}/{total_entry_num}: start_time={start_time.strftime("%Y-%m-%d_%H-%M-%S")}', end='', flush=True)
            tsnd151.get_saved_entry(i+1)
            res = np.array([conv(q.get(), start_timestamp) for i in range(q.qsize())])

            np.savetxt(f'{start_time.strftime("%Y-%m-%d_%H-%M-%S")}.csv.gz', res, delimiter=',')
            print(' [Done]')

