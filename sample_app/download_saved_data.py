from tsnd import TSND151
from queue import Queue
import numpy as np
import time

path_to_serial_port="/dev/tty.TSND151-AP09181537"

with TSND151.open(path_to_serial_port) as tsnd151:
    # init
    tsnd151.stop_recording()
    tsnd151.set_time()

    total_entry_num = tsnd151.get_saved_entry_num() 
    print(f'saved entry num: {total_entry_num}')

    def conv(q_data, start_time):
        ms, acc, gyro = TSND151.parse_acc_gyro(q_data)
        res = [ms / 1000 + start_time, ]
        res.extend(acc)
        res.extend(gyro)
        return np.array(res)

    for i in range(total_entry_num):
        q = Queue()
        tsnd151.set_response_queue('acc_gyro_data', q)  # it should be arranged for the used recording setting
        start_time = tsnd151.get_saved_entry_start_date(i+1, remove_hms=False)
        start_timestamp = start_time.replace(hour=0, minute=0, second=0, microsecond=0).timestamp() 

        print(f'{i+1}/{total_entry_num}: start_time={start_time.strftime("%Y-%m-%d_%H-%M-%S")}', end='')
        tsnd151.get_saved_entry(i+1)
        res = np.array([conv(q.get(), start_timestamp) for i in range(q.qsize())])

        np.savetxt(f'{start_time.strftime("%Y-%m-%d_%H-%M-%S")}.csv', res, delimiter=',')
        print(' [Done]')

