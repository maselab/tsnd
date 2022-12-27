from tsnd import TSND151
from queue import Queue
import numpy as np
import time


hz=100
with TSND151.open(path_to_serial_port="/dev/tty.TSND151-AP09181537-Blue"
        , wait_sec_on_open_for_stability=0.2
        , wait_sec_on_auto_close_for_stability=0.2
        ) as tsnd151:
    # init
    tsnd151.stop_recording()
    tsnd151.set_time()
    tsnd151.set_acc_range(16)  # +-16g
    tsnd151.set_gyro_range(2000)  # +-2000g, ignored
    tsnd151.set_acc_and_gyro_interval(  # perhaps ignored, because quat is enabled.
        interval_in_ms=2
        , avg_num_for_send=int(1000 / hz / 2)  # 10 if 50Hz: 10 avg with 2 ms data = 20 ms
        , avg_num_for_save=0)
    tsnd151.set_quaternion_interval(
        interval_in_5ms_unit=1
        , avg_num_for_send=int(1000 / hz / 5)  # 4 if 50Hz: 4 avg with 5 ms data = 20 ms
        , avg_num_for_save=0)

    tsnd151.set_magnetism_interval(0, 0, 0)  # disable
    tsnd151.set_atmosphere_interval(0, 0, 0)  # disable
    tsnd151.set_battery_voltage_measurement(False, False)  # disable
    tsnd151.set_overwrite_protection(False)  # enable overwrite
    tsnd151.set_auto_power_off(0)  # disable

    q = Queue()
    tsnd151.set_response_queue('quaternion_acc_gyro_data', q)
    start_time = tsnd151.start_recording(force_restart=True).timestamp()
    ##############################
    try:
        def conv(q_data):
            ms, _, acc, gyro = TSND151.parse_quaternion_acc_gyro(q_data)
            res = [ms / 1000 + start_time, ]
            res.extend(acc)
            res.extend(gyro)
            return np.array(res)

        while True:
            #if q.empty():
            time.sleep(0.005)  # 5 ms
            #else:
            #    res = np.array([conv(q.get()) for i in range(q.qsize())])
            #    print(res)
    finally:
        tsnd151.stop_recording()
        res = np.array([conv(q.get()) for i in range(q.qsize())])
        np.savetxt("tmp.csv", res, delimiter=',')
    ##############################

