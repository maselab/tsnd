# tsnd
Library to handle TSND series, IMU sensors produced by ATR Japan.

# Reqirement
 pip install pyserial

# How to clone this repository
For git clone, please use the following process
```Bash
git clone https://github.com/maselab/tsnd.git
cd tsnd
git submodule update -i
echo "common_utils.py" > .git/modules/utils/info/sparse-checkout
echo "thread_utils.py" >> .git/modules/utils/info/sparse-checkout
(cd tsnd/utils; git config core.sparsecheckout true; git read-tree -mu HEAD)
# git submodule update --remote # if necessary
```

# Sample code
```python
tsnd151 = TSND151.open(path_to_serial_port)
try:
    tsnd151.stop_recording()
    tsnd151.set_time()
    q = Queue()
    tsnd151.set_response_queue('quaternion_acc_gyro_data', q)
    tsnd151.set_quaternion_interval(interval_in_5ms_unit=1, avg_num_for_send=4,
                                    avg_num_for_save=0)  # each 20 ms = 50 Hz, no save on device
    tsnd151.start_recording(force_restart=True)

    time.sleep(3)

    for i in range(q.qsize()):
        print(TSND151.parse_quaternion_acc_gyro(q.get()))

    tsnd151.stop_recording()

    time.sleep(0.2)  # wait for all data
    for i in range(q.qsize()):
        print(TSND151.parse_quaternion_acc_gyro(q.get()))
finally:
    tsnd151.close()
```
or using with
```python
from tsnd import TSND151
from queue import Queue
import time 

path_to_serial_port="/dev/tty.TSND151-AP09181536-Blue"
# path to the blt port
# Examples
#     mac: "/dev/tty.TSND151-AP09181536-Blue"
#     win: "COM1"
#     lin: /dev/rfcomm0
#
# How to make rfcommN in linux (N is int, and > 0)
#   If mac addr of bluetooth device: AC:7A:4D:CE:1B:77,
#   rfcomm bind N "AC:7A:4D:CE:1B:77"
with TSND151.open(path_to_serial_port) as tsnd151:
    tsnd151.stop_recording()
    tsnd151.set_time()
    q = Queue()
    tsnd151.set_response_queue('quaternion_acc_gyro_data', q)
    tsnd151.set_quaternion_interval(interval_in_5ms_unit=1, avg_num_for_send=4, avg_num_for_save=0) # each 20 ms = 50 Hz, no save on device
    tsnd151.start_recording(force_restart=True)

    time.sleep(3)

    for i in range(q.qsize()):
        print(TSND151.parse_quaternion_acc_gyro(q.get()))

    tsnd151.stop_recording()

    time.sleep(0.2) # wait for all data
    for i in range(q.qsize()):
        print(TSND151.parse_quaternion_acc_gyro(q.get()))
```

# NOTE
There are many lacks of functions that is not necessary on related project.
We are happy if you contribute to add such lacking functions. Thanks.

