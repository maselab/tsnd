from tsnd import TSND151
from queue import Queue
import time 


with open('serial_port.txt', 'r') as f:
    path_to_serial_port = f.readline()[0:-1] # remove \n


acc_gyro_hz=100

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

    tsnd151.set_time()
    tsnd151.set_acc_range(8)  # +-16g
    tsnd151.set_gyro_range(250)  # +-2000g, ignored
    tsnd151.set_acc_and_gyro_interval(  # perhaps ignored, because quat is enabled.
        interval_in_ms=2 # 500hz
        , avg_num_for_send=int(1000 / acc_gyro_hz / 2)  # 10 if 50Hz: 10 avg with 2 ms data = 20 ms
        , avg_num_for_save=int(1000 / acc_gyro_hz / 2))
    tsnd151.set_quaternion_interval(
        interval_in_5ms_unit=0 # off
        , avg_num_for_send=0
        , avg_num_for_save=0)

    tsnd151.set_magnetism_interval(0, 0, 0)  # disable
    tsnd151.set_atmosphere_interval(0, 0, 0)  # disable
    tsnd151.set_battery_voltage_measurement(False, False)  # disable
    tsnd151.set_overwrite_protection(False)  # enable overwrite
    tsnd151.set_auto_power_off(0)  # disable
    tsnd151.set_option_button_behavior(TSND151.OptionButtonMode.START_STOP)

    print(tsnd151.get_option_button_behavior())
