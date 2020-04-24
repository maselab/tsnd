import datetime
import time
import warnings
from tsnd.utils.common_utils import *
from tsnd.utils.thread_utils import *
from queue import Queue, Empty
from threading import Lock
# require pip install pyserial
import serial

class TSND151(ReusableLoopThread):
    """This class is a controller for TSND151"""
    
    _START_BIT_ = b'\x9A'
    _OK_BIT_    = b'\x00'
    _NG_BIT_    = b'\x01'
    _OVERWRITE_OK_BIT_    = b'\x00'
    _OVERWRITE_NG_BIT_    = b'\x01'

    _RESPONCE_ARG_LEN_MAP_ = {
     b'\x8F':1
    ,b'\x90':30
    ,b'\x92':8
    ,b'\x93':13
    ,b'\x97':3
    ,b'\x99':3
    ,b'\x9B':3
    ,b'\x9D':2
    ,b'\x9F':5
    ,b'\xA1':3
    ,b'\xA3':1
    ,b'\xA6':1
    ,b'\xAA':12
    ,b'\xAB':9
    ,b'\xAD':1
    ,b'\xAF':1
    ,b'\xB1':4
    ,b'\xB3':1
    ,b'\xB6':1
    ,b'\xB7':24
    ,b'\xB8':60
    ,b'\xB9':1
    ,b'\xBA':5
    ,b'\xBB':3
    ,b'\xBC':1
    ,b'\xBD':12
    ,b'\xBE':12
    ,b'\xD1':1
    ,b'\xD3':1
    ,b'\xD6':3
    ,b'\xD8':78
    ,b'\xDA':7
    ,b'\xDC':28
    ,b'\xDD':1
    ,b'\x80':22
    ,b'\x81':13
    ,b'\x82':9
    ,b'\x83':7
    ,b'\x84':9
    ,b'\x85':6
    ,b'\x86':13
    ,b'\x87':5
    ,b'\x88':1
    ,b'\x89':1
    ,b'\x8A':30
    ,b'\x8B':22
    ,b'\x8C':12}
    
    _RESPONCE_CODE_MAP_ = {
         'simple': b'\x8F'
        ,'recording_time_settings': b'\x93'
        ,'start_recording': b'\x88'
        ,'stop_recording': b'\x89'
        ,'acc_gyro_data': b'\x80'
        ,'acc_magnetism_data': b'\x81'
        ,'acc_atmosphere_data': b'\x82'
        ,'acc_batery_voltage_data': b'\x83'
        ,'quaternion_acc_gyro_data': b'\x8A'
        ,'acc_range': b'\xA3'
        ,'overwrite_protection': b'\xAF'
        ,'mode': b'\xBC'
        ,'saved_entry_num': b'\xB6'
    }
    
    _CMD_CODE_MAP_ = {
         'set_time': 0x11
        ,'start':0x13
        ,'get_recording_time_settings': 0x14
        ,'stop':0x15
        ,'set_acc_and_gyro_interval':0x16
        ,'set_magnetism_interval':0x18
        ,'set_atmosphere_interval':0x1A
        ,'set_batery_voltage_measurment':0x1C
        ,'set_acc_range': 0x23
        ,'set_gyro_range': 0x25
        ,'set_overwrite_protection':0x2E
        ,'get_overwrite_protection':0x2F
        ,'clear_saved_data': 0x35
        ,'get_saved_entry_num':0x36
        ,'get_mode': 0x3C
        ,'set_quaternion_interval': 0x55        
    }

    def __init__(self, responce_wait_timeout=5):
        self.responce_wait_timeout=responce_wait_timeout

        self.serial = None
        self._responce_queue_map = {
             self._RESPONCE_CODE_MAP_['simple']: Queue()
            ,self._RESPONCE_CODE_MAP_['recording_time_settings']: Queue()
            ,self._RESPONCE_CODE_MAP_['start_recording']: Queue()
            ,self._RESPONCE_CODE_MAP_['stop_recording']: Queue()
            ,self._RESPONCE_CODE_MAP_['mode']: Queue()
            ,self._RESPONCE_CODE_MAP_['overwrite_protection']: Queue()
            ,self._RESPONCE_CODE_MAP_['saved_entry_num']: Queue()
            ,self._RESPONCE_CODE_MAP_['acc_range']: Queue()
        }
        
        self.serial_lock = Lock()
        self.__close=False
        super().__init__(self._in_loop_read_responce)
    
    def set_responce_queue(self, resp_code, q):
        """
        resp: a key of TSND151._RESPONCE_CODE_MAP_
        q: queue. or None to stop store responce
        """
        if resp_code not in TSND151._RESPONCE_CODE_MAP_:
            raise ValueError("Invalid responce code")
        
        self._responce_queue_map[TSND151._RESPONCE_CODE_MAP_[resp_code]] = q

    def clear_all_queue(self):
        for q in self._responce_queue_map.values():
            if q is not None:
                while not q.empty():
                    q.get()
            
    def open(self, path_to_serial_port, timeout_sec=1, baudrate=115200):
        try:
            self.serial_lock.acquire()
            self.__close = False
            self.serial = serial.Serial(path_to_serial_port, baudrate, timeout=timeout_sec)
            time.sleep(0.2)
        finally:
            self.serial_lock.release()

    def close(self):
        if self.serial is not None and self.serial.is_open:
            self.__close = True
            self.serial.close() 
            time.sleep(0.2)
            try:
                self.serial_lock.acquire()
                self.serial = None
            finally:
                self.serial_lock.release()

    
    def read(self, num=1):
        res = b''
        while len(res) < num and not self.check_should_be_stop():
            res += self.serial.read(num-len(res))
        return res

    def read_responce(self):
        while True: 
            b = self.read()
            if b == self._START_BIT_:
                break

        cmd = self.read()
        if cmd not in self._RESPONCE_ARG_LEN_MAP_:
            raise IOError("Invalid cmd_code is recieved: {}".format(cmd))
        args = self.read(self._RESPONCE_ARG_LEN_MAP_[cmd])
        bcc = self.read()
        
        check = self._START_BIT_[0]^cmd[0]
        for b in args:
            check ^=b

        if check.to_bytes(1, 'little') != bcc:
            raise IOError("Invalid Verification Bit")

        return (cmd, args)
    
    def _in_loop_read_responce(self):
        try:
            self.serial_lock.acquire()
            if self.__close or self.serial is None or not self.serial.is_open:
                time.sleep(0.1)
                return
            
            cmd, args = self.read_responce()
            if cmd in self._responce_queue_map:
                q = self._responce_queue_map[cmd]
                if q is not None:
                    q.put(args)
        except serial.SerialException as e:
            if not self.__close:
                raise e
        finally:
            self.serial_lock.release()

    def send(self, cmd, args=[0x00]):
        self.serial.write(self.build_cmd(cmd, args))
        self.serial.flush()

    @staticmethod  
    def build_cmd(cmd_code, args):
        """cmd is array like object of int (of byte)"""
        total_cmd = [TSND151._START_BIT_[0]] # Set start bit
        total_cmd.append(cmd_code)
            
        if type(args) is list or type(args) is tuple:
            total_cmd.extend(args)
        else:
            total_cmd.append(args)
        
        # calc verif bit.
        bcc = 0x00
        for c in total_cmd:
            bcc ^= c

        total_cmd.append(bcc)
        return total_cmd

    def wait_responce(self, code_name):
        while not self.check_should_be_stop():
            try:
                code = self._RESPONCE_CODE_MAP_[code_name]
                if code in self._responce_queue_map:
                    return self._responce_queue_map[code].get(timeout=self.responce_wait_timeout)
                else:
                    raise NotImplementedError("Invalid responce code: {}".format(code))
            except Empty:
                pass
    
    def check_success(self):
        return self.wait_responce('simple') == self._OK_BIT_

    def set_time(self, dt = datetime.datetime.now()):
        if type(dt) != datetime.datetime:
            raise ValueError('Invalid dt. type(dt) have to be datetime.datetime.')
        
        if not self.check_is_cmd_mode():
            return False

        msec = round(dt.microsecond/1000).to_bytes(2, "little")

        self.send(self._CMD_CODE_MAP_['set_time'],[
            dt.year%100, dt.month, 
            dt.day, dt.hour, dt.minute, 
            dt.second, msec[0], msec[1]])
        
        return self.check_success()
        
    def check_is_cmd_mode(self, cannot_send_cmd_warn=True):
        mode = self.get_mode()
        is_cmd_mode = mode == 0 or mode == 2
        if cannot_send_cmd_warn and not is_cmd_mode:
            warnings.warn('Mode is recording. Command cannot be sent.')
        
        return is_cmd_mode
            
    def set_acc_range(self, g=4):
        """g: 2 or 4 or 8 or 16"""
        
        if not self.check_is_cmd_mode():
            return False
        
        flag = 0
        if(g==2):
            flag=0x00 
        elif(g==4):
            flag=0x01
        elif(g==8):
            flag=0x02
        elif(g==16):
            flag=0x03
        else:
            raise ValueError("Invalid acc range: (2,4,8,16) but {}".format(g))
             
        self.send(self._CMD_CODE_MAP_['set_acc_range'], [flag])
        resp = self.wait_responce('acc_range')
        return resp[0] == flag

    def set_gyro_range(self, dps=2000):
        """dps: 250, 500, 1000, 2000"""

        if not self.check_is_cmd_mode():
            return False

        flag = 0
        if(dps==250):
            flag=0x00 
        elif(dps==500):
            flag=0x01
        elif(dps==1000):
            flag=0x02
        elif(dps==2000):
            flag=0x03
        else:
            raise ValueError("Invalid gyro range: (250,500,1000,2000) but {}".format(dps))
             
        self.send(self._CMD_CODE_MAP_['set_gyro_range'], [flag])
        return self.check_success()

    def set_quaternion_interval(self, interval_in_5ms_unit, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_5ms_unit: interval in 5 ms unit.
                              0: off, on: 1-51 (means 5-255 ms interval) 
        avg_num_for_send    : N of SMA for sending via Bluetooth. (def:1)
                              0: off (not send), enable: 1-255.
                              e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save    : N of SMA for saving data in device memory. (def:0)
                              0: off (not save), enable: 1-255.
                              e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.

        If quaternion is enabled,
        NOTE1: +-2000 gyro range is forced.
        NOTE2: quaternion_acc_gyro_data responce is used, no acc_gyro_data.
        """
        
        check_range("interval_in_5ms_unit", interval_in_5ms_unit, 0, 51)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False
        
        self.send(self._CMD_CODE_MAP_['set_quaternion_interval'], 
                  [interval_in_5ms_unit*5, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_acc_and_gyro_interval(self, interval_in_ms, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_ms   : interval in ms.
                           0: off, on: 1-255 
        avg_num_for_send : N of SMA for sending via Bluetooth. (def:1)
                           0: off (not send), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save : N of SMA for saving data in device memory. (def:0)
                           0: off (not save), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.
        """
        check_range("interval_in_ms", interval_in_ms, 0, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False
        
        self.send(self._CMD_CODE_MAP_['set_acc_and_gyro_interval'], 
                  [interval_in_ms, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_magnetism_interval(self, interval_in_ms, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_ms   : interval in ms.
                           0: off, on: 10-255 
        avg_num_for_send : N of SMA for sending via Bluetooth. (def:1)
                           0: off (not send), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save : N of SMA for saving data in device memory. (def:0)
                           0: off (not save), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.
        """
        
        if interval_in_ms != 0:
            check_range("interval_in_ms", interval_in_ms, 10, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False
        
        self.send(self._CMD_CODE_MAP_['set_magnetism_interval'], 
                  [interval_in_ms, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_atmosphere_interval(self, interval_in_10ms_unit, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_10ms_unit: interval in 10 ms unit.
                               0: off, on: 4-255 (means 40-2550 ms interval) 
        avg_num_for_send     : N of SMA for sending via Bluetooth. (def:1)
                               0: off (not send), enable: 1-255.
                               e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save     : N of SMA for saving data in device memory. (def:0)
                               0: off (not save), enable: 1-255.
                               e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.
        """
        
        if interval_in_10ms_unit != 0:
            check_range("interval_in_10ms_unit", interval_in_10ms_unit, 4, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False
        
        self.send(self._CMD_CODE_MAP_['set_atmosphere_interval'], 
                  [interval_in_10ms_unit, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    
    def set_batery_voltage_measurment(self, send=False, save=False):
        """
        send: sending via Bluetooth or not. (def:False)
        send: sending to device memory or not. (def:False)
        """
        if type(send) is not bool:
            ValueError("Invalid 'send' (have to be bool):" + send)
        if type(save) is not bool:
            ValueError("Invalid 'save' (have to be bool):" + save)

        if not self.check_is_cmd_mode():
            return False
         
        self.send(self._CMD_CODE_MAP_['set_batery_voltage_measurment'], [send, save])
        return self.check_success()
    
    def set_overwrite_protection(self, enable=False):
        """
        enable: enable overwrite protection for device memory or not
        """
        if type(enable) is not bool:
            ValueError("Invalid 'enable' (have to be bool):" + send)

        if not self.check_is_cmd_mode():
            return False
             
        self.send(self._CMD_CODE_MAP_['set_overwrite_protection'], [enable])
        return self.check_success()

    def get_overwrite_protection(self, enable=False):
        """
        return: overwrite protection for device memory is enabled or not
        """
        if not self.check_is_cmd_mode():
            return None
             
        self.send(self._CMD_CODE_MAP_['get_overwrite_protection'])
        res = self.wait_responce('overwrite_protection')
        return res == TSND151._OVERWRITE_NG_BIT_
    
    def start_recording(self, force_restart=False, return_start_time_hms=False):

        if not self.check_is_cmd_mode(False):
            warnings.warn('Recording already')
            if not force_restart:
                return None
            elif not self.stop_recording():
                warnings.warn('Failed to stop recording')
                return None
        
        flag = [0, 0, 1, 1, 0, 0, 0, # start immediately  
                0, 0, 1, 1, 0, 0, 0] # run forever
        self.send(self._CMD_CODE_MAP_['start'], flag)
        
        resp = self.wait_responce('recording_time_settings')
        start_time = None
        if resp[0] == 1:
            if return_start_time_hms:
                start_time = datetime.datetime(resp[1]+2000, resp[2], resp[3], resp[4], resp[5], resp[6])
            else:
                start_time = datetime.datetime(resp[1]+2000, resp[2], resp[3], 0, 0, 0)

        run_forever = True
        for res, src in zip(resp[7:14], [100,1,1,0,0,0]): # run forever flags
            if res != src:
                run_forever = False
        
        if not run_forever:
            stop_time = datetime.datetime(resp[7]+2000, resp[8], resp[9], resp[10], resp[11], resp[12])
            warnings.warn('Set run forever, but stop time has been set:{}'.format(stop_time))

        self.wait_responce('start_recording')        
        return start_time
        
    def stop_recording(self):
        if self.check_is_cmd_mode(False):
            warnings.warn('Stopped already')
            return True

        cmd_code = self._CMD_CODE_MAP_['stop']
        self.send(cmd_code)
        
        resp = self.check_success()
        if not resp:
            return False

        self.wait_responce('stop_recording')        
        return True
    
    def get_recording_time_settings(self, return_start_time_hms=False):
        if not self.check_is_cmd_mode():
            return None
             
        self.send(self._CMD_CODE_MAP_['get_recording_time_settings'])
        resp = self.wait_responce('recording_time_settings')
        
        scheduled = resp[0] == 1
        if return_start_time_hms:
            start_time = datetime.datetime(resp[1]+2000, resp[2], resp[3], resp[4], resp[5], resp[6])
        else:
            start_time = datetime.datetime(resp[1]+2000, resp[2], resp[3], 0, 0, 0)
        
        stop_time = datetime.datetime(resp[7]+2000, resp[8], resp[9], resp[10], resp[11], resp[12])
        
        return((scheduled, start_time, stop_time))
    
    def get_mode(self):
        """
        return: 0: USB_CMD, 1:USB_RECORDING, 2: BLT_CMD, 3:BLT_RECORDING
        """
        self.send(self._CMD_CODE_MAP_['get_mode'])
        resp = self.wait_responce('mode')
        return(resp[0])
    
    def get_saved_entry_num(self):
        """
        return: number of entry saved in the device
        """
        if not self.check_is_cmd_mode():
            return None

        self.send(self._CMD_CODE_MAP_['get_saved_entry_num'])
        resp = self.wait_responce('saved_entry_num')
        return(resp[0])
    
    def clear_saved_entry(self):
        """clear saved data"""
        if not self.check_is_cmd_mode():
            return False
    
        self.send(self._CMD_CODE_MAP_['clear_saved_data'])
        return self.check_success()

    @staticmethod
    def parse_acc_gyro(bytes_):
        """
        return: (ms, acc, gyro): acc=(ax, ay, az) in 0.1mg, gyro=(gx, gy, gz) in 0.01dps
        """
        ms = int.from_bytes(bytes_[0:4], "little")
        acc  = [(int.from_bytes(bytes_[i:(i+3)], "little", signed=True)) for i in range(4, 13, 3)]
        gyro  = [(int.from_bytes(bytes_[i:(i+3)], "little", signed=True)) for i in range(13, 22, 3)]
        
        return (ms, acc, gyro) # ms, acc, gyro
 
    @staticmethod
    def parse_quaternion_acc_gyro(bytes_):
        """
        return: (ms, quaternion, acc, gyro): acc=(ax, ay, az) in 0.1mg, gyro=(gx, gy, gz) in 0.01dps
        """
        ms = int.from_bytes(bytes_[0:4], "little", signed=False)
        quat = [int.from_bytes(bytes_[i:(i+2)], "little", signed=True) for i in range(4, 12, 2)]
        acc  = [(int.from_bytes(bytes_[i:(i+3)], "little", signed=True)) for i in range(12, 21, 3)]
        gyro  = [(int.from_bytes(bytes_[i:(i+3)], "little", signed=True)) for i in range(21, 30, 3)]
        return (ms, quat, acc, gyro)
