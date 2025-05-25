
from lib.comm.wifi_comm import WifiComm
from lib.comm.bluetooth_comm import BluetoothComm
from lib.comm.print_comm import PrintComm


def create_comm(mode: str, **kwargs):
    if mode == 'wifi':
        return WifiComm(kwargs['ip'])
    elif mode == 'bluetooth':
        return BluetoothComm(kwargs['port'], 9600)
    else:
        return PrintComm()