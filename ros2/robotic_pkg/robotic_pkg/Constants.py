class Code:
    START = 0xFA
    STOP  = 0xFB

    # Transmit messages
    CONFIG = 0x43
    INIT  = 0x49
    SPEED = 0x53

    # Receive messages
    ACK   = 0x41
    ODOM  = 0x4F

class Wheel:
    DIAMETER = 0.007
    TRACK = 0.16642