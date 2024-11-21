import serial
import time

def create_modbus_frame(frame_list, chk_s='LRC', output="ascii", frame_info = 't'):
    frame_len = len(frame_list)
    frame_list_h = [None] * frame_len

    for i in range(0, frame_len):
        if type(frame_list[i]) == int:
            frame_list_h[i] = hex(frame_list[i])
            if len(frame_list_h[i]) <= 3:
                frame_list_h[i] = "0" + frame_list_h[i][2:].upper()
            else:
                frame_list_h[i] = frame_list_h[i][2:].upper()
        elif type(frame_list[i]) == str:
            frame_list_h[i] = ''.join([f"{ord(char):02X}" for char in frame_list[i]])

    ST = frame_list_h[0]
    DA = frame_list_h[1]
    CMD = frame_list_h[2]
    Data = frame_list_h[3]
    END = frame_list_h[4].upper()
    if frame_len == 6:
        END = END +" " + frame_list_h[5].upper()
   
    total = DA + CMD + Data
    total_h = [total[i:i+2] for i in range(0, len(total), 2)]
    total_sum = sum(int(hex_value, 16) for hex_value in total_h)

    if chk_s.lower() == "lrc":
        LRC_inv_b = bin((~total_sum + 1) & 0xFF)
        LRC_inv_h = hex(int(LRC_inv_b, 2))
        LRC_inv_h_1 = hex(ord(LRC_inv_h[2]))
        LRC_inv_h_2 = hex(ord(LRC_inv_h[3]))
        LRC_inv_h_1 = int(LRC_inv_h_1, 16)
        LRC_inv_h_2 = int(LRC_inv_h_2, 16)
        if LRC_inv_h_1 >= 97:
            LRC_inv_h_1 = LRC_inv_h_1 - 32
        if LRC_inv_h_2 >= 97:
            LRC_inv_h_2 = LRC_inv_h_2 - 32
        chk_h_1 = hex(LRC_inv_h_1)
        chk_h_2 = hex(LRC_inv_h_2)
    elif chk_s.lower() == "sum":
        SUM_b = bin(total_sum & 0xFF)
        SUM_h = hex(int(SUM_b, 2))
        chk_h_1 = hex(ord(SUM_h[2]))
        chk_h_2 = hex(ord(SUM_h[3]))

    frame_tmp = f'{ST}{DA}{CMD}{Data}{chk_h_1[2:]}{chk_h_2[2:]}{END}'

    if output.lower() in ["ascii", "asc"]:
        frame = bytes.fromhex(frame_tmp)
    elif output.lower() in ["h"]:
        frame = frame_tmp
    
    if frame_info == 't':
        print(frame)

    return frame

def setup_serial(port='/dev/ttyUSB0', baudrate=9600):
    return serial.Serial(port, baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

if __name__ == "__main__":
    ser_plc = setup_serial()
    frame = create_modbus_frame([0x3A, '01', "03", "00010008B3", 0x0D, 0x0A], 'lrc')
    ser_plc.write(frame)
    time.sleep(1)
