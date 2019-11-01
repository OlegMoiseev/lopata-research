import json
import pandas as pd
import serial
import time


def get_data(ard):
    start_time = time.time()
    start_flow = 'g\n'.encode()
    stop_flow = 's\n'.encode()
    num_packets = 5000
    ard.write(start_flow)
    data = []
    for i in range(num_packets):
        packet = [float(n) for n in ard.readline().decode()[:-1].split('\t')]
        packet.append(time.time() - start_time)
        data.append(packet)
    ard.write(stop_flow)
    stop = False
    while not stop:
        if "stop" in ard.readline().decode():
            stop = True
    return data


arduino = serial.Serial('/dev/ttyUSB0', 115200)  # Establish the connection on a specific port
skip = True
while skip:
    line = arduino.readline()
    if "OK".encode() in line:
        skip = False

print("Started")
time.sleep(3)
print("Go!")

path_settings = 'settings.json'
with open(path_settings) as settings_file:
    settings = json.load(settings_file)
    parameters = ['1', '5', '8']

    for i in range(len(settings[parameters[0]])):
        print(i)
        for j in range(len(settings[parameters[1]])):
            for k in range(len(settings[parameters[2]])):
                f_name = str(int(settings[parameters[0]][i], 2)) + '_' \
                         + str(int(settings[parameters[1]][j], 2)) + '_' \
                         + str(int(settings[parameters[2]][k], 2)) + '.csv'
                par_1 = 'w ' + str(parameters[0]) + ' ' + str(int(settings[parameters[0]][i], 2)) + '\n'
                par_2 = 'w ' + str(parameters[1]) + ' ' + str(int(settings[parameters[1]][j], 2)) + '\n'
                par_3 = 'w ' + str(parameters[2]) + ' ' + str(int(settings[parameters[2]][k], 2)) + '\n'

                arduino.write(par_1.encode())
                arduino.write(par_2.encode())
                arduino.write(par_3.encode())

                data = get_data(arduino)
                df = pd.DataFrame(data, columns=['a_x', 'a_y', 'a_z', 't'])
                df.to_csv('data/rest/' + f_name, index_label='Index')

    arduino.close()
