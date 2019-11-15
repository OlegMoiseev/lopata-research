import serial
import time
import csv

arduino = serial.Serial('/dev/ttyUSB0', 115200)  # Establish the connection on a specific port

skip = False
for i in range(10):
    arduino.readline()

a_x, a_y, a_z, t = [], [], [], []
start_time = time.time()
collecting_time = 10

print('Start moving!')
while not skip:  # 7 secs!!!
    st = arduino.readline()[:-1].decode().split()
    ax, ay, az = float(st[0]), float(st[1]), float(st[2])
    a_x.append(ax)
    a_y.append(ay)
    a_z.append(az)
    t.append(time.time() - start_time)
    if time.time() - start_time > collecting_time:
        skip = True
print("End moving")
arduino.close()

with open('data/hand_strait/data_strait_double.csv', 'w', newline='') as csvfile:
    datawriter = csv.writer(csvfile)
    datawriter.writerow(['a_x', 'a_y', 'a_z', 't'])
    for i in range(len(a_x)):
        datawriter.writerow([a_x[i], a_y[i], a_z[i], t[i]])
