import serial
import time
import csv

arduino = serial.Serial('/dev/ttyACM0', 115200)  # Establish the connection on a specific port
skip = True
while skip:
    line = arduino.readline()
    if "OK".encode() in line:
        skip = False

print("Started")

par_1 = 'w 1 01100000\n'  # 96
par_2 = 'w 1 00100000\n'  # 32
arduino.write(par_1.encode())
arduino.write(par_2.encode())

a_x, a_y, a_z, t = [], [], [], []
arduino.write('g'.encode())
print('Start moving!')
try:
    start_time = time.time()
    while True:
        st = arduino.readline()[:-1].decode().split()
        ax, ay, az = float(st[0]), float(st[1]), float(st[2])
        a_x.append(ax)
        a_y.append(ay)
        a_z.append(az)
        t.append(time.time() - start_time)
except:
    print("End moving")
    arduino.close()

    with open('data/static/static.csv', 'w', newline='') as csvfile:
        datawriter = csv.writer(csvfile)
        datawriter.writerow(['a_x', 'a_y', 'a_z', 't'])
        for i in range(len(a_x)):
            datawriter.writerow([a_x[i], a_y[i], a_z[i], t[i]])

