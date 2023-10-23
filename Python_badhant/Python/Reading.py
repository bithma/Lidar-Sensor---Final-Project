import serial
import math


s = serial.Serial("COM12",115200)


print("Opening: " + s.name)

s.write(b'1')


ang = 0
xdistance = 0


file = open('number_measurements.xyz', 'w')

for a in range(10):

    ang = 0

    for b in range(64):
  
        a = s.readline()

        distance = float(a.decode())

        ang = ang + (45/8)
    
        x_dis = xdistance
        y_dis = distance*(math.cos(math.pi*(ang/180)))
        z_dis = distance*(math.sin(math.pi*(ang/180)))
        x_dis, y_dis, z_dis = map(float,(x, y, z))

        file.write("{}\t {}\t {}\n".format(x,y,z))

        print(distance , " " , x_dis , " " , y_dis , " " ,z_dis)

    xdistance = xdistance + 100




print("Closing: " + s.name)
s.close();