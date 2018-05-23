import csv
import numpy as np



def filter(line):
  addr=int(line[0])
  data = eval(line[1])
  motor = int(line[3])
  dir = int(line[4])
  angle = int(line[5])
  return addr, data, motor, dir, angle
A = open('Black_dog_contact_map.csv', 'rb')
#dialect = csv.Sniffer().sniff(A.read())
sp = csv.reader(A, delimiter='|' )
i = 0
A = np.zeros([16,12])
#print A
data = [[0]*12]*16
A = data[2][2]
for row in sp:
     row=map(lambda x: x.strip(),row)
     [addr, Data, motor, dir, angle] = filter(row)
     print addr, Data, motor, dir, angle
     data[i][0] = addr
     data[i][ 1] = Data
     data[i][3] = motor
     data[i] [5] = angle
     data[i][ 4] = dir
     i +=1
print data
