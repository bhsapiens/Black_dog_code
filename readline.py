'''fp = open("setpoint.txt","rw+")
line = fp.readline()
line=line.translate(None,'[]\n ')
motor=line.split(',')
print int(motor[0])'''
filepath = 'setpoint.txt'
with open(filepath) as fp:
    for cnt, line in enumerate(fp):
#        print("Line {}: {}".format(cnt, line))
#        line = fp.readline()
         line=line.translate(None,'[]\n ')
         motor=line.split(',')
         print motor[1]
