import socket
import time
s = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
s.connect( ( 'localhost', 7779 ) )
#s.send( 'python says hello nc' )
while 1:
    print s.recv(100)
#    time.sleep(1)
