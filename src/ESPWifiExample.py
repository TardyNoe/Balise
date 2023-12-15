#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import Header
import struct


esp_ip = "10.42.0.202"
esp_port = 1234

local_ip = "10.42.0.1" 
local_port = 1234 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((local_ip, local_port))


def sendToRobot(a,b,c):	
    message = a.to_bytes(2, byteorder='big') + b.to_bytes(2, byteorder='big') + c.to_bytes(2, byteorder='big')
    sock.sendto(message, (esp_ip, esp_port))

def exctractData(data):
    try:
        a,b,c = struct.unpack('hhh',data)
        return a,b,c
        
    except Exception as e:
        print(e)
        return 0,0,0
        

if __name__ == '__main__':
    rospy.init_node('WifiCom')
    rate = rospy.Rate(30)  # 30Hz rate
    while not rospy.is_shutdown():
        a = 10
        b = 234
        c = 5678
        sendToRobot(a,b,c)
        print("Sent : "+str(a)+" "+str(b)+" "+str(c))
        try:
            sock.settimeout(0.1)
            data,addr = sock.recvfrom(1024)
            if len(data) == 6:
                a,b,c = exctractData(data)
                print("Recived : "+str(a)+" "+str(b)+" "+str(c))
        except Exception as e:
            #print(e)
            continue
        rate.sleep()
