#!/usr/bin/env python
import rospy
import bluetooth

# Define the Bluetooth address of the ESP32 (replace with your ESP32's address)
target_bluetooth_address = "A0:B7:65:6B:67:A6"


sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_bluetooth_address, 1))

def sendToRobot(a,b,c):
    sock.send(b''+str(a).encode() +b','+str(b).encode() +b','+str(c).encode())	

def exctractData(data):
    try:
        array = data.decode().split(",")
        a,b,c = array[0],array[1],array[2]
        return a,b,c
        
    except Exception as e:
        print(e)
        return 0,0,0
        

if __name__ == '__main__':
    rospy.init_node('WifiCom')
    rate = rospy.Rate(0.5)  # 1Hz rate
    while not rospy.is_shutdown():
        a = 10
        b = 234
        c = 5678
        sendToRobot(a,b,c)
        print("Sent : "+str(a)+" "+str(b)+" "+str(c))
        try:
            data = sock.recv(1024)
            a,b,c = exctractData(data)
            print("Recived : "+str(a)+" "+str(b)+" "+str(c))
        except Exception as e:
            #print(e)
            continue
        rate.sleep()
