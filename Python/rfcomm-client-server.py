#! /usr/bin/python

# To change this license header, choose License Headers in Project Properties.
# To change this template file, choose Tools | Templates
# and open the template in the editor.

__author__ = "Ayuni Rashid"
__date__ = "$Feb 16, 2015 6:30 PM$"
## Taken from code at http://people.csail.mit.edu/albert/bluez-intro/
## Debugged, modified, and Threaded by Kellen Carey

import bluetooth
import sys
import threading
import socket

def listen_to_android(bt_sock, c_sock):#,
    while True:
        recvd = bt_sock.recv(1024);
        print "Received", recvd, "from android!"
        c_sock.send(recvd);

def listen_to_pc(bt_sock, c_sock):
    while True:
        recvc = c_sock.recv(1024);
        print "Received", recvc, "from PC!"
        bt_sock.send(recvc);

def rfcomm_client():
#    c_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#    c_sock.connect(('localhost', 9999))


    uuid = "0c65fe91-9412-498e-b6e7-1fcd3d3d2237" #secure uuid android app
    service_matches = bluetooth.find_service( uuid = uuid )

    if len(service_matches) == 0:
        print "couldn't find the FooBar service"
        return

    first_match = service_matches[0]
    port = first_match["port"]
    name = first_match["name"]
    host = first_match["host"]

    print "Connecting to \"%s\" on %s and port %d" % (name, host, port)

    sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

    sock.connect((host, port))
    
    android = threading.Thread(name='android2pc', target=listen_to_android, 
		args=(sock, c_sock, ))
    pc = threading.Thread(name='pc2android', target=listen_to_pc,
		args=(sock, c_sock, ))
    
    android.start()
    pc.start()

    while True:
        pass
    
#    flag = True
#    while flag is True:
#        sendData = raw_input("Enter input: ")
#        if sendData == "Stop":
#            flag = False
#        else:
#	    try:
#            	sock.send(sendData)
#    		data = sock.recv(1024)
#		print data
#            except bluetooth.btcommon.BluetoothError:
#		print 'ERROR HERE! :('
#		break
    
    sock.close()
    
if __name__ == "__main__":
    
    flag = True

    try:
        c_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        c_sock.connect(('localhost', 9999))
    except:
	print "Make sure to start the C Server!"

    while flag is True:
            rfcomm_client()    
