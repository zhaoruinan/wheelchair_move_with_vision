from vision_2 import get_obj2w
import time
from threading import Timer
import socket
import random
PORT = 8089
HOST = '192.168.0.2'  # The server's hostname or IP address
#HOST = '127.0.0.1'
from datetime import datetime
from ctypes import *
SIZE_DATA_TCP_MAX  = 200
class Data(Union):
	_fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),("int7Arr", c_int * 7),("float63dArr", c_float * 63)]

write_buffer = (c_char* 1024)()  
def tcp_init():
	global client, send_data
	#client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_address = (HOST, PORT)
	#client.connect(server_address)
	send_data = Data()
def tcp_com():

	#'''
	global client
	global send_data
	start = datetime.now()
	try:
		task_data = get_obj2w()
		print("task_data",task_data)
		send_data.float63dArr[6]=task_data[0,0]
		send_data.float63dArr[7]=task_data[0,1]
		send_data.float63dArr[8]=task_data[0,2]
	except:
		pass
	#send_data.int7Arr[0]=random.randrange(1,100)
	send_data.float63dArr[0]=1.7
	memmove( write_buffer, send_data.byte,1024)
	#client.sendall(write_buffer)
	end = datetime.now()
	#labview_control = client.recv(40)
	#print(labview_control)


class RepeatingTimer(Timer): 
	def run(self):
		while not self.finished.is_set():
			self.function(*self.args, **self.kwargs)
			self.finished.wait(self.interval)
def main():

	tcp_init()
	t_tcp = RepeatingTimer(0.05, tcp_com)
	t_tcp.start()
#if __name__ =='__main__':
main()