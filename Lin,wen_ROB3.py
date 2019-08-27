#imports
import serial
import time
import math

X_init = 290
Y_init = 0
Z_init = 10

l1 = 271
l2 = 200
l3 = 130
width = 50
height = 170
class ROB3(object):
	def check_device_name(self):
		ser=serial.Serial('/dev/ttyS0')
		ser.baudrate=2400
		serial.timeout=0.5
		return ser
	def Init(self,ser):
		ser.write(b'\x20')
		# ROB3 should answer x15 or xF1
		x=ser.read()
		print(x)
		
	def initial_position(self,ser):
		# axis 1
		ser.write(b'\x00' b'\x80' b'\x03')
		# axis 2
		ser.write(b'\x01' b'\x64' b'\x03')
		# axis 3
		ser.write(b'\x02' b'\xA0' b'\x03')
		# axis 4
		ser.write(b'\x03' b'\xA0' b'\x03')
		# axis 5
		ser.write(b'\x04' b'\x80' b'\x03')
		# axis 6 (don’t operate with printhead)
		ser.write(b'\x04' b'\x00' b'\x03')
		# leave time to settle
		time.sleep(3)
		
	def toggle_drive_switch(self,ser):
		ser.write(b'\x11' b'\x40' b'\x03')
		time.sleep(1)
		ser.write(b'\x12' b'\x00' b'\x03')
		# wait to be heated
		time.sleep(9)
		
	def printhead_on(self,ser):
		ser.write(b'\x11' b'\x40' b'\x03')
		print("Printhead on")
		
	def printhead_off(self,ser):
		ser.write(b'\x12' b'\x00' b'\x03')
		print("Printhead off")
		
		
	def kinematic(self,x,y,z,ser):	
		z = z + height
		lengthXY = math.sqrt(x**2 + y**2) - width
		angle = math.atan2(y,x)
		x = math.cos(angle) * lengthXY 
		y = math.sin(angle) * lengthXY
		length = math.sqrt(x**2 + y**2 + (z-l1)**2)          
		#theta1
		radian1 = math.degrees(math.atan2(y, x))
		#theta2
		ra2 = (l3**2-l2**2-length**2)/(-(2*l2*length))
		radi2 = (math.pi/2) - math.acos(ra2)-math.asin((z-l1)/length)
		radian2 = math.degrees(radi2) - 22
		#theta3
		radian3 = math.pi - math.acos((l2*l2+l3*l3-length*length)/(2*l2*l3))
		radian3 = math.degrees(radian3)   
		#theta4
		radian4 = 180 - (radian2 + radian3) + 90
				
		return radian1, radian2, radian3, radian4
	
	def get_angle(self,x, y, z, ser):
		angle1, angle2, angle3, angle4 = self.kinematic(x, y, z,ser)
		   
		angle1_ = round(angle1 + 80 * 1.6)
		angle2_ = round(angle2 * 2.56)
		angle3_ = round(angle3 * 2.56)
		angle4_ = round(angle4 * 1.28)
		ser.write(b'\x7F')
			  
		# axis 1
		ser.write(bytes([angle1_]))
		# axis 2
		ser.write(bytes([angle2_])) 
		# axis 3
		ser.write(bytes([angle3_]))
		# axis 4
		ser.write(bytes([angle4_]))

		ser.write(bytes([128]))

		ser.write(b'\x00')
		ser.write(b'\x2D' b'\x2D' b'\x16' b'\x16' b'\x16' b'\x16' b'\x03') 
		temp = ser.read()
		#wait for term signal
		while (temp != b'z') and (temp != b'\x03'):
			temp = ser.read()
		
	def draw_graphic(self,ser):
		self.printhead_off(ser)
		time.sleep(10)
		x = X_init
		y = Y_init + 20
		z = Z_init
	
		#self.printhead_on(ser)
		time.sleep(5)
		print("start ")
		#x = X_init + 20
		#self.printhead_on(ser)
		while z< (Z_init + 10):
			x = X_init
			y = Y_init + 20
			self.printhead_on(ser)
			while x > X_init - 20 and x <= X_init :
				while y <=60 and y >= 20:
					while x > X_init - 20:
						while y <= 60 and y >= 20:
							y = y + 0.2
							print(y)
							self.get_angle(x,y,z,ser)
						x = x - 0.2
						self.get_angle(x,y,z,ser)
					y = y - 0.2
					print(y)
					self.get_angle(x,y,z,ser)
				x = x + 0.2
				self.get_angle(x,y,z,ser)	
			z = z + 1
		self.printhead_off(ser)
	def close(self,ser):
		ser.close
                                                       
def main():
        rob3 = ROB3()
        ser=rob3.check_device_name()
        print("name")
        rob3.Init(ser)
        print("init")
        rob3.initial_position(ser)
        print("position")
        rob3.toggle_drive_switch(ser)
        print("drive")
        rob3.draw_graphic(ser)
        print("finish draw")
        rob3.close(ser)
        print("Bye")
    
if __name__ == '__main__':
    main()		

	
