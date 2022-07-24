import math
def ap():
	if yaw<1:
		return 0
	if yaw<2 and yaw>1:
		return 1
	if yaw > 2 and yaw < 3.14:
		return 2
	if yaw <-1 and yaw > -2:
		return 3
	if yaw > -3.14 and yaw < -2:
		return 2
	else:
		return 0

def turn(ang):
	pose= [0,0]

	pick = [0, 90, 180, 270]

	if ang == 90:
		f = (pose[1] + 90)/90
		del pose[0]
		pose.append((f%4)*90)
		if (f%4) != 3:
			rad = pick[f%4]*math.pi/180
		if (f%4) == 3:
			rad = -90*math.pi/180
		print"1st "
		print pose
		print rad


	if ang == -90:
		f = (pose[1] - 90)/90
		del pose[0]
		pose.append((f%4)*90)
		if f >=0:
			if (f%4) == 3:
				rad = -90*math.pi/180
			else:
				rad = pick[f%4]*math.pi/180
			print "2nd"
			print pose
			print rad
		if f < 0:
			if f%4 == 1:
				rad = 90*math.pi/180
				print 'above'
			else:			
				rad = -pick[(4-f)%4]*math.pi/180
				print 'this'
				print f%4
			print "3rd"
			print pose
			print rad


	if ang == 180:
		f = (pose[1] - 180)/90
		del pose[0]
		pose.append((f%4)*180)
		if f >=0:
			rad = pick[f%4]*math.pi/180
			print "180"
		if f < 0:
			rad = -pick[(4-f)%4]*math.pi/180
			print "-180"
		print pose
		print rad

turn(90)

