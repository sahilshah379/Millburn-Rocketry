import matplotlib.pyplot as plt


def main():
	print('Whats the name of the file?')
	filepath = 'Data/' + input()
	x = []
	y = []
	with open(filepath) as fp:  
		line = fp.readline().strip()
		while line != 'LAUNCH':
			line = fp.readline().strip()
		line = fp.readline().strip()
		while line != 'LAND':
			if (line != 'PARACHUTE'):
				time = round(float(line.split(': ')[0]),3)
				altitude = round(float(line.split(': ')[1]),3)
				print('(%s,%s)' %(time,altitude))
				x.append(time)
				y.append(altitude)
			else:
				print('PARACHUTE')
			line = fp.readline().strip()
		plt.title('Rocket Flight')
		plt.xlabel('Time')
		plt.ylabel('Altitude')
		plt.plot(x,y)
		plt.show()

if __name__ == '__main__':
	main()
