#Version 6.0 well with CartDemoPlugin from 12.0 awards.
import matplotlib.pyplot as plt
import re
tmp_t = []
vel_curr = []
vel_target = []
acc = []
x = []
z = []
y = []
theta_e = []
kp = []
ki = []
with open('pidOut.csv','r+') as f:
	for line in f:
		temp = map(float, re.findall(r'\d+', line)) #find all number from line
		tmp_t.append(float(line.split('\t')[0]))
		vel_curr.append(float(line.split('\t')[2]))
		vel_target.append(float(line.split('\t')[10]))
		acc.append(float(line.split('\t')[20]))
		x.append(float(line.split('\t')[12]))
		z.append(float(line.split('\t')[14]))
		y.append(float(line.split('\t')[16]))
		theta_e.append(float(line.split('\t')[18]))
		kp.append(float(line.split('\t')[22]))
		ki.append(float(line.split('\t')[24]))


plt.figure(1)                # the first figure
plt.subplot(2, 2, 1)				#Firt subplot in row
plt.plot(tmp_t, vel_target,'y', tmp_t, vel_curr, 'r')
plt.title('Velocity Response')   
plt.xlabel('Time(0.1s interval)')
plt.ylabel('Velocity(m/s)')
plt.axis([0, 160, -5, 15])
plt.grid(True)

#kp, ki with respect to x,z
plt.subplot(2, 2, 2)				#Second subplot in row
plt.plot(x, z, 'r', x, kp, 'g', x, ki,'y')
plt.title('kp, ki response')   
plt.xlabel('x')
plt.ylabel('z -red, kp -green, ki- yellow')
plt.axis([-32, 32, -5, 5])
plt.grid(True)

#x, y, z and theta_e
plt.subplot(2, 2, 3)				#Second subplot in row
plt.plot(x, z, 'r', x, theta_e,'g', x, y, 'y')
plt.title('z and theta_e with respect to x')   
plt.xlabel('x)')
plt.ylabel('z, theta_e')
plt.axis([-32, 32, 0, 10])
plt.grid(True)


#x, z and velcurrent
plt.subplot(2, 2, 4)				#Second subplot in row
plt.plot(x, z, 'r', x, vel_curr,'g',x, vel_target, 'y')
plt.title('z and vel_curr with respect to x')   
plt.xlabel('x)')
plt.ylabel('z, vel_curr')
plt.axis([-32, 32, 0, 10])
plt.grid(True)
plt.show()
