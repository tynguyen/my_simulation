#Version 17
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
gas_actual_signal = []
gas_limited_signal = []
brake_force = []
gas_force = []
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
		gas_actual_signal.append(float(line.split('\t')[26]))
		gas_limited_signal.append(float(line.split('\t')[28]))
		brake_force.append(float(line.split('\t')[30])/50)
		gas_force.append(float(line.split('\t')[4])/50)

plt.figure(1)                # the first figure
plt.subplot(2, 3, 1)				#Firt subplot in row
plt.plot(tmp_t, vel_target,'b', tmp_t, vel_curr, 'r', tmp_t, z, 'k',tmp_t, brake_force,'y', gas_force, 'm')
plt.title('Velocity Response, brake_force and Z with respect to time')   
plt.xlabel('Time(0.1s interval)')
plt.ylabel('Velocity(m/s), brake_force(N), Z(m)')
plt.axis([0, 310, -5, 30])
plt.grid(True)

#x, z, brake_force and velcurrent
plt.subplot(2, 3, 2)				#Second subplot in row
plt.plot(x, z, 'k', x, vel_curr,'r',x, vel_target, 'b')
plt.title('z and vel_response with respect to x')   
plt.xlabel('x)')
plt.ylabel('z, vel_curr, vel_target')
plt.axis([-160, 160, 0, 10])
plt.grid(True)


#kp, ki with respect to x,z
plt.subplot(2, 3, 3)				#Second subplot in row
plt.plot(x, z, 'k', x, kp, 'r', x, ki,'c')
plt.title('kp, ki response')   
plt.xlabel('x')
plt.ylabel('z -red, kp -green, ki- yellow')
plt.axis([-160, 160, -5, 5])
plt.grid(True)

#gas_control signal with respect to time
plt.subplot(2, 3, 4)				# Second row
plt.plot(tmp_t, gas_limited_signal,'y*', tmp_t, gas_actual_signal, 'r')
plt.title('Gas control signal respect to time')   
plt.xlabel('Time(0.1s interval)')
plt.ylabel('Gas_limited_signal, gas_actual_signal')
plt.axis([0, 310, 0, 5])
plt.grid(True)


#gas_control signal with respect to x and z
plt.subplot(2, 3, 5)				#Second subplot in row
plt.plot(x, z, 'k', x, gas_limited_signal,'y*',x, gas_actual_signal, 'r')
plt.title('gas_control signal with respect to x and z')   
plt.xlabel('x')
plt.ylabel('z, gas_limited_signal, gas_actual_signal')
plt.axis([-160, 160, 0, 2])
plt.grid(True)

#x, y, z and theta_e
plt.subplot(2, 3, 6)				#Second subplot in row
plt.plot(x, z, 'k', x, theta_e,'r', x, y, 'y')
plt.title('y, z and theta_e with respect to x')   
plt.xlabel('x')
plt.ylabel('z, y, theta_e')
plt.axis([-160, 160,-10, 10])
plt.grid(True)
plt.show()

