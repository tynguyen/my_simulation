#Version 3.0 well with CartDemoPlugin.cc 7.3.3
import matplotlib.pyplot as plt
import re
tmp_t = []
vel_curr = []
vel_target = []
vel_end = []
dis_target = []
dis_curr = []
with open('pidOut.csv','r+') as f:
	for line in f:
		temp = map(float, re.findall(r'\d+', line)) #find all number from line
		tmp_t.append(float(line.split('\t')[0]))
		vel_curr.append(float(line.split('\t')[2]))
		vel_target.append(float(line.split('\t')[12]))
		vel_end.append(float(line.split('\t')[14]))
		dis_curr.append(float(line.split('\t')[4]))
		dis_target.append(float(line.split('\t')[6]))

plt.figure(1)                # the first figure
plt.subplot(2, 1, 1)				#Firt subplot in row
plt.plot(tmp_t, vel_end,'go-',tmp_t,vel_target,'y', tmp_t, vel_curr, 'r')
plt.title('Velocity Response')   
plt.xlabel('Time(0.01s interval)')
plt.ylabel('Velocity(m/s)')
plt.axis([0, 160, -5, 15])
plt.grid(True)


plt.subplot(2, 1, 2)				#Second subplot in row
plt.plot(tmp_t, dis_target,'g-',tmp_t, dis_curr, 'r')
plt.title('Distance Response')   
plt.xlabel('Time(0.01s interval)')
plt.ylabel('Distance(m)')
plt.axis([0, 160, 0, 700])
plt.grid(True)
plt.show()
