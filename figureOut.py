#Version 2.0 well with CartDemoPlugin from 9.0 awards.
import matplotlib.pyplot as plt
import re
tmp_t = []
vel_curr = []
vel_target = []
with open('pidOut.csv','r+') as f:
	for line in f:
		temp = map(float, re.findall(r'\d+', line)) #find all number from line
		tmp_t.append(float(line.split('\t')[0]))
		vel_curr.append(float(line.split('\t')[2]))
		vel_target.append(float(line.split('\t')[10]))
plt.figure(1)                # the first figure
plt.plot(tmp_t, vel_target,'g', tmp_t, vel_curr, 'r')
plt.title('Velocity Response')   
plt.xlabel('Time(0.1s interval)')
plt.ylabel('Velocity(m/s)')
plt.axis([0, 160, -5, 15])
plt.grid(True)
plt.show()
