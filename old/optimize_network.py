from nanovna import NanoVNAV2
import skrf
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import motor1
import motor2
import motor3
import scipy.optimize as spo
import threading
import os

matplotlib.rcParams['toolbar'] = 'None'

### User functions ###
#a = input('press enter to continue')
#motor1.drive(goto=200)
#a = input('press enter to continue')

### Input parameters ###

# Goals
f_goal = 3420e6
sign = -1 #minimize S21: sign=1, maximize S21: sign=-1

# Motor bounds
bnds = ((0,45000), (0,45000), (0,45000))
##bnds = (0,45000)
# Initial go-to position (begin midway)
##goto_start = (bnds[0][1]-bnds[0][0])/2
goto_start = [22500, 22500, 22500]

# Excitation parameters
f_start = 3300e6 # Hz
f_stop = 3500e6 # Hz
sweep_pts = 101 # 101 / Number of points to sweep

### Clear previous data/logs ###
files = ['Current.s2p', 'Initial.s2p', 'Best.s2p', 'log.txt']

for file in files:
	if os.path.isfile(file):
		os.remove(file)
	else:
		pass

### Device initiation ###
# Initiate device
nv = NanoVNAV2()

# Set frequency range
nv.set_sweep(f_start, f_stop)
nv.fetch_frequencies()

# Measure initial S11 and S21
s11 = nv.data(0)
s21 = nv.data(1)

f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
f = skrf.Frequency.from_f(f_arr, unit='MHz')
s = np.zeros((len(f), 2, 2), dtype = 'complex_')
s[:,0,0] = s11
s[:,0,1] = np.ones_like(s11)*np.nan
s[:,1,0] = s21
s[:,1,1] = np.ones_like(s11)*np.nan
ntw_init = skrf.Network(frequency=f, s=s)

ntw_init.write_touchstone('Initial')
ntw_init = skrf.Network('Initial.s2p')

ntw_best = ntw_init
ntw_best.write_touchstone('Best')
ntw_best = skrf.Network('Best.s2p')

f_goal /= 1e6
difference_array = np.absolute(f_arr-f_goal)
idx = (np.abs(f_arr - f_goal)).argmin()

if f_goal == int(f_goal):
	f_goal_label = str(int(f_goal))
else:
	f_goal_label = str(f_goal)

positions = np.empty((0,3))
s11_progress = np.array([])
s21_progress = np.array([])

plt.ion()

fig = plt.figure('Network Optimizer')
plt.tight_layout()
mng = plt.get_current_fig_manager()
mng.full_screen_toggle()

flag = True
def s21_mag(goto):
	global positions, best, ntw_best, s, s11_progress, s21_progress, f_goal_label, flag
	try:
		# Float to Int
		goto = [int(pos) for pos in goto]

		goto1 = goto[0]
		goto2 = goto[1]
		goto3 = goto[2]
		if not flag:
			positions = np.append(positions, [[goto1, goto2, goto3]], axis=0)
		with open('log.txt', 'a') as log:
			log.write('==================================\n')
			log.write('Shifting motor1 to position: '+str(goto1)+'\n')
			log.write('Shifting motor2 to position: '+str(goto2)+'\n')
			log.write('Shifting motor3 to position: '+str(goto3)+'\n')
		threads = [
			threading.Thread(target=motor1.drive, args=(goto1,)),
			threading.Thread(target=motor2.drive, args=(goto2,)),
			threading.Thread(target=motor3.drive, args=(goto3,))
		]
		for thread in threads:
			thread.start()
		for thread in threads:
			thread.join()

		prev_iter = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])

		# Measure current S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.ones_like(s11)*np.nan # S12 (incompatible with NanoVNA)
		s[:,1,0] = s21
		s[:,1,1] = np.ones_like(s11)*np.nan # S22 (incompatible with NanoVNA)

		if not flag:
			s11_progress = np.append(s11_progress, skrf.mathFunctions.complex_2_db(s[idx,0,0]))
			s21_progress = np.append(s21_progress, skrf.mathFunctions.complex_2_db(s[idx,1,0]))

		ntw = skrf.Network(frequency=f, s=s)

		ntw.write_touchstone('Current')
		ntw = skrf.Network('Current.s2p')

		if flag:
			best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
			ntw_best = ntw
			ntw_best.write_touchstone('Best')
			ntw_best = skrf.Network('Best.s2p')
			flag = False
		else:
			if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < best:
				best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
				ntw_best = ntw
				ntw_best.write_touchstone('Best')
				ntw_best = skrf.Network('Best.s2p')

		ax1 = plt.subplot(221) # Initial vs Best
		ax2 = plt.subplot(223) # Current
		ax3 = plt.subplot(224) # Positions vs Iteration
		ax4 = plt.subplot(222) # |S11|, |S21| vs Iteration (maybe with axhline for Initial for reference?)

		plt.subplots_adjust() #hspace=0.075)

		# Initial vs Best
		ax1.set_title('Optimization Progress')

		ax1.axvline(x=f_goal*1e6, color='#9467bd', linestyle='--')
		ntw_init.plot_s_db(n=0,m=0,ax=ax1)
		ntw_init.plot_s_db(n=0,m=1,ax=ax1)

		#if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < prev_iter:
		#	ntw_best = ntw
		#	ntw_best.write_touchstone('best')
		#	ntw_best = skrf.Network('best.s2p')

		ntw_best.plot_s_db(n=0,m=0,ax=ax1)
		ntw_best.plot_s_db(n=0,m=1,ax=ax1)

		ax1.set_ylim(-40,0)
		ax1.grid()
		ax1.legend(loc='lower left')

		# Current
		ax2.axvline(x=f_goal*1e6, color='#9467bd', linestyle='--')
		ntw.plot_s_db(n=0,m=0,ax=ax2)
		ntw.plot_s_db(n=0,m=1,ax=ax2)
		ax1.get_shared_x_axes().join(ax1, ax2)
		ax1.set_xticklabels([])
		x_axis = ax1.axes.get_xaxis()
		x_axis.set_label_text('')
		x_label = x_axis.get_label()
		x_label.set_visible(False)
		ax2.set_ylim(-40,0)
		ax2.grid()
		ax2.legend(loc='lower left')

		# Positions vs Iteration #
		ax3.plot(range(len(positions)),positions[:,0],label='Motor 1') # Motor1
		ax3.plot(range(len(positions)),positions[:,1],label='Motor 2') # Motor2
		ax3.plot(range(len(positions)),positions[:,2],label='Motor 3') # Motor3
		ax3.set_xlim(left=0)
		ax3.grid()
		ax3.legend(loc='best')
		ax3.set_xlabel('Iteration #')
		ax3.set_ylabel('Position (steps)')

		# |S11|, |S21| vs Iteration #
		ax4.set_title('Motor Iterations')
		ax4.plot(range(len(s11_progress)),s11_progress,label='S11('+f_goal_label+' MHz)') # Motor1
		ax4.plot(range(len(s21_progress)),s21_progress,label='S21('+f_goal_label+' MHz)') # Motor2
		ax4.set_xlim(left=0)
		ax4.get_shared_x_axes().join(ax4, ax3)
		ax4.set_xticklabels([])
		x_axis = ax1.axes.get_xaxis()
		x_axis.set_label_text('')
		x_label = x_axis.get_label()
		x_label.set_visible(False)
		ax4.grid()
		ax4.legend(loc='best')
		#ax4.set_xlabel('Iteration #')
		ax4.set_ylabel('Magnitude (dB)')

		plt.show()
		plt.tight_layout()
		plt.draw()
		plt.pause(0.0001)
		plt.clf()
	except KeyboardInterrupt:
		raise ValueError('\nSweep terminated by user.')
		#return 1

	return sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])

# Optimization
try:
	# Initiate plot (goto=0 to begin plot)
	with open('log.txt', 'a') as log:
		log.write('=-=-=-=-=-=-=-=-=-=-=\n')
		log.write('Initiating motors...\n')
	s21_mag([0,0,0])
	result = spo.minimize(s21_mag, goto_start, method='Nelder-Mead', options={'disp': True}, bounds=bnds)
	##result = spo.minimize_scalar(s21_mag, options={'disp': True}, bounds=bnds) # only for single-variable functions
	#result = spo.brute(s21_mag, ranges=bnds, full_output=True, finish=spo.fmin, disp=True)
	#if result.success:
	#	print('Success!')
	#	print(f'Optimal goto position: {result.x}, |S21| = {result.fun}')
	#else:
	#	print('Sorry, could not find a maximum.')
except Exception as e:
	with open('log.txt', 'a') as log:
		#log.write(str(e))
		log.write('\nOptimization interrupted.\n')

reset = input('Would you like to reset motor positions [y/N]? ')

if reset == 'y':
	with open('log.txt', 'a') as log:
		log.write('Resetting motors back to initial positions (0 steps)...\n')

	threads = [
		threading.Thread(target=motor1.drive, args=(0,)),
		threading.Thread(target=motor2.drive, args=(0,)),
		threading.Thread(target=motor3.drive, args=(0,))
	]

	for thread in threads:
		thread.start()
	for thread in threads:
		thread.join()

	# GPIO cleanup
	motor1.cleanup()
	motor2.cleanup()
	motor3.cleanup()
