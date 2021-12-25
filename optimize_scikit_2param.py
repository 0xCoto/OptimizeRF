from nanovna import NanoVNAV2
import skrf
from skrf.media import Coaxial
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import gridspec
import time
from datetime import timedelta
import motor1
import motor2
#import motor3
import scipy.optimize as spo
import threading
import os
from skopt import gp_minimize
from scipy import interpolate
import sys


### Good config (with screwdriver inside): 43500, 17000, 0 [yields -3.9 dB at 3418 MHz] (see last pic in poco)

default_params = {
	'toolbar': 'None',
	'figure.dpi':  120,
	'figure.figsize': [4,3],
	'figure.subplot.left': 0.15,
	'figure.subplot.right': 0.9,
	'figure.subplot.bottom': 0.12,
	'axes.titlesize': 'medium',
	'axes.labelsize': 14 ,
	'ytick.labelsize': 'small',
	'xtick.labelsize': 'small',
	'legend.fontsize': 12,
	'legend.loc': 'best',
	'font.size': 16,
	'font.family': 'serif',
	'text.usetex': True, # LaTeX
}
plt.rcParams.update(default_params)

### Input parameters ###
# Goals
f_goal = 3418e6 #3420e6
sign = -1 #minimize S21: sign=1, maximize S21: sign=-1

# Maximum number of iterations
max_calls = 50
exp_ratio = 0.1

# Motor bounds
bnds = [(0,45000), (0,45000)]

# Excitation properties
f_start = 3400e6 # Hz
f_stop = 3500e6 # Hz
sweep_pts = 101 # Number of points to sweep

f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
f = skrf.Frequency.from_f(f_arr, unit='MHz')

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

calibrated = False
load_cal = 'n'
#load_cal = input('Would you like to import a local measurement calibration from file? [Y/n]: ')
if load_cal != 'n' and load_cal != 'N':
	calibrated = True
else:
	calibrate = 'n'
	#calibrate = input('Would you like to perform a measurement calibration? [y/N]: ')
	if calibrate == 'y' or calibrate == 'Y':
		### OPEN ###
		input('Connect OPEN to Port 1 and press Enter.')
		# Measure S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
		s[:,1,0] = s21
		s[:,1,1] = np.ones_like(s11) #np.ones_like(s11)*np.nan
		ntw_init = skrf.Network(frequency=f, s=s)

		ntw_init.write_touchstone('measured/open')

		### SHORT ###
		input('Connect SHORT to Port 1 and press Enter.')
		# Measure S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
		s[:,1,0] = s21
		s[:,1,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
		ntw_init = skrf.Network(frequency=f, s=s)

		ntw_init.write_touchstone('measured/short')

		### LOAD ###
		input('Connect LOAD to Port 1 and press Enter.')
		# Measure S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
		s[:,1,0] = s21
		s[:,1,1] = np.ones_like(s11) #np.ones_like(s11)*np.nan
		ntw_init = skrf.Network(frequency=f, s=s)

		ntw_init.write_touchstone('measured/load')

		### THRU ###
		input('Connect THRU between Port 1 & 2 and press Enter.')
		# Measure S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.ones_like(s11) #np.ones_like(s11)*np.nan
		s[:,1,0] = s21
		s[:,1,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
		ntw_init = skrf.Network(frequency=f, s=s)

		ntw_init.write_touchstone('measured/thru')
		calibrated = True
if calibrated:
	"""
	### Create ideals ###
	s = np.arange(101*2*2, dtype='complex_')
	s = s.reshape((101,2,2))
	# Open
	s_open = 1*np.ones_like(s)
	s_open[:,0,1] = 0 #np.nan+np.nan*1j
	s_open[:,1,0] = 0
	s_open[:,1,1] = 1 #np.nan+np.nan*1j
	ntw_open = skrf.Network(frequency=f, s=s_open)
	ntw_open.write_touchstone('ideal/open')
	# Short
	s_short = -1*np.ones_like(s)
	s_short[:,0,1] = 0 #np.nan+np.nan*1j
	s_short[:,1,0] = 0
	s_short[:,1,1] = 1 #np.nan+np.nan*1j
	ntw_short = skrf.Network(frequency=f, s=s_short)
	ntw_short.write_touchstone('ideal/short')
	# Load
	s_load = 0.0001*np.ones_like(s)
	s_load[:,0,1] = np.nan+np.nan*1j
	s_load[:,1,0] = 0
	s_load[:,1,1] = np.nan+np.nan*1j
	ntw_load = skrf.Network(frequency=f, s=s_load)
	ntw_load.write_touchstone('ideal/load')
	# Thru
	s_thru = 0.0001*np.ones_like(s)
	s_thru[:,0,1] = 1+0j #np.nan+np.nan*1j
	s_thru[:,1,0] = 1+0j
	s_thru[:,1,1] = 0.0001 #np.nan+np.nan*1j
	ntw_thru = skrf.Network(frequency=f, s=s_thru)
	ntw_thru.write_touchstone('ideal/thru')

	# a list of Network types, holding 'ideal' responses
	my_ideals = [
		skrf.Network('ideal/thru.s2p'),
		skrf.Network('ideal/short.s2p'),
		skrf.Network('ideal/open.s2p'),
		skrf.Network('ideal/load.s2p'),
		]
	"""
	### Create ideals ###
	coax = Coaxial(frequency=f, z0=50)
	my_ideals = [
		coax.short(nports=2),
		coax.open(nports=2),
		coax.match(nports=2),
		coax.thru()
	]
	my_measured = [
		skrf.Network('measured/short.s2p'),
		skrf.Network('measured/open.s2p'),
		skrf.Network('measured/load.s2p'),
		skrf.Network('measured/thru.s2p'),
		]

	## Create SOLT instance
	cal = skrf.calibration.TwoPortOnePath(
		measured = my_measured,
		ideals = my_ideals,
		)
	# Run calibration algorithm
	cal.run()
	input('Calibration complete. Press Enter to begin DUT optimization.')

# Measure initial S11 and S21
s11 = nv.data(0)
s21 = nv.data(1)

f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
f = skrf.Frequency.from_f(f_arr, unit='MHz')
s = np.zeros((len(f), 2, 2), dtype = 'complex_')
s[:,0,0] = s11
s[:,0,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan
s[:,1,0] = s21
s[:,1,1] = np.ones_like(s11) #np.ones_like(s11)*np.nan
ntw_init = skrf.Network(frequency=f, s=s)
if calibrated:
	cal.apply_cal((ntw_init,ntw_init))

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

positions = np.empty((0,len(bnds)))
s11_progress = np.array([])
s21_progress = np.array([])
grid_x, grid_y = np.mgrid[0:45000:250j, 0:45000:250j]

plt.ion()

fig = plt.figure('Network Optimizer', figsize=(16,10))

spec = gridspec.GridSpec(ncols=3, nrows=2,
                         width_ratios=[2.15, 2.15, 0.9]) #, hspace=0.075)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()

start_time = time.time()
best = None
flag = True

def s21_mag(goto):
	global start_time, positions, best, ntw_best, s, grid_x, grid_y, s11_progress, s21_progress, f_goal_label, flag
	try:
		# Float to Int
		goto = [int(pos) for pos in goto]

		goto1 = goto[0]
		goto2 = goto[1]
		#goto3 = goto[2]
		if not flag:
			positions = np.append(positions, [[goto1, goto2]], axis=0)
		with open('log.txt', 'a') as log:
			log.write('==================================\n')
			log.write('Shifting motor1 to position: '+str(goto1)+'\n')
			log.write('Shifting motor2 to position: '+str(goto2)+'\n')
			#log.write('Shifting motor3 to position: '+str(goto3)+'\n')
		"""threads = [
			threading.Thread(target=motor1.drive, args=(goto1,)),
			threading.Thread(target=motor2.drive, args=(goto2,))
			#threading.Thread(target=motor3.drive, args=(goto3,))
		]
		for thread in threads:
			thread.start()
		for thread in threads:
			thread.join()"""

		prev_iter = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])

		# Measure current S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.zeros_like(s11) #np.ones_like(s11)*np.nan # S12 (incompatible with NanoVNA)
		s[:,1,0] = s21
		s[:,1,1] = np.ones_like(s11) #np.ones_like(s11)*np.nan # S22 (incompatible with NanoVNA)

		if not flag:
			s11_progress = np.append(s11_progress, skrf.mathFunctions.complex_2_db(s[idx,0,0]))
			s21_progress = np.append(s21_progress, skrf.mathFunctions.complex_2_db(s[idx,1,0]))


		ntw = skrf.Network(frequency=f, s=s)
		if calibrated:
			cal.apply_cal((ntw,ntw))

		ntw.write_touchstone('Current')
		ntw = skrf.Network('Current.s2p')

		if flag:
			best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
			ntw_best = ntw
			ntw_best.write_touchstone('Best')
			ntw_best = skrf.Network('Best.s2p')
		else:
			if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < best:
				best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
				ntw_best = ntw
				ntw_best.write_touchstone('Best')
				ntw_best = skrf.Network('Best.s2p')

		ax1 = fig.add_subplot(spec[0]) #plt.subplot(231) # Initial vs Best
		ax2 = fig.add_subplot(spec[3]) #plt.subplot(234) # Current
		ax3 = fig.add_subplot(spec[4]) #plt.subplot(235) # Positions vs Iteration
		ax4 = fig.add_subplot(spec[1]) #plt.subplot(232) # |S11|, |S21| vs Iteration
		ax5 = fig.add_subplot(spec[1:,-1]) #plt.subplot(236) # Motor1 vs Motor2 (S21)
		ax6 = fig.add_subplot(spec[2]) #plt.subplot(233) # Motor1 vs Motor2 (S11)

		#plt.subplots_adjust(wspace=0,hspace=0) #hspace=0.075)

		# Initial vs Best
		ax1.set_title('$\mathrm{Optimization \ Progress}$', fontsize=19)

		ax1.axvline(x=f_goal*1e6, color='#9467bd', linestyle='--')
		ntw_init.plot_s_db(n=0,m=0,ax=ax1)
		ntw_init.plot_s_db(n=0,m=1,ax=ax1)

		#if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < prev_iter:
		#	ntw_best = ntw
		#	ntw_best.write_touchstone('best')
		#	ntw_best = skrf.Network('best.s2p')

		ntw_best.plot_s_db(n=0,m=0,ax=ax1)
		ntw_best.plot_s_db(n=0,m=1,ax=ax1)

		ax1.set_xlabel('$\mathrm{Frequency \ (MHz)}$')
		ax1.set_ylabel('$\mathrm{Magnitude \ (dB)}$')
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
		ax2.set_ylabel('$\mathrm{Magnitude \ (dB)}$')
		ax2.set_ylim(-40,0)
		ax2.grid()
		ax2.legend(loc='lower left')

		# Positions vs Iteration #
		ax3.plot(range(len(positions)),positions[:,0],label='$x_{1} \ \mathrm{(Motor \ 1)}$') # Motor1
		ax3.plot(range(len(positions)),positions[:,1],label='$x_{2} \ \mathrm{(Motor \ 2)}$') # Motor2
		#ax3.plot(range(len(positions)),positions[:,2],label='Motor 3') # Motor3
		ax3.set_xlim(left=0)
		ax3.grid()
		ax3.legend(loc='best')
		ax3.set_xlabel('$\mathrm{Iteration \ \#}$')
		ax3.set_ylabel('$\mathrm{Position \ (steps)}$')

		# |S11|, |S21| vs Iteration #
		ax4.set_title('$\mathrm{Parameter \ Sequence}$', fontsize=19)
		ax4.plot(range(len(s11_progress)),s11_progress,label='$S_{11} \ \mathrm{('+f_goal_label+' \ \mathrm{MHz})}$') # Motor1
		ax4.plot(range(len(s21_progress)),s21_progress,label='$S_{21} \ \mathrm{('+f_goal_label+' \ \mathrm{MHz})}$') # Motor2
		ax4.set_xlim(left=0)
		ax4.get_shared_x_axes().join(ax4, ax3)
		ax4.set_xticklabels([])
		x_axis = ax1.axes.get_xaxis()
		x_axis.set_label_text('')
		x_label = x_axis.get_label()
		x_label.set_visible(False)
		ax4.grid()
		ax4.legend(loc='best')
		ax4.set_ylabel('$\mathrm{Magnitude \ (dB)}$')

		# |S21| vs x1 vs x2
		if not flag and len(positions) >= 4:
			grid_z2 = interpolate.griddata(positions, s21_progress, (grid_x, grid_y), method='cubic')
			ax5.imshow(grid_z2.T, extent=(bnds[0][0],bnds[0][1],bnds[1][0],bnds[1][1]), origin='lower')
			levels = np.arange(np.amin(s21_progress), np.amax(s21_progress), abs(np.amax(s21_progress)-np.amin(s21_progress))/3)
			ax5.contour(grid_z2.T, levels, colors='k', origin='lower', extent=(bnds[0][0],bnds[0][1],bnds[1][0],bnds[1][1]))
			ax5.plot(positions[:,0], positions[:,1], 'r.', ms=3)
		#ax5.annotate('$|S_{21}|$', xy=(10,30), size=13, ha='left', va='top', color='black')
		ax5.set_xlabel('$x_{1} \ \mathrm{(Motor \ 1)}$')
		ax6.set_xticks([])
		ax6.get_shared_x_axes().join(ax6, ax5)
		ax6.set_xticklabels([])
		x_axis = ax6.axes.get_xaxis()
		x_axis.set_label_text('')
		x_label = x_axis.get_label()
		x_label.set_visible(False)
		ax5.set_ylabel('$x_{2} \ \mathrm{(Motor \ 2)}$')

		# |S11| vs x1 vs x2
		if not flag and len(positions) >= 4:
			grid_z2 = interpolate.griddata(positions, s11_progress, (grid_x, grid_y), method='cubic')
			ax6.imshow(grid_z2.T, extent=(bnds[0][0],bnds[0][1],bnds[1][0],bnds[1][1]), origin='lower')
			levels = np.arange(np.amin(s11_progress), np.amax(s11_progress), abs(np.amax(s11_progress)-np.amin(s11_progress))/3)
			ax6.contour(grid_z2.T, levels, colors='k', origin='lower', extent=(bnds[0][0],bnds[0][1],bnds[1][0],bnds[1][1]))
			ax6.plot(positions[:,0], positions[:,1], 'r.', ms=3)
		ax5.set_xlabel('$x_{1} \ \mathrm{(Motor \ 1)}$')
		ax6.set_title('$\mathrm{Evaluation \ Heatmaps}$\n', fontsize=19)
		ax6.set_xlabel('$x_{1} \ \mathrm{(Motor \ 1)}$')
		ax6.set_ylabel('$x_{2} \ \mathrm{(Motor \ 2)}$')
		ax5.set_aspect('equal', anchor='N')
		ax6.set_aspect('equal', anchor='S')

		# Optimization log
		#ax6.set_title('$\mathrm{Optimization \ Log}$', fontsize=19)
		#ax6.annotate('[+] This is a test\n[*] And this is another line', xy=(0,0.5), size=16, ha='left', va='top', color='black')
		#ax6.axis('off')

		if flag:
			flag = False

		fig.text(x=0.91, y=0.786, s=r'$|S_{11}|$', fontsize=13)
		fig.text(x=0.91, y=0.506, s=r'$|S_{21}|$', fontsize=13)

		fig.text(x=0.862, y=0.926, s=r'$\mathrm{\underline{Optimize\textbf{RF}}}$', fontsize=26, color='seagreen')

		fig.text(x=0.862, y=0.180, s=r'$\mathrm{\underline{Optimization \ Status}}$', fontsize=16)

		if len(positions) >= int(exp_ratio*max_calls):
			search_space = 'Local \ (exploitation)'
		else:
			search_space = 'Global \ (exploration)'
		fig.text(x=0.842, y=0.140, s=r'$\mathrm{Search \ space: '+search_space+'}$', fontsize=14)
		percentage = str(round((len(positions)/max_calls), 1))
		fig.text(x=0.842, y=0.110, s=r'$\mathrm{Progress: '+str(len(positions))+'/'+str(max_calls)+'\ ('+percentage+'\%)}$', fontsize=14)
		runtime = round(time.time()-start_time, 1)
		fig.text(x=0.842, y=0.080, s=r'$\mathrm{Runtime: '+str(timedelta(seconds=runtime))+'}$', fontsize=14)

		plt.tight_layout(pad=0.7)
		plt.show()
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
	result = gp_minimize(s21_mag, bnds, n_calls=max_calls, n_initial_points=int(exp_ratio*max_calls), initial_point_generator='lhs', verbose=True)
except Exception as e:
	with open('log.txt', 'a') as log:
		print(e)
		log.write(str(e))
		log.write('\nOptimization interrupted.\n')
plt.savefig('results.png')
#reset = input('Would you like to reset motor positions [y/N]? ')
reset = 'y'
if reset == 'y':
	with open('log.txt', 'a') as log:
		log.write('Resetting motors back to initial positions (0 steps)...\n')

	threads = [
		threading.Thread(target=motor1.drive, args=(0,)),
		threading.Thread(target=motor2.drive, args=(0,))
		#threading.Thread(target=motor3.drive, args=(0,))
	]

	for thread in threads:
		thread.start()
	for thread in threads:
		thread.join()

	# GPIO cleanup
	motor1.cleanup()
	motor2.cleanup()
	#motor3.cleanup()
