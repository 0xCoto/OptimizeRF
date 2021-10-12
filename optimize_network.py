from nanovna import NanoVNAV2
import skrf
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import motor
import scipy.optimize as spo

### User functions ###
#a = input('press enter to continue')
#motor.drive(goto=200)
#a = input('press enter to continue')

### Input parameters ###

# Goals
f_goal = 1500e6
sign = -1 #minimize: sign=1, maximize: sign=-1

# Motor bounds
bnds = ((0,45000),)
##bnds = (0,45000)
# Initial go-to position (begin midway)
##goto_start = (bnds[0][1]-bnds[0][0])/2
goto_start = 20000
# Excitation parameters
f_start = 100e6 # Hz
f_stop = 3000e6 # Hz
sweep_pts = 101 # Number of points to sweep


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

ntw_init.write_touchstone('initial')
ntw_init = skrf.Network('initial.s2p')

ntw_best = ntw_init
ntw_best.write_touchstone('best')
ntw_best = skrf.Network('best.s2p')

f_goal /= 1e6
difference_array = np.absolute(f_arr-f_goal)
idx = (np.abs(f_arr - f_goal)).argmin()

plt.ion()

flag = True
def s21_mag(goto):
	global best, ntw_best, s, flag
	print('Shifting to position: '+str(goto[0]))
	goto = int(goto)
	motor.drive(goto)

	try:
		prev_iter = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])

		# Measure current S11 and S21
		s11 = nv.data(0)
		s21 = nv.data(1)

		f_arr = np.linspace(f_start/1e6, f_stop/1e6, num=sweep_pts)
		f = skrf.Frequency.from_f(f_arr, unit='MHz')
		s = np.zeros((len(f), 2, 2), dtype = 'complex_')
		s[:,0,0] = s11
		s[:,0,1] = np.ones_like(s11)*np.nan
		s[:,1,0] = s21
		s[:,1,1] = np.ones_like(s11)*np.nan
		ntw = skrf.Network(frequency=f, s=s)

		ntw.write_touchstone('current')
		ntw = skrf.Network('current.s2p')

		if flag:
			best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
			ntw_best = ntw
			ntw_best.write_touchstone('best')
			ntw_best = skrf.Network('best.s2p')
			flag = False
		else:
			if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < best:
				best = sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])
				ntw_best = ntw
				ntw_best.write_touchstone('best')
				ntw_best = skrf.Network('best.s2p')

		plt.figure(1)

		plt.subplot(211)
		ntw_init.plot_s_db(n=0,m=0)
		ntw_init.plot_s_db(n=0,m=1)

		#if sign*skrf.mathFunctions.complex_2_db(s[idx,1,0]) < prev_iter:
		#	ntw_best = ntw
		#	ntw_best.write_touchstone('best')
		#	ntw_best = skrf.Network('best.s2p')

		ntw_best.plot_s_db(n=0,m=0)
		ntw_best.plot_s_db(n=0,m=1)
		plt.ylim(-100,0)

		plt.subplot(212)
		ntw.plot_s_db(n=0,m=0)
		ntw.plot_s_db(n=0,m=1)

		plt.ylim(-100,0)
		plt.show()
		plt.draw()
		plt.pause(0.0001)
		plt.clf()
	except KeyboardInterrupt:
		print('\nSweep terminated by user.')
		return 1

	return sign*skrf.mathFunctions.complex_2_db(s[idx,1,0])

# Optimization
try:
	#result = spo.minimize(s21_mag, goto_start, options={"disp": True}, bounds=bnds)
	#result = spo.minimize_scalar(s21_mag, options={"disp": True}, bounds=bnds)
	result = spo.brute(s21_mag, ranges=bnds, full_output=True, finish=spo.fmin, disp=True)
	#if result.success:
	#	print("Success!")
	#	print(f"Optimal goto position: {result.x}, |S21| = {result.fun}")
	#else:
	#	print("Sorry, could not find a maximum.")
except:
	print('\nOptimization interrupted.')

print('Reseting back to initial position (0 steps)...')
motor.drive(goto=0)

# GPIO cleanup
motor.cleanup()
