import skrf
from skrf.calibration import SOLT
import numpy as np

### Create ideals ###
freq = skrf.Frequency(3400, 3500, 101, 'mhz')
s = np.arange(101*2*2, dtype='complex_')
s = s.reshape((101,2,2))
# Open
s_open = 1*np.ones_like(s)
s_open[:,0,1] = np.nan+np.nan*1j
s_open[:,1,0] = 0
s_open[:,1,1] = np.nan+np.nan*1j
ntw_open = skrf.Network(frequency=freq, s=s_open)
ntw_open.write_touchstone('ideal/open')
# Short
s_short = -1*np.ones_like(s)
s_short[:,0,1] = np.nan+np.nan*1j
s_short[:,1,0] = 0
s_short[:,1,1] = np.nan+np.nan*1j
ntw_short = skrf.Network(frequency=freq, s=s_short)
ntw_short.write_touchstone('ideal/short')
# Load
s_load = 0*np.ones_like(s)
s_load[:,0,1] = np.nan+np.nan*1j
s_load[:,1,0] = 0
s_load[:,1,1] = np.nan+np.nan*1j
ntw_load = skrf.Network(frequency=freq, s=s_load)
ntw_load.write_touchstone('ideal/load')
# Thru
s_thru = 0*np.ones_like(s)
s_thru[:,0,1] = np.nan+np.nan*1j
s_thru[:,1,0] = 1+0j
s_thru[:,1,1] = np.nan+np.nan*1j
ntw_thru = skrf.Network(frequency=freq, s=s_thru)
ntw_thru.write_touchstone('ideal/thru')

"""
# a list of Network types, holding 'ideal' responses
my_ideals = [
    skrf.Network('ideal/thru.s2p'),
    skrf.Network('ideal/short, short.s2p'),
    skrf.Network('ideal/open, open.s2p'),
    skrf.Network('ideal/load, load.s2p'),
    ]

# a list of Network types, holding 'measured' responses
my_measured = [
    skrf.Network('measured/thru.s2p'),
    skrf.Network('measured/short, short.s2p'),
    skrf.Network('measured/open, open.s2p'),
    skrf.Network('measured/load, load.s2p'),
    ]


## create a SOLT instance
cal = skrf.calibration.SOLT(
    ideals = my_ideals,
    measured = my_measured,
    )


## run, and apply calibration to a DUT

# run calibration algorithm
cal.run()

# apply it to a dut
dut = skrf.Network('my_dut.s2p')
dut_caled = cal.apply_cal(dut)

# plot results
dut_caled.plot_s_db()
# save results
dut_caled.write_touchstone()
"""