# References http://spg.ucolick.org/KTLPython/index.html#

#! /usr/bin/env kpython

import ktl



"""
Running the most simple kPython example,
in order to see if things are connected,
and code can actually access EPICS and command CPU
"""
# If PCU is the main EPICS Service (motor)
service = ktl.Service('PCU')
interest = ('RTR', 'LNR')

for name in interest:
    keyword = service[name]
    keyword.callback(myCallback)
    keyword.monitor()
