__author__ = 'Ulrich Matchi AIVODJI'

from time import time

class Output(object):
	def __init__(self):
		super(Output, self).__init__()


def timer(func, *pargs, **kargs):
    """
    Measures the time required to run func with the given parameters.
    Returns the time as well as the result of the computation.
    """
    start = time()
    ret = func(*pargs, **kargs)
    elapsed = time() - start
    return elapsed, ret