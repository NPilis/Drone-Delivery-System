from functools import wraps
from time import time

def with_timer(func):
    """Print function runtime wrapper"""
    wraps(func)
    def f(*args, **kwargs):
        before = time()
        rv = func(*args, **kwargs)
        after = time()
        print('Elapsed time', after - before, 's')
        return rv
    return f