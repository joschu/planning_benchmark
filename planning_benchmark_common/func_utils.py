"""
Useful function decorators, and functional programming stuff 
"""

import functools

class once:
    def __init__(self,fn):
        self.fn = fn
        self.out = None
        self.first_time = True
    def __call__(self,*args,**kw):
        if self.first_time:
            self.out = self.fn(*args,**kw)
            self.first_time = False
        return self.out

def disp_args(*args,**kw):
    return ",".join([str(arg) for arg in args] + ["%s=%s"%(str(key),str(val)) for (key,val) in kw.items()])        
        
TAB_LEVEL = 0
def verbose(fn):
    @functools.wraps(fn)
    def new_ver(*args,**kw):
        global TAB_LEVEL
        print("\t"*TAB_LEVEL+"%s(%s)"%(fn.__name__,disp_args(*args,**kw)))
        TAB_LEVEL += 1
        result = fn(*args,**kw)
        TAB_LEVEL -= 1            
        print("\t"*TAB_LEVEL+"=> %s"%str(result))    
        return result
    return new_ver        


class memoized(object):
    '''Decorator. Caches a function's return value each time it is called.
    If called later with the same arguments, the cached value is returned 
    (not reevaluated).
    '''
    def __init__(self, func):
        self.func = func
        self.cache = {}
    def __call__(self, *args):
        try:
            return self.cache[args]
        except KeyError:
            value = self.func(*args)
            self.cache[args] = value
            return value
        #except TypeError:
            ## uncachable -- for instance, passing a list as an argument.
            ## Better to not cache than to blow up entirely.
            #return self.func(*args)
    def __repr__(self):
        '''Return the function's docstring.'''
        return self.func.__doc__
    def __get__(self, obj, objtype):
        '''Support instance methods.'''
        return functools.partial(self.__call__, obj)