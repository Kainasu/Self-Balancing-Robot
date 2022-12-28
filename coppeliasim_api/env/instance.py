#
# JLC 2020-12-08 :  replace str.format() with fstring
#                   

import subprocess
import os, sys, psutil, signal

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()
    
class Instance():
    '''The class to manage subprocess'''

    __list_of_instances = []
    
    @staticmethod
    def cleanup():
        for inst in Instance.__list_of_instances:
            inst.end()
        
    @staticmethod
    def nb():
        return len(Instance.__list_of_instances)
    
    @staticmethod
    def ls():
        print('#\targs')
        for i, inst in enumerate(Instance.__list_of_instances):
            print(f'{i}\t{inst.args}')        
    
    def __init__(self, args, cwd=None, verbose=2):
        self.args = args # the command to run with its arguments
        self.inst = None # the return of Popen call
        self.cwd  = cwd
        Instance.__list_of_instances.append(self)
        
        # verbosity:
        if verbose == 0:
            self.prt1 = lambda x: x
            self.prt2 = lambda x: x
        elif verbose == 1 :
            self.prt1 = print
            self.prt2 = lambda x: x
        elif verbose == 2 :
            self.prt1 = print
            self.prt2 = print

    def start(self):
        self.prt1(f'[Instance] trying to start <{" ".join(self.args)}>')
        try:
            if self.cwd is not None:
                self.inst = subprocess.Popen(self.args, cwd=self.cwd)
            else:
                self.inst = subprocess.Popen(self.args)
        except EnvironmentError:
            print(f'[Instance] Error: cannot find executable at {self.args[0]}')
            raise
        self.prt2(f'[Instance] started : {self.inst}')
        return self

    def isAlive(self):
        return True if self.inst.poll() is None else False

    def end(self, procname=None):
        self.prt1('[Instance] terminating...')
        if self.isAlive():
            if sys.platform == 'linux':
                if procname:
                    for proc in psutil.process_iter():
                        # check whether the process name matches
                        if proc.name() == procname:
                            self.prt2(f"kill by name <{proc.name()}>")
                            proc.kill()
                            break
                else:
                    print(f"Trying to kill by pid <{self.inst.pid}>", end="")
                    done = " failed!"
                    for proc in psutil.process_iter():
                        if self.inst.pid == proc.pid:                            
                            for child_proc in proc.children(recursive=True):
                                print(f"\n\tkilling child <{child_proc.pid}>")
                                child_proc.kill()
                            print(f"\n\tkilling <{proc.pid}>")
                            proc.kill()
                            done = " done !"
                            break
                    print(done)                    
            else:
                print('Error: Instance.end only supported for GNU/Linux platform.')
                raise 
            self.inst.terminate()
            retcode = self.inst.wait()
            self.prt2('[Instance] waiting ...')
        else:
            retcode = self.inst.returncode
        self.prt2(f'[Instance] retcode: {retcode}')
        return self

if __name__ == "__main__":
    import platform
    if platform.system() =='Linux':
        import time
        # Testing class Instance...
        i = Instance(['gedit'], cwd="/tmp")
        i.start()
        Instance.ls()
        time.sleep(5)
        print("alive:",i.isAlive())
        Instance.cleanup()
    else:
        print("Only for Linux platform !!!")