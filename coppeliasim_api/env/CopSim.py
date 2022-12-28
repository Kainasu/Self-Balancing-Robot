#
# Adapted from Qin Yongliang 20170410
#
# JLC 2018 : inspect.getargspec deprecated => using inspect.signature instead.
#            replace V-Rep buy CoppeliaSIm everywhere.
import sys
sys.path.append("/tmp/temp/")
import root_as_cwd as rcwd
rcwd.prereq()

try:
    from coppeliasim_api.env import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported.\nThis means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.  ')
    print ('Make sure both are in the same folder as this file,           ')
    print ('or appropriately adjust the file "sim.py"                     ')
    print ('--------------------------------------------------------------')
    print ('')
    raise

import os, types
import numpy as np
from inspect import signature
from coppeliasim_api.env.instance import Instance

blocking    = sim.simx_opmode_blocking
oneshot     = sim.simx_opmode_oneshot
buffer      = sim.simx_opmode_buffer
streaming   = sim.simx_opmode_streaming
discontinue = sim.simx_opmode_discontinue


def check_ret(ret_tuple, ignore_one=False):
    '''To check the tuple returned by Sim API functions call.
        Raise error if retcode is not OK, return remaining data otherwise.'''

    istuple = isinstance(ret_tuple, tuple)
    if not istuple:
        ret = ret_tuple
    else:
        ret = ret_tuple[0]

    if (not ignore_one and ret != sim.simx_return_ok) or (ignore_one and ret > 1):
        raise RuntimeError('retcode(' + str(ret) + \
                           ') not OK, API call failed. Check the parameters!')

    return ret_tuple[1:] if istuple else None


#==========================================================================

class SimObject():
    def __init__(self, env, handle, is_joint): #env useless is we only use copsim
        self.env      = env  #self.env. has been replaced by sim.
        self.handle   = handle
        self.is_joint = is_joint

    def get_object_floatParameter(self, parameter_number):
        return check_ret(self.env.simxGetObjectFloatParameter(
            self.handle,
            parameter_number,
            blocking))

    def get_object_orientation(self, relative_to=None):
        (eulerAngles,) = check_ret(self.env.simxGetObjectOrientation(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            blocking))
        return eulerAngles
    
    def set_object_orientation(self, array, relative_to=None):
        return check_ret(self.env.simxSetObjectOrientation(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            array,
            blocking))

    def get_object_position(self, relative_to=None):
        (position,) = check_ret(self.env.simxGetObjectPosition(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            blocking))
        return position
    
    def set_object_position(self, array, relative_to=None):
        return check_ret(self.env.simxSetObjectPosition(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            array,
            blocking))

    def get_object_velocity(self):
        return check_ret(self.env.simxGetObjectVelocity(
            self.handle,
            blocking))

    def set_joint_position(self, p):
        self._check_joint()
        return check_ret(self.env.simxSetJointPosition(
            self.handle,
            p,
            blocking))

    def set_joint_target_velocity(self, v):
        self._check_joint()
        return check_ret(self.env.simxSetJointTargetVelocity(
            self.handle,
            v,
            blocking))

    def set_joint_force(self, f):
        self._check_joint()
        return check_ret(self.env.simxSetJointForce(
            self.handle,
            f,
            blocking))

    def set_joint_target_position(self, value):
        '''Set desired position of a servo
           :param int value: position or angle depending on the joint nature
           :return: None if successful, otherwise raises exception'''
        self._check_joint()
        return check_ret(self.env.simxSetJointTargetPosition(
            self.handle,
            value,
            blocking))

    def get_joint_position(self, relative_to=None):
        position, = check_ret(self.env.simxGetJointPosition(
            self.handle,
            blocking))
        return position

    def get_joint_force(self):
        self._check_joint()
        force = check_ret(self.env.simxGetJointForce(
                self.handle,
                blocking))
        return force

    def read_force_sensor(self):
        state, forceVector, torqueVector = check_ret(self.env.simxReadForceSensor(
            self.handle,
            blocking))

        if state & 1 == 1:
            return None  # sensor data not ready
        else:
            return forceVector, torqueVector

    def get_vision_image(self):
        resolution, image = check_ret(self.env.simxGetVisionSensorImage(
            self.handle,
            0,  # options=0 -> RGB
            blocking))
        dim, im = resolution, image
        nim = np.array(im, dtype='uint8')
        nim = np.reshape(nim, (dim[1], dim[0], 3))
        nim = np.flip(nim, 0)  # LR flip
        nim = np.flip(nim, 2)  # RGB -> BGR
        return nim

    def _check_joint(self):
        if not self.is_joint:
            raise Exception("[SimObject] Trying to call a joint function on a non-joint object.")


#=================================================================================


class Simulator():
    '''Encapsulates the Sim simulation instance.'''
    
    def __init__(self, port_num=19997, dir_sim='', args='',
                 headless=False, verbose=1):
        
        self.port_num     = port_num # the port number where Sim is listening
        self.dir_sim      = dir_sim # the install directory of Sim dsitrib.
        self.instance     = None     # the Instance objet for Sim simulation
        self.sim_method   = None     # wrapping of Sim fucntions

        self.sim_command  = './coppeliaSim.sh'
        self.started      = False 

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
       
        # clientID of the instance when connected to server,
        # to differentiate between instances in the driver:
        self.cid = -1 
 
        # is the simulation currently running (as far as we know):
        self.sim_running = False

        self.prt2(f'[CopSim] port_num is: {self.port_num}')
        self.prt2(f'[CopSim] path to sim is: {self.dir_sim}')

        # start CoppeliaSim in a sub process.
        # self.env.exe -gREMOTEAPISERVERSERVICE_PORT_DEBUG_PREENABLESYNC: 
        # the argument can be used to request a continuous remote API server service 
        # to be started at CopSim start-up. For that to happen, replace in above string following:
        #   PORT   : is the port number
        #   DEBUG  : is the debug mode (set to TRUE or FALSE)
        #   PREENABLESYNC : allows to preenable the synchronous mode (set to TRUE or FALSE)
        
        if args == '':
            args = [self.sim_command, '-gREMOTEAPISERVERSERVICE_' + str(self.port_num) + '_FALSE_TRUE']
        if headless:
            args.append('-h')
        args.append("-vnone")
        self.prt2(f'[CopSim] sim args: {args}')            
        
        # instance created but not started:
        self.instance = Instance(args, cwd=self.dir_sim, verbose=verbose)

        # assign every API function call from sim to self
        sim_methods = [a for a in dir(sim) if
                        not a.startswith('__') and isinstance(getattr(sim, a), types.FunctionType)]

        def assign_from_sim_to_self(name):
            wrapee = getattr(sim, name)
            args   = signature(wrapee)
            arg0   = list(args.parameters.keys())[0]
            if arg0 == 'clientID':
                def func(*args, **kwargs):
                    return wrapee(self.cid, *args, **kwargs)
            else:
                def func(*args, **kwargs):
                    return wrapee(*args, **kwargs)
            setattr(self, name, func)

        for name in sim_methods: assign_from_sim_to_self(name)

       
    def start(self):
        '''To start everything...'''
        if self.started == True:
            raise RuntimeError('[CopSim] you should not call start() more than once')

        self.prt1('[CopSim] starting an instance of CoppeliaSim...')
        self.instance.start()

        # try to connect to CoppeliaSim instance via socket
        retries = 0
        while True:
            self.prt2('[CopSim] trying to connect to server on port '\
                      +str(self.port_num)+', retry:'+str(retries))
            sim.simxFinish(-1)    # just in case, close all opened connections
            self.cid = self.simxStart('127.0.0.1', 
                                      self.port_num,
                                      waitUntilConnected=True,
                                      doNotReconnectOnceDisconnected=True,
                                      timeOutInMs=5000,
                                      commThreadCycleInMs=5)
                                      
            if self.cid == -1:
                retries += 1
                if retries > 15:
                    self.end()
                    raise RuntimeError('[CopSim] Unable to connect to CoppeliaSim after 15 retries.')
            else:
                self.prt2('[CopSim] Connected to remote API server with cid:{}'.format(self.cid))
                break

        # Now send some data to CoppeliaSim in a non-blocking fashion:
        self.simxAddStatusbarMessage('[CopSim] Hello CoppeliaSim', oneshot)

        # setup a useless signal
        self.simxSetIntegerSignal('asdf', 1, blocking)

        self.prt1('[CopSim] CoppeliaSim instance started, remote API connection OK.')

        self.started = True
        
        return self.cid #returned "self" before, return self.cid is used in testSimObject 


    def end(self):
        '''kill everything, clean up.'''

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had 
        # time to arrive. You can guarantee this with (for example):
        # sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        if self.sim_running: self.stop_simulation()
        self.simxFinish()
        self.instance.end()
        self.started= False
        self.prt1('[CopSim] everything has been shut down.')
        return self

    def load_scene(self, fullpathname="./balancing_robot/copsim/Balancing-robot.ttt"):
        self.prt1('[CopSim] loading scene from '+fullpathname)
        flg = 0
        i = 0
        while i< 10:
            if flg == 0:
                res = sim.simxLoadScene(self.cid, fullpathname, 0xFF, blocking)
                if(0 == res):
                    flg += 1
                    self.prt2('[CopSim] scene successfully loaded')
                print("[CopSim] scene failled to load", i, "times")

            else :
                i = 111
            i+= 1
            if(i == 111):
                print("[CopSim] make sure that CoppeliaSim is oppened and that the latest version is installed") #pb: coppeliaSim crash après s'être ouvert, telecharger la dernière version a réglé le pb
                
            #try:
                #check_ret(self.simxLoadScene(fullpathname, 0xFF, blocking)) #<- ancienne ligne de code
            #    print(sim.simxLoadScene(self.cid, fullpathname,0,blocking)) #check_ret est la source d'erreurs
            #except:
            #    raise RuntimeError('[CopSim] scene loading failure')
            #self.prt2('[CopSim] scene successfully loaded')


    def start_synchro_simulation(self, dt=0.02):
        self.start_simulation(True, dt)

    def start_nonsynchro_simulation(self, dt=0.02):
        self.start_simulation(False, dt)

    def start_simulation(self, is_sync, dt):
        # IMPORTANT
        # you should poll the server state to make sure
        # the simulation completely stops before starting a new one
        while True:
            # poll the useless signal (to receive a message from server)
            check_ret(self.simxGetIntegerSignal('asdf', blocking))

            # check server state (within the received message)
            e = self.simxGetInMessageInfo(sim.simx_headeroffset_server_state)

            # check bit0
            not_stopped = e[1] & 1

            if not not_stopped:
                break
        # set the timestep simulation :
        # => the CoppeliaSim scene must be configured with 'custom'
        param_id = sim.sim_floatparam_simulation_time_step
        check_ret(self.simxSetFloatingParameter(param_id,dt,blocking))
        
        # enter sync mode
        check_ret(self.simxSynchronous(is_sync))
        check_ret(self.simxStartSimulation(blocking))
        self.sim_running = True

##    def make_simulation_synchronous(self, sync):
##        if not self.sim_running:
##            self.prt1("[CopSim] simulation doesn't seem to be running, starting up")
##            self.start_simulation(sync)
##        else:
##            check_ret(self.simxSynchronous(sync))

    def stop_simulation(self):
        check_ret(self.simxStopSimulation(oneshot), ignore_one=True)
        self.sim_running = False

    def step_synchro_simulation(self):
        check_ret(self.simxSynchronousTrigger())

    def get_object_handle(self, name):
        handle, = check_ret(self.simxGetObjectHandle(name, blocking))
        return handle

    def get_object_by_handle(self, handle, is_joint=True):
        '''Get the sim object for a given handle
           :param int handle: handle code
           :param bool is_joint: True if the object is a joint that can be moved
           :returns: SimObject'''
        return SimObject(self, handle, is_joint)

    def get_object_by_name(self, name, is_joint=True):
        '''Get the sim object for a given name
           :param str name: name of the object
           :param bool is_joint: True if the object is a joint that can be moved
           :returns: SimObject'''
        return self.get_object_by_handle(self.get_object_handle(name), is_joint)

    @staticmethod
    def create_params(ints=[], floats=[], strings=[], bytes=''):
        if bytes == '':
            bytes_in = bytearray()
        else:
            bytes_in = bytes
        return (ints, floats, strings, bytes_in)

    def call_script_function(self, function_name, params, script_name="remoteApiCommandServer"):
        '''Calls a function in a script that is mounted as child in the scene
           :param str script_name: the name of the script that contains the function
           :param str function_name: the name of the function to call
           :param tuple params: the parameters to call the function with 
                (must be 4 parameters: list of int, list of float, list of str, and bytearray)
           :returns: tuple (res_ints, res_floats, res_strs, res_bytes):
                list res_ints is a list of integer results
                list res_floats is a list of floating point results
                list res_strs is a list of string results
                bytearray res_bytes is a bytearray containing the resulting bytes.'''
        assert type(params) is tuple
        assert len(params) == 4

        return check_ret(self.simxCallScriptFunction(
            script_name,
            sim.sim_scripttype_childscript,
            function_name,
            params[0],  # integers
            params[1],  # floats
            params[2],  # strings
            params[3],  # bytes
            blocking))

if __name__ == "__main__":
    # Testing class Instance...
    import time, platform
    if platform.system() =='Linux':
        # LINUX CUSTOMIZE : the file 'constants.py' from 'coppeliaSim/env' sets COPSIM_DIR to be the
        # name of the CoppeliaSim installation dircetory with the name COPSIM_DIR : modify it as needed...
        from constants import COPSIM_DIR
        print(f"CoppeliaSim installation found in directory <{COPSIM_DIR}>")
        S = Simulator(19997, COPSIM_DIR, headless=False, verbose=2)
        S.start()
        scene = "balancing_robot/copsim/Balancing-robot.ttt"
        S.load_scene(scene)
        print("waiting 5 secondes before shutting down....")
        time.sleep(5)
        S.end()    
