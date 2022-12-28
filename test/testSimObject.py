from glob import glob
import sys
import time
import numpy as np
import colorama
from colorama import Fore
from colorama import Style
import math
from tracemalloc import get_object_traceback
sys.path.append("/tmp/temp/")
import root_as_cwd as rcwd
rcwd.prereq()
import coppeliasim_api.env.sim as sim
import coppeliasim_api.env.CopSim as CopSim #we will only be testing functions from this file 
from coppeliasim_api.env.constants import COPSIM_DIR

#___________________________________ functions used for testing ________________
epsilon = 10**(-4) #error threshold


def equals( a , b ):
    if(type(a)==list):
        if(dist(a,b)<epsilon and dist(a,b)>epsilon):
            return True
        return False

    #if a and b are floats
    if( a< b+epsilon and a>b-epsilon):
        return True
    return False 

def normaliseAngle( a : list ):
    a = np.asarray(a)
    for i in range(len(a)):
        if(np.abs(a[i]%(2*math.pi)-(2*math.pi))%(2*math.pi)< epsilon): #checks if the angle modulo pi is in [-epsilon,+epsilon]
                a[i] = 0
        else:
            a[i] = a[i]%(2*math.pi)
    return a


def dist( a: list, b: list, isAngle = False ):
    '''
    This function calculates the euclidian or angular distance between two points (depending on last param)
    returns the difference between two lists (avoids conversions of lists to np arrays)
    '''
    if isAngle == False:
        return np.linalg.norm(np.asarray(a)-np.asarray(b)) 
    else: 
        normaliseAngle(a)
        normaliseAngle(b)
        return np.linalg.norm(np.asarray(a)-np.asarray(b)) 


#_______________________________________________________________________________


class SimObjectTest():
    '''
        A new function is written every time the scene is reloaded.
    '''
    S = CopSim.Simulator(19997, COPSIM_DIR, headless=False, verbose=2)
    clientID= -1000
    wheelL = None
    wheelR = None
    jointL = None
    jointR = None
    body = None
    
    def __init__(self,scene) -> None: 
        '''This function initialises the simultor orbject, starts coppeliaSim, loads a given scene, gets object handles and initialises objects
        :@variable scene : path to the scene (.ttt) to be used '''
        self.clientID = self.S.start()
        self.S.load_scene(scene)
        self.wheelL = self.S.get_object_by_name("Wheel_L",False)
        self.wheelR = self.S.get_object_by_name("Wheel_R",False)
        self.jointL = self.S.get_object_by_name("Joint_L", True )
        self.jointR = self.S.get_object_by_name("Joint_R", True)
        self.body = self.S.get_object_by_name("Body", False)
        
    
    
    def end(self):
        '''Must be called at the end of the code, 
        it end the connection to coppeliaSim and shuts everything down'''
        self.S.end()

    #some code might be repeated but this aims to make each test indepently executable and readable
 
 
#_______________________________________________Testing get/set_object position ____________________

    def testPosition1(self):
        ''' testing get_set_object_position() by coposing the functions'''
        self.S.load_scene()
        try:
            for i in range(5):
                for j in range(5):
                    for k in range(5):
                        self.wheelL.set_object_position([i,j,k])
                        assert(equals( dist(self.wheelL.get_object_position(), [i,j,k]), 0 ) )
            print(Fore.YELLOW + "======== PASSED ====== : setting the left wheel to various positions works" + Style.RESET_ALL)
        except AssertionError:
            print(Fore.YELLOW+ "======== FAILED ====== : setting wheel to various positions doesn't work, stops at values [i,j,k]", print(i,j,k) + Style.RESET_ALL)


    def testPosition2(self):
        '''testing that the structure moves cohesively when the body is moved'''
        self.S.load_scene()
        try:
            distw_ini = dist(self.wheelL.get_object_position(), self.wheelR.get_object_position())
            self.body.set_object_position([10,0,0])
            distw_fin = dist(self.wheelL.get_object_position(), self.wheelR.get_object_position())   
            assert(equals(distw_ini, distw_fin) )
            print(Fore.YELLOW+ "======== PASSED ====== : When body is moved all elements move as one, cohesively" + Style.RESET_ALL)
        except AssertionError:
            print(Fore.YELLOW+ "======== FAILED ===== : When body is moved the rest of the strcture is not moved (possibly because of a hierarchy of objects in coppeliaSim)")
            print("the initial distance between wheel is: " , distw_ini, "dist after moving the wheel:", distw_fin, "" + Style.RESET_ALL)
        

    def testPosition2bis(self):
        '''testing that the structure moves cohesively when a wheel is moved'''
        self.S.load_scene()
        try:
            distw_ini = dist(self.wheelL.get_object_position(), self.wheelR.get_object_position())
            self.wheelL.set_object_position([10,0,0])
            distw_fin = dist(self.wheelL.get_object_position(), self.wheelR.get_object_position())   
            assert(equals(distw_ini, distw_fin) )
            print(Fore.YELLOW+  "======== PASSED ====== : When a wheel is moved the rest of the strcture is not moved (this is normal because of a hierarchy of objects in coppeliaSim)")
            print("the initial distance between wheel is: " , distw_ini, "dist after moving the wheel:", distw_fin, "" + Style.RESET_ALL)
        except AssertionError:
            print(Fore.YELLOW+  "======== FAILED ====== : When a wheel is moved all elements move as one, cohesively (hierarchy not respected !!)" + Style.RESET_ALL)
        

    def testPosition3(self):
        self.S.load_scene()
        #testing that distances are respected in simple cases  
        try:
            self.wheelL.set_object_position([30,0,0])
            self.wheelR.set_object_position([20,0,0])
            distw_fin = dist(self.wheelL.get_object_position(), self.wheelR.get_object_position())   
            assert(equals(distw_fin,10))
            print(Fore.YELLOW+ "======== PASSED ====== : Distances are as expected" + Style.RESET_ALL)
        except AssertionError:
            print(Fore.YELLOW+ "======== FAILED ====== : Distances are not as expected" + Style.RESET_ALL)

        #testing whether or not objects can be placed one on top of the other (is seems like physics are not activated until )
        try:
            self.body.set_object_position([0,0,0])
            self.wheelR.set_object_position([0,0,0])
            self.wheelL.set_object_position([0,0,0])
            self.jointR.set_object_position([0,0,0])
            self.jointL.set_object_position([0,0,0])
            assert(equals(self.body.get_object_position(), self.wheelR.get_object_position())and 
            equals(self.wheelR.get_object_position(), self.wheelL.get_object_position()) and
            equals(self.wheelL.get_object_position(),self.jointR.get_object_position()) and
            equals(self.jointR.get_object_position(), self.wheelL.get_object_position()))
            print(Fore.YELLOW+"======== PASSED ====== : All objects can be supperposed before a simulations lauch" + Style.RESET_ALL)
        except AssertionError:
            print(Fore.YELLOW+"======== FAILED ====== : All objects cannot be supperposed, some may behave differently, please take a closer look" + Style.RESET_ALL)


#_______________________________________________Testing set/get_object_orientation______________________


    def testOrientation1(self):
        ''' change body angle, check that angle is the one set'''
        self.S.load_scene()
        #the body's angle is set and we then check that the angle measured equals the angle set 
        try:
            temp1 = [0,0,0]
            refWheel = self.wheelL.get_object_orientation() #initial orientation of the wheel
            for i in range(3):
                for j in range(8):
                    temp1[i]= (j*(2*math.pi)/8)%(2*math.pi) 
                    temp1[(i+1)%3] = 0
                    temp1[(i+2)%3] = 0 #((4-j)*j*(2*math.pi)/5)%(2*math.pi)
                    self.body.set_object_orientation(temp1) 
                    #print("wheelL orientation :", normaliseAngle(self.wheelL.get_object_orientation()), "initial pos: ", refWheel, "diff : ", normaliseAngle(np.asarray(self.wheelL.get_object_orientation())- np.asarray(refWheel)) , "body orientiation :", self.body.get_object_orientation())
                    assert(equals(dist(self.body.get_object_orientation(), temp1, True),0))
                    
                    temp1=[0,0,0] #resetting for the new series (does not change anything)
                    self.body.set_object_orientation(temp1) 


            print(Fore.YELLOW+"======== PASSED ====== : The angles that we measured are the ones that where set " + Style.RESET_ALL)

        except AssertionError :
            print(Fore.YELLOW+"======== FAILED ====== : The angles that we set where not the ones we measured. expected : ", normaliseAngle(temp1), "received", normaliseAngle(np.asarray(self.wheelL.get_object_orientation())- np.asarray(refWheel))  ,"" + Style.RESET_ALL)
        return 0

    def testOrientation2(self): 
        '''change wheel orientation, make sure body orientation is unchanged'''
        self.S.load_scene()
        try:
            temp1=[0,0,0] #resetting for the new series
            refBodyAngle = self.body.get_object_orientation() #initial orientation of the wheel
            refBodyPos = self.body.get_object_position() #initial orientation of the wheel
            for i in range(3):
                for j in range(8):
                    temp1[i]= (j*(2*math.pi)/8)%(2*math.pi) 
                    temp1[(i+1)%3] = 0
                    temp1[(i+2)%3] = 0 #((4-j)*j*(2*math.pi)/5)%(2*math.pi)
                    self.wheelL.set_object_orientation(temp1)
                    self.wheelR.set_object_orientation(temp1)
                    #print("wheelL orientation :", normaliseAngle(self.wheelL.get_object_orientation()), "initial pos: ", refWheel, "diff : ", normaliseAngle(np.asarray(self.wheelL.get_object_orientation())- np.asarray(refWheel)) , "body orientiation :", self.body.get_object_orientation())
                    assert(equals(dist(self.body.get_object_orientation(), refBodyAngle, True),0) and equals(dist(self.body.get_object_position(), refBodyPos, True), 0))
    


            print(Fore.YELLOW+"======== PASSED ====== : changing the wheel's angle does not affect the body's position or angle " + Style.RESET_ALL)

        except AssertionError :
            print(Fore.YELLOW+"======== FAILED ====== : The angles that we set where not the ones we measured. expected : ", normaliseAngle(refBodyAngle), refBodyPos, "received", normaliseAngle(np.asarray(self.body.get_object_orientation())), self.body.get_object_position()  ,"" + Style.RESET_ALL)
        return 0




#_____________________  testing get/set_object_orientation RELATIVE TO some other object _____________________________________

    '''
    attention !! By default in CoppeliaSim when moving an object with the mouse make sure that the object is being moved relative to the world and not it's relative to it's own boxed zone
    attention !! An object's orientation is in polar coordinates !! Changing an object's orientation will not affect the position of the object ['s center] 
    '''

    def testRelOrientation1(self):
        ''' This test aims to show that changing orientation does not change body position, ie changing orientation of an object A relative to another object B is not moving A around B (which would change A's position)'''
        self.S.load_scene()
        try:
         
            iniBodyPos = self.body.get_object_position(self.jointL) #initial orientation of the wheel
            #refRelOrientation3 = self.body.get_object_orientation() #initial orientation of the wheel
            
            self.body.set_object_orientation([0,0,0])
            self.body.set_object_orientation([0, 0, -1.5707963705062866], self.jointL)

            finalBodyPos = self.body.get_object_position() #initial orientation of the wheel
            assert(equals(dist(iniBodyPos, finalBodyPos), 0)  )
            print(Fore.YELLOW+"======== PASSED ====== The orientation was changed as expected but the body's center did not move when it's orientation relative to the wheel's joint was changed (this may be a problem but is the expected behaviour)" + Style.RESET_ALL)
        except AssertionError :
            print(Fore.YELLOW+"======== FAILED ====== The body's center moved when it's orientation was changed relative to the wheel's joint" + Style.RESET_ALL)
            print("initial body pos :",iniBodyPos ,"final body pos", finalBodyPos)

    def testRelOrientation2(self):
        ''' This test aims to show that changing relative orientation of an element higher in the dependance tree relative to an element lower in the dependance tree does not change the relative orientation of those two objects)'''
        self.S.load_scene()
        try:
         
            iniRelOrientation = self.body.get_object_orientation(self.jointR) #initial orientation of the wheel
            #refRelOrientation3 = self.body.get_object_orientation() #initial orientation of the wheel
            
            self.body.set_object_orientation([0,0,0])
            self.body.set_object_orientation([0, 0, -1.5707963705062866], self.jointL)

            finalBodyOrientation = self.body.get_object_orientation() #initial orientation of the wheel
            assert(  equals(dist( normaliseAngle( np.asarray(iniRelOrientation)+ [0, 0, -1.5707963705062866]), normaliseAngle(finalBodyOrientation )),0))
            print(Fore.YELLOW+"======== PASSED ====== changing relative orientation of an element higher in the dependance tree relative to an element lower in the dependance tree does not change the relative orientation of those two objects)" + Style.RESET_ALL)
        except AssertionError :
            print(Fore.YELLOW+"======== FAILED ====== changing relative orientation of an element higher in the dependance tree relative to an element lower in the dependance tree changes the relative orientation of those two objects)" + Style.RESET_ALL)



    

if __name__ == "__main__":
    #on crée une instance de classe de test

    tester = SimObjectTest("./balancing_robot/copsim/Balancing-robot.ttt")
    #tester.testPosition1()
    #tester.testPosition2()
    #tester.testPosition2bis()
    #tester.testPosition3()
    #tester.testOrientation1()  #don't know what's happening 
    #tester.testOrientation2()   
    tester.testRelOrientation1()
    #tester.testRelOrientation2()
    time.sleep(15)
    tester.end()

    #code pour lancer une simu


'''
    chemin_model ="/home/erik/rl_model_100000_steps.zip"
    from ai.src.visu.copsim_render import model_render
    from stable_baselines3 import PPO
    from balancing_robot.copsim.BalancingRobotEnv_CopSim import BalancingRobotEnv_CopSim
    scene = "./balancing_robot/copsim/Balancing-robot.ttt"
    model = PPO.load(chemin_model)
    env   = BalancingRobotEnv_CopSim('v2', scene=scene, headless=False, veloc_mag=3, dt=0.05, reward= 0, x_lim=0.1)
    model_render(model, env)

'''

    


