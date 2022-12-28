
import sys, os

#This function sets the root of the project as current working directory.
#This allows to later be able to access and import files with their path relative to the project's root.
def root_as_cwd():
    # initialize default path values:
    target_dir = "coppeliasim_api/env"
    root_dir = os.getcwd()
    print("os.getcwd() is :", os.getcwd())
    copsim_env_path = target_dir

    if not os.path.isdir(target_dir):
        while not os.path.isdir(copsim_env_path):
            copsim_env_path = os.path.join('..', copsim_env_path)
        root_dir = copsim_env_path.replace(target_dir, "")

    # run notebook in root dir and add the required paths to sys.path:
    if  root_dir !=  os.getcwd():  
        os.chdir(root_dir)
        sys.path.append(root_dir)
        sys.path.append(target_dir)

    print(f"\troot directory: <{root_dir}>")
    print(f"\tworking directory is now: <{os.getcwd()}>")

    # automatic reload of modules when modification
    #%load_ext autoreload
    #%autoreload 2   <-- does not work in python, is specific to Ipython (jupyter notebooks)

#this function 
def prereq():
    root_as_cwd()
    if not "/coppeliasim_api/env" in sys.path: sys.path.append("./coppeliasim_api/env")
    if not "./ai/src/run" in sys.path: sys.path.append("./ai/src/run")
    if not "/temp/tmp" in sys.path: sys.path.append("temp/tmp")
    if not "./" in sys.path: sys.path.append("./")

