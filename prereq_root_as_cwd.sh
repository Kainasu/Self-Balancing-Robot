#!/bin/bash
#obj: set a known absolute path for custum_tools.py in order to be able to call root_as_cwd() in any files"
mkdir /tmp/temp 
rm /tmp/temp/root_as_cwd.py && echo "the previous file located at /tmp/temp/root_as_cwd.py was deleted"
cp ./root_as_cwd.py /tmp/temp/ && echo "./root_as_cwd.py was copied to that location"
