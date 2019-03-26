# vrep-python-test

Purpose: Jacobian-based Inverse Kinematics

1. Download and extract recent V-REP PRO/DEU (Hereafter, vrep folder)
2. Git clone this repository (Hereafter, source folder)
3. In vrep/programming/bluezero

   ```mkdir build && cd build```

   ```cmake .. -DCMAKE_BUILD_TYPE=RELEASE```
   
   ```make```
   
   After compiling ti, copy libb0.s0 in build folder and paste it into source/Extern 
   
 4. Copy b0.py and b0RemoteApi.py in vrep/programming/b0RemoteApiBindings/python/python
 
    And Paste them into source/Extern
    
 5. Open franka_urdf.ttt file with Vrep, for example
 
   ```sudo $HOME/vrep/vrep.sh $Home/source/franka_urdf.ttt```
   
   and Turn on the Bluezero server
   
   in top-menu bar (Add-ons -> b0remoteApiSever click)
   
 6. Run demo script

   
 # TODO
 Bug#1: to exit the python script, push Ctrl+c
 
