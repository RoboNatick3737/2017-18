<p align="center"> 
<img src="/doc/hankstanks/hankstanks.jpg">
</p>

# Welcome!
This GitHub codebase contains the code for Team 3737's (Hank's Tanks) 2017-18 FTC challenge: Relic Recovery!  

# Key Concepts
* Multithreading occurs using ```SimpleTask``` and ```ComplexTask``` instances (the former of which is packaged into ```SimpleTaskPackage``` instances.  The difference between the two is the way that the latter runs in its own AsyncTask, and the former runs in a simple loop() method (which the task package, a ```ComplexTask``` instance, will mandate).  
* Music is available by typing ```Tunes.play(Tunes.Option.YOUR_TUNE_HERE)``` after being added to the project.  
* OpenCV and Vuforia are used as two vision libraries in this project, since each are specialized to accomplish different tasks.  ```OpenCVCam``` and ```VuforiaCam``` control their UI elements and camera states respectively.  
* In order to avoid crashing upon quitting the OpMode during a wait, use ```Flow.msPause(MS_TO_PAUSE)```, which continually checks to ensure that the op mode is still active.  
