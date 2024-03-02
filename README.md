# Tree Planning Through The Trees
In this project a RRT* path planning and minimum snap trajectory generation (motion planning) stack is implemented for the 3D navigation of a DJI Tello Edu quadcopter in simulation (in blender) and is then tested on real quadcopter via NVIDIA Jetson Orin Nano. Additionally a cascaded PID controller gains for position and velocity control are tuned to follow the generated trajectory. 
(Check the full problem statements here [project2a](https://rbe549.github.io/rbe595/fall2023/proj/p2a/) and [project2b](https://rbe549.github.io/rbe595/fall2023/proj/p2b/))
## Steps to run the code
- This code is tested on Ubuntu 20.04 system
- Install Numpy, Scipy, Matplotlib, blender python, pyquaternion, djitellopy libraries before running the code.
- To run the simulation:
	-  open the `main.blend` file in blender
	- Go to compositing and set appropriate path to `outputs` folder
   	- Go to scripting tab and load the `main.py` file
   	- In `main.py` choose a map from the given set of four sample maps and uncomment the code accordingly to set start and goal locations for each map.
   	- Run the script in blender.
   	- Once the script is run press `spacebar` to run the simulation in layout tab.

## Report
For detailed description see the report [here](Report.pdf).
## Plots and Animations
On sample maps in blender:
### Sample map 1:
Watch the full simulation for sample map 1 [here](https://youtu.be/_bZi1fZofUs).

<p float="middle">
<img src="p2a/media/samplemap1.PNG" />
</p>

<p float="middle">
	<img src="p2a/media/samplemap1_frontview.gif" width="350" height="350" title="Front view"/>
	<img src="p2a/media/samplemap1_sideview.gif" width="350" height="350" title="Side View"/>
</p>

### Sample map 3:
Watch the full simulation for sample map 3 [here](https://youtu.be/z33Q_nKZx80).

<p float="middle">
<img src="p2a/media/samplemap3.PNG" />
</p>

<p float="middle">
	<img src="p2a/media/samplemap3_frontview.gif" width="350" height="350" title="Front view"/> 
	<img src="p2a/media/samplemap3_sideview.gif" width="350" height="350" title="Side View"/>
</p>

### Real map 1:
Watch the full blender simulation for Real map 1 [here](https://youtu.be/L_uQLSAiuyI) and a test run on the real tello drone here ([link1](https://youtu.be/1HOdJ2UzKZM) and [link2](https://youtu.be/9xGu6nxwU6Q)).

<p float="middle">
<img src="p2a/media/realmap1.PNG" />
</p>

<p float="middle">
	<img src="p2a/media/realmap1_frontview.gif" width="350" height="350" title="Front view"/> 
	<img src="p2a/media/realmap1_sideview.gif" width="350" height="350" title="Side View"/>
</p>



## References
1. Richter, Charles, Adam Bry, and Nicholas Roy. "Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments." Robotics Research: The 16th International Symposium ISRR. Springer International Publishing, 2016.

