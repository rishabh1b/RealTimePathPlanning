# RealTimePathPlanning
Sampling based rewiring approaches to solve real-time path planning problems for a holonomic robot under the presence of dynamic obstacles
<p align="center">
  <img src="https://github.com/rishabh1b/RealTimePathPlanning/blob/master/giphy.gif?raw=true" alt="Final Simulation Result"/>
</p>

## Installation Instructions
The exisiting codebase has a dependency on OpenFrameworks - an open source C++ toolkit for GUI and experimentation. The current implementation works well with _VS2015_. Make sure you install VS2015 and install OpenFrameworks plugin for Visual Studio by following the steps documented [here](http://openframeworks.cc/setup/vs/). Once installed, follow these further steps - 
1. Download the OpenFrameworks binaries for windows from [here](http://openframeworks.cc/versions/v0.9.8/of_v0.9.8_vs_release.zip)
2. Create a OpenFrameworks based project in Visual Studio. Point to the path where OpenFrameworks was installed in step 1. Make sure you tick 'ofxGUI' addon while creating the project
3. Clone this repository and copy the code in the ```src```. Add these items using 'Add Existing Item' option in Visual Studio
4. Build the Solution.
5. Run the ```ofApp.cpp``` application

## Live Demonstration
The demonstration is available at this YouTube [link](https://www.youtube.com/watch?v=WSfL9L6eBpk)


 
