%% Make the mex file to use C++ codes via Matlab
% Replace the folder locations with corresponding folders in your local
% environment.
mex -I"C:\libs"...
-I"C:\Users\varunjos\AppData\Roaming\MathWorks\MATLAB Add-Ons\Collections\kyamagu_mexplus\include"...
-I"C:\Users\varunjos\Documents\GitHub\Moco-custom-goal\exampleMatlabInterface\MocoCoordinateAccelerationGoal"...
-I"C:\Users\varunjos\Documents\GitHub\Moco-custom-goal\exampleMatlabInterface\MocoMarkerAccelerationGoal"...
-I"C:\OpenSim 4.3\sdk\spdlog\include"...
-I"C:\OpenSim 4.3\sdk\Simbody\include"...
-I"C:\OpenSim 4.3\sdk\include"...
-I"C:\OpenSim 4.3\sdk\include\OpenSim"...
-L"C:\OpenSim 4.3\sdk\Simbody\lib"...
-L"C:\Users\varunjos\Documents\GitHub\Moco-custom-goal\exampleMatlabInterface\build\Release"...
-L"C:\OpenSim 4.3\sdk\lib"...
-losimMoco -losimCommon -losimLepton -losimTools...
-losimActuators -losimExampleComponents -losimSimulation -losimAnalyses -losimJavaJNI...
-losimMocoCoordinateAccelerationGoal -losimMocoMarkerAccelerationGoal extendProblem.cpp