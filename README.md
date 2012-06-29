BeingThere Package
==========

==DESCRIPTION==

This package is written mostly btw. Nov.11 to Mar.12 to provide a single framework to develop 3d-telepresence-related-projects.

It incorporates works by UNC's and ETHZ's researchers, kinect fusion and more.... It is organized into several software sub-packages shipped together with cmake-build system, and serveral examples to test each part of the packages and projects separatedly.

Designed as many popular cmake-based libraries (OpenCV, PCL...), this cmake-based package is cross-platform by design. It aims to be a large-scale project. However, due to short development time and lacking of man-power, the cmake script is only tested carefully in window OS. Some modifications are needed to make it work in other platforms such as linux, mac OS. 

==FEATURES==

Some of the key features are:

-It is cmake-based package. So it is portable, cross-platform, object-oriented and well-written... You choose your own OS, IDE...

-Using cmake's feature, it automatically finds and links with almost all software libraries, packages you ever need for this large project. It supports the following: Cuda, OpenGL(glew, glut), GLSL, CG Shading Language, OpenCV, PCL1.5.1, V3D, GLM, OpenNI, all PCL's dependencies such as Boost, VTK, eigen ... and much more.... The script is able to handle complex build which might contain mixing cuda .cu codes and c++ .cpp codes, which is hard to build in normal way.

-The code is organized in tree structure, each sub-packages comes together with a CMakeLists file to build seperatedly into a library. So it's very Extendable. You can extend the package in the way you like such as adding Networking, Compression, Segmentation... packages into the existing system.

-It provides many examples to help you start. For people who are new with this project, this packages provides you abt. 30 code examples, each deals with seperated issues: such as how to get data from multiple kinects with OpenNI, Microsoft SDK, how to upload data to textures, access textures and convert to point-cloud, mesh, how to use PCL for visualizing point cloud, and do depth-rgb segmentation using V3D-CG, calibration and more ....

-It incoporates current works by UNC and ETHZ presented in their papers.... which are re-written for clarity.

If you want to work in a team and struggle to find a way to organize work, collaborate, test ... this package might provide you a good solution.

==INSTALLATION==

1. Having Cmake Installed. 
2. Install and Configure Cuda in Your System.
3. Build PCL1.5.1, OpenCV From Source. Install all its dependencies, Kinect Driver, OpenNI.... Version should be the same, otherwise, you need to modify code to get it running.
4. Install CG Tooklit.
5. Create Out-Of-Source Build Folder to build the package into. Suppose BeingThere folder has src/ folder which contains source code.

On CMakeLists.txt in top folder of source folder. Change the path to OpenCV Libraries: set(OpenCV_DIR OpenCV_PATH). 
Cmake should be able to find PCL automatically and other packages. (FIND_PACKAGE(PCL 1.5 REQUIRED)...)

    * Using command line: 
	-cd BeingThere
	-mkdir/md build
	-cd build
	-cmake -G"Visual Studio 10 - Win 64" ../src

    * Using Cmake GUI 
	-Navigate To Build/ and src/ folder.
	-Choose build platform, IDE.
	-Press Configure, and Generate....


6. The package is shipped together with Glew, Glut 64 bit libraries. It is encouraged that you build the system in 64bit mode. If you insist to build in 32bit mode, please modify the opengl path in the CMakeLists.txt in top folder to the path of GLUT, GLEW 32 bit version. In linux, you should change the cmake script correspondingly. It is easy to add FindGLUT.cmake, FindGLEW.cmake into the system....

7. After building, the build/ folder now contain the generated target IDE solutions. In window, open the solution in Visual Studio 10 and build as usual. In Linux, import the folder to Eclipse IDE. If there is some error in some projects, use make command to force it keep continue despite of error and modify the script later. In Linux, you might need to modify the code to get it running. Just need to add #ifdef _WIN32 #include Win32 Header #endif ... to some sections of the code.

==HOW TO USE==
The linking process is written into Cmake MACRO. If you want to write your application using existing packages, libraries ... , you might create a new folder in example directories. Copy A CMakeList.txt into this folder. Put your source code into the folder as usual. 

*The CMakeList.txt usually contains something like the following:
 
-INCLUDE(MacroExampleApplication)
  
-MACRO_EXAMPLE_APPLICATION(ARG0 OPENCV OPENGL)

for linking your application with OpenCV, OpenGL (GLew, GLut). If you need to use cuda, PCL, CG, ... just need to add it at the end of the list: MACRO_EXAMPLE_APPLICATION(ARG0 OPENCV OPENGL PCL CG CUDA ....)

==HOW TO EXTEND==
The package provides you some data structures, algorithms for your to use in your codes. If you want to extend the system for your need, create a sub-package folder. Looking at CMakeLists.txt in other sub folder to see how to build software sub-package. 

After you are familiar to the package, it doesn't take much time to extend, configure, use this package. It provides you a nice way to organize your own code or collaborate with other people... With this system, transfering code from one place to other place will never be easier.... 

Feedback can be sent to nguy0066@e.ntu.edu.sg ...

Thanks for viewing and having fun!!


