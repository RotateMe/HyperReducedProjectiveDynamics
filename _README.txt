********************************************************************************
MIT License

Copyright (c) 2018 Christopher Brandt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************

In addition to the above license we would like to kindly ask you to cite the publication
"Hyper-Reduced Projective Dynamics" by Christopher Brandt, Elmar Eisemann and 
Klaus Hildebrandt if you use this code for your research.

ACKNOLEDGEMENTS:
This project uses TetGen for volumetric mesh generation and libIGL for loading and displaying the
meshes:
Hang Si. 2015. "TetGen, a Delaunay-Based Quality Tetrahedral Mesh Generator". ACM Trans. on 
Mathematical Software. 41 (2), Article 11 (February 2015)
Alec Jacobson, Daniele Panozzo et al. "libIGL: A simple C++ geometry processing library",
http://libigl.github.io/libigl/, 2017


INTRODUCTION:
The code files provided here are C++ 11 source and header files, which provide the precomputation 
and simulation methods presented in the paper "Hyper-Reduced Projective Dynamics" by Christopher
Brandt and Klaus Hildebrandt.
The usage is demonstrated in the code in the main() function in HRPD.cpp, where a mesh is loaded
and simulated in real-time, using Alec Jacobson's libIGL for loading and rendering (libIGL is only 
needed for the example code, the Hyper-Reduced Projective Dynamics code does not use it).
On http://graphics.tudelft.nl/~klaus/ you will also find a precompiled executable.

COMPILATION/SETUP:
- Add all code files in the "\src" folder, EXCEPT FOR THE FILES
    doubleToFloatDeviceCpy.cu 
	CUDAMatrixVectorMult.cpp 
  to your project.
  Add the "\include" folder as an additional include folder.
  The file HRPD.cpp contains the example code. It contains the main() function and adds 
  the dependencies to libIGL and thus to GLAD and GLFW.
  If any other viewer and mesh loading functions are desired, it should be easy to
  modify the code to use your own pipeline (see "USAGE" below).
- Carefully read the dependencies below and set up your project links and includes accordingly
- Move the mesh file to the working directory, which is either the directory where
  the HRPD.exe is located or the Visual Studio Project directory (if you run the
  executable through the VS Debugger).
- Once the program runs, it will show an armadillo being dropped on an invisible floor.
  You can use the mouse to rotate the camera.
  The following key-commands are available
   - press 1 to display the framerate of the simulation,
   - press 2 to reset the positions 
   - press 3 to hang the back of the armadillo into the air (soft constraints)
     (or release it)
   - press 4 to toggle stiffness between normal and super elastic
   - press 5 to toggle between bullet time (1/4th of real time) and real time 
   - press 6 to repeat precomputation

COMPILATION CMAKE:
 - prerequisites:
    - Install CMake version 3.22.1 or higher
    - Install a compiler of your choice Visual studio build tools, gcc and minGW should work
 - use an IDE to import the project as cmake project and run it or use command line
    - CMAKE COMMANDLINE:
        - make a folder called build in the project directory
        - open a terminal in the folder and call `cmake ../`
        - now build the generated build folder by cmake with your preferred command line tool.
   
DEPENDENCIES:
- CPU parallelization is done using OpenMP, so e.g. in Visual Studio, the /openmp option needs
  to be enabled (Project Properties -> C++ -> Language -> OpenMP Support).
  The number of threads is defined in ProjDynSimulator.h as PROJ_DYN_NUM_THREADS
  This should NOT be more than the number of physical threads your CPU is able to handle.
- The Eigen library is used for linear algebra computations (https://eigen.tuxfamily.org)
  It is a header only library, so simply download it and add the base directory of Eigen as a
  include directory to your project.
  IT IS IMPORTANT TO SET THE PREPROCESSOR DEFINITION EIGEN_DONT_PARALLELIZE, as otherwise
  Eigen's and our parellelization will conflict and FPS will be much lower!!!
- For the example in HRPD.cpp we use Alec Jacobson's libIGL to load and display  
  meshes, so libIGL headers need to be included.
  The libIGL OpenGL viewer depends on GLAD and GLFW, both of which are linked in the
  libIGL's Git repository in the "\external" folder.
  They need to be built using CMake and the libraries glad.lib and glfw3.lib need 
  to be added as linker input.
  The "\include" folder of GLAD and GLFW need to be added to the additional include
  directories.
  libIGL is available from https://github.com/libigl/libigl
  Due to the methods used to access files in the libIGL readOBJ() function, I had to add
  the preprocessor definition _CRT_SECURE_NO_WARNINGS.
   
NOTES:
- The example simulation in HRPD.cpp shows a dropping armadillo with volume preservation
  constraints. User interaction can be achieved by pressing the keys 2-5 (see below).
  The tetrahedralized mesh used there has 19k vertices and we are using 10 local global
  iterations per timestep, 1800 DOFs for vertex positions, 360 DOFs for the subspace
  of constraint projections and we evaluate 1000 sampled constraint projections.
  We use 8 cores for parallel evaluations of constraint projections.
  If you have less than 8 logical processors (4 cores), you should change the defines
	#define PROJ_DYN_NUM_THREADS 8
  accordingly.
- On the machine this code was tested on, the simulation step (including evaluation
  of the final vertex positions) took 6ms, i.e. the simulation runs at 166 FPS
  (note that this number exceeds what is stated in the paper, which is due to simple code
  optimizations done after publication).
  This is the timing WITHOUT using cholmod and CUDA/CUBLAS for acceleration (see below).
  By pressing 1 during the running simulation the current FPS (of the simulation)
  is printed and you can compare to this value. Even for significantly slower machines
  the FPS should not be too far off. Note that the code should only be run in Release mode.
- This code has been modified from the reference implementation to offer an easier setup and
  compilation procedure. As such, a few parts are no longer optimized.
  Most importantly, for large meshes, you would want to map the full vertex positions directly
  to the GPU OpenGL vertex buffer, to circumvent computing them on the GPU, copying to the CPU,
  and then writing them into the buffer again each frame, like it is done now (in HRPD.cpp).
  This will quickly become a bottleneck for large meshes.
- Running this project in Debug mode will lead to extremely slow execution times and is 
  not recommended.
- There are optional dependencies below. Note that if the CUDA/CUBLAS is not used, the
  simulation timings will be dominated by updating the vertex positions from reduced
  coordinates (a sparse matrix vector multiplication).
  Thus, for very large meshes (>200k vertices) it is recommended to use CUDA.
- The code does originate from research code and is inconsistently commented, still contains
  bugs, contains variables and methods which were only used for testing purposes and has
  public members that should not be public.
- We only tested this code in a Visual Studio 2015 and 2017 project and can not help with compilation
  on other systems. However, all dependent libraries are available cross-platform.
- To narrow down problems with running the example in HRPD.cpp, try to make sure first that
  you are able to compile and run a project where you simply load and diplay a mesh using libIGL,
  then add the simulator and only THEN add the GPU acceleration for vertex position updates (see
  below).
- volumetric meshes are always created from surface meshes in the precomputation phase,
  which is done using tetgen: http://www.wias-berlin.de/software/index.jsp?id=TetGen

  
OPTIONAL DEPENDENCIES:
- Vertex position updates from reduced coefficients can be handled using the GPU, which requires
  CUDA includes and libraries, as well as the .cu and .cuh files to be compiled with the CUDA
  compiler.
  If this is desired, add the the .cu and .cuh files as well as the files
  CUDAMatrixVectorMult.h/.cpp to the project, note however and uncomment the 
  #define PROJ_DYN_USE_CUBLAS and #define PROJ_DYN_USE_CUBLAS_IN_PRE in 
  ProjDynSimulator.h.
  The project needs to add the CUDA build customization.
  Then the following libraries need to be linked:
	cublas.lib
	cudart_static.lib
	cusparse.lib
- Eigen's cholmod interface can be used to solve the linear systems in the global and
  local steps faster. This also shortens factorization timings, such that changing stiff-
  ness or the timestep doesn't lead to framedrops anymore.
  If you want to use cholmod, uncomment the #define PROJ_DYN_USE_CHOLMOD 
  in ProjDynSimulator.h.
  There is a great CMake project for suitesparse by Jose Luis Blanco at 
  https://github.com/jlblancoc/suitesparse-metis-for-windows
  This also contains precompiled libraries for BLAS and LAPACK, which also need to be
  linked.
  Specifically, using suitesparse in Windows, you need to compile and link the following 
  libraries:
	libcholmod.lib
	libcolamd.lib
	libcxsparse.lib
	libklu.lib
	libldl.lib
	libspqr.lib
	libumfpack.lib
	suitesparseconfig.lib
	libamd.lib
	libbtf.lib
	libcamd.lib
	libccolamd.lib
	libblas.lib
	liblapack.lib
	metis.lib
  libblas and liblapack are precompiled and will not be in the build folder, but in the
  original git repository.
  metis.lib only needs to be included if you built suitesparse with metis, which is
  not required (and might cause problems ).
  You will have to include the "include\suitesparse" folder, since Eigen includes cholmod
  simply as "cholmod.h".
  
CODE USAGE:
- The example code in HRPD.cpp should demonstrate clearly how a simulation can be
  set up and run within some real-time environment and it is recommended to understand 
  the example first.
  After that, the bullet points below will provide some details.
- The only headers that must be included are "ProjDynTypeDef.h" and "ProjDynSimulator.h".
- The mesh vertex positions and mesh faces are stored as certain Eigen matrices, whose types
  are defined as PD::PDPositions and PD::PDTriangles.
- From these quantities, you can construct a simulator object using the constructor
  ProjDynSimulator(
		PDTriangles& triangles, 				// Triangles
		PDPositions& initialPositions,			// Initial Pos.
		PDPositions& initialVelocities,			// Initial velocities
		PDScalar timeStep,						// The timestep 
		int numSamples = -1,					// Number of samples from which the vertex
												// positions subspace is constructed (k)
		PDScalar baseFunctionRadius = 2.5,		// The radius multiplier for the weight
												// functions (base radius = smallest radius
												// that ensures that each vertex is covered)
		int interpolBaseSize = 120,				// The desired size of the interpolation subspace
												// after the PCA. The intermediate space will be
												// constructed from half as many weight functions,
												// each of which results in four intermediate base
												// vectors.
		PDScalar rhsInterpolWeightRadius = 2.,  // Radius multiplier in the construction of the
												// subspace for constraint projections V
		int numConstraintSamples = -1,			// Number of constraint samples (s)
		PDScalar massPerUnitArea = 1,			// A means of correcting mass for larger/smaller
												// meshes without changing their actual vertices
		PDScalar dampingAlpha = 0,				// Rayleigh damping alpha coefficient 
		bool makeTets = false,					// If set to true, tet-gen will be called to created
												// a tet-mesh from the surface mesh. Required if
												// tet-strain constraints are used
		std::string meshURL = "",				// Required if tets are generated, since tet-gen
												// is used externally
		PDScalar rhsRegularizationWeight = 0.,	// In case few constraint samples are used, this
												// can be used to stabilize the inner forces via
												// a temporal regularization. Causes heavy damping.
		PDScalar yTranslation = 0				// Initial translation of the y-value of the verts.
	);
- After this, constraints need to be added to define the type of material.
  This can be done by one or more of the following functions:
		void addEdgeSprings(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax);
		void addTriangleStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax);
		void addTetStrain(PDScalar weight, PDScalar rangeMin, PDScalar rangeMax);
		void addBendingConstraints(PDScalar weight, bool preventBendingFlips, bool flatBending);
  For tet-strain, tets need to be available (see constructor above).
  For bending constraints, one can set the flag "flatBending" to true, which means that the mesh's
  rest-shape is assumed to be flat. This in turn means that the rhs contribution of the bending
  constraints will always be zero, and thus they can be skipped in the local step.
- Optionally, one can now add gravity, floor collisions and friction/repulsion to the system, via
		void addGravity(PDScalar g);
		void addFloor(int floorCoordinate, PDScalar floorHeight, PDScalar floorCollisionWeight);
		void setFrictionCoefficient(PDScalar fricCoeff, PDScalar repCoeff = -1.);
- Then the hyper-reduced simulation can be setup via the setup() method.
  This triggers the construction of the subspaces V and U, factorization of all involved systems,
  setting up communication with the GPU, etc.
- In the online phase, the essential methods are
		void step(int numIterations);
		PDPositions& getPositions();
  used to perform time-stepping and to get the current vertex positions (of the full mesh).
- There are more undocumented methods available to add collision constraints and user interaction.
  A good starting point to understand how these things can be used are the following methods:
	- For user interaction via soft constraints:
		void setGrip(std::vector<unsigned int>& vInds, PDPositions gripPos);
		void releaseGrip();
	- For collisions:
		void addCollisionsFromFile(std::string fileName);
		


