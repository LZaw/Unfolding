# Unfolding

This repository contains the code of the following papers:

- [**Unfolding Polyhedra via Tabu Search**](https://doi.org/10.1007/s00371-024-03395-2) (The Visual Computer)
- [**Unfolding via Mesh Approximation using Surface Flows**](https://doi.org/10.1111/cgf.15031) (Computer Graphics Forum, Vol. 43-2)
- **TBA** (Code, links, and references will follow on acceptance)

## Notes and Warnings

None at the moment.

## Requirements

To compile and run the code you need the following things:

- A C++ compiler supporting C++17 (C++>=11 may also work with some tweaks)
- CMake >= 3.13
- Eigen (included as a submodule)
- libIGL (included as a submodule)

If you want to run the code with the GUI (CMake option `WITH_FRONTEND=ON`), you also need the following:

- OpenGL (tested with 4.6)
- Spectralib (included as a submodule)

The code has been written and tested under Linux with GCC (7.5) and Clang (10). Any other operating system or compiler may or may not need additional work to adjust.

## License

The submodules are licensed independelty of this repository.  
You may use the code of this repository as you like. If you use the code for a scientific work, please cite the most fitting paper(s) mentioned above.
Of course, the software is provided as is and I don't take any responsibility for anything that results from using it.

## Structure

The main repository contains the following folders:

- **external**  
  The submodules, which are needed to run the code
- **shader**  
  The shaders for the GUI.
- **src**  
  All source and header files of the repository.
  - **Collisions**  
    Everything about collisions: Detection and resolving.
  - **Datastructures**  
    All relevant datastructures for the main code.
  - **Model**  
    The semantical connection between the different datastructures and functionalities.
  - **SVG**  
    Saving SVG files.
  - **Transformer**  
    The Surface Flows of the paper *Unfolding via Mesh Approximation using Surface Flows*.
  - **Unfolding**  
    The initial unfolders for a mesh.
  - **Util**  
    Different utilities which didn't fit anywhere else.
  - **View**  
    All related to the GUI. This folder is ignored if `WITH_FRONTEND` is set to `OFF` in CMake.

## Building

Use the following lines to build this repository:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_FRONTEND=OFF ..
make
```
To also use the GUI set the `WITH_FRONTEND` option to `ON`.

## Usage

When using the GUI, just check out the menu. The entries should be self-explaining.  
Without GUI, you can use the program as follows:
```
./Unfolding inputModel resolvingMethod abortTime pathToUnfolding [pathToApproximativeModel]
```
The time is given in seconds.
For the resolving methods you can currently use:

- TU
- AEF
- cMCF
- CCF0
- CCF1
- CCF2
- ENAF
- IIF

## FAQ

**Q:** I'm getting errors when compiling, what do I do?  
**A:** You may contact me and possibly I can help you.

**Q:** I found a bug/error/...  
**A:** Either contact me, or open up a pull request / issue, please.

**Q:** Where is ... in the code?  
**A:** Above there is a listing of the repository structure. This should help you finding our way around.

**Q:** I want this really cool feature, which is not in the code yet.  
**A:** Drop me a mail and we can check if/how to include it.

**Q:** I want to collaborate on a future project.  
**A:** Please write me a mail. I'm always interested.

**Q:** Can I use your code or parts of it?  
**A:** Yes (please check the licenses of each submodule!). If it is for a scientific work, please cite the most fitting paper(s) mentioned above.
