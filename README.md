# CAD Test Work, Subtract the Kinematic Solid from the Parallelepiped

Русский текст задания и его решение смотрите [по ссылке](README_rus.md).

## Task

A 3d Euclidian point cloud aligned in main axis directions *(x, y, z)* and with a constant distance grid (Figure 1) starting at a given reference point (the point with indices *0, 0, 0* is located at reference point) is intersected by a move of sphere (Figure 2), where the path of the sphere center is defined by a user given formula *x = f(t)* where *t* is in the interval between *t0* and *t1.* The function *f(t)* can be handled as a discrete function with a user given *∆t.* Points that intersect with the sphere move are considered as deleted (Figure 3 middle).

Only the first layer of points (which remains visible/undeleted) from top view must be written to a file as ASCII data (the skin of the point cloud from top view, see Figure 3 right). The file format is defined as follows:
- Each line contains a single point.
- The point definition contains *x, y* and *z* coordinates delimited by space characters.
- Each line ends with a new line character.

### Illustrations

![Task Illustrations](/pictures/01_Task_Illustrations.png)

### Given

- Point class for the definition of a point in 3d and (some) methods for vector algebra.
- High level test function ```CreateSkin(. . .)``` (in ```src/CreateSkin.cpp```) to test the resulting component. This defines also the interface of the component to be written.
- Program named 'PointVisualizer' (see ```tools/PointVisualizer_*``` files) which allows you to view the results you obtain in 3d.

### Hints

- Use of vector algebra (dot product, cross product etc.) is highly recommended. The use of sin and cos functions is not desired.
- Calculation speed and memory footprint are important but secondary in comparison to clearly readable code. E.g. a sphere, linear move of a sphere, point writer etc. might be modeled as classes. Prefer compact classes and functions over large classes and functions.
- Comments in the source code are welcome. Please use *good* names for classes and functions.
- Move of a sphere between *f(t)* and *f(t + ∆t)* can be assumed as a linear move of the center of the sphere.

## Solution

Regarding this task, a given point cloud can be considered as a certain geometric solid bounded by a parallelepiped of dimension *nX x nY x nZ,* located in space with a given reference point *referencePoint* and filled with points of a given crystal lattice.

![Parallelepiped](/pictures/02_Parallelepiped.png)

When moving a given sphere along an arbitrary user-defined function *f(t),* we obtain a solid of the motion, or a kinematic solid.

![Kinematic solid](/pictures/03_Kinematic_Solid.png)

Next, need to perform a Boolean operation of subtraction of the obtained kinematic solid *K* from the initial parallelepiped *P.* As a result, we get a new solid *P` = P − K.*

![Substracted solid](/pictures/04_Subtraction.png)

And then need to leave only the uppermost points from the resulting solid *P`,* which have the largest  value of the coordinates along the *Z* axis.

![Skin from substracted solid](/pictures/05_Skin_01.png) ![Skin from substracted solid](/pictures/06_Skin_02.png)

### Used Books

- Голованов Н. Н. Геометрическое моделирование. — М.: Издательство Физико-математической литературы, 2002. (Golovanov N.N. Geometric modeling. — Moscow: Publisher of physical and mathematical literature, 2002.)

### Code

Solution code with using the C++ and the vector algebra is presented in the ```src/TaskSolution.hpp``` and ```src/TaskSolution.cpp``` files.

CMake is used to build the project, for this reason the ```CMakeLists.txt``` file was created.

### Build

Make the following commands to build the project:

```
mkdir ./build
cd ./build
cmake ..
cmake --build .
```
