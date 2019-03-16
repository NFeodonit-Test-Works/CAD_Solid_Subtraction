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
- High level test function *CreateSkin(. . .)* (in 'CreateSkin.cpp') to test the resulting component. This defines also the interface of the component to be written.
- Program named 'PointVisualizer' which allows you to view the results you obtain in 3d.

### Hints

- Use of vector algebra (dot product, cross product etc.) is highly recommended. The use of sin and cos functions is not desired.
- Calculation speed and memory footprint are important but secondary in comparison to clearly readable code. E.g. a sphere, linear move of a sphere, point writer etc. might be modeled as classes. Prefer compact classes and functions over large classes and functions.
- Comments in the source code are welcome. Please use *good* names for classes and functions.
- Move of a sphere between *f(t)* and *f(t + ∆t)* can be assumed as a linear move of the center of the sphere.
