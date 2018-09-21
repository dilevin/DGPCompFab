# University of Toronto CSC2521: Computational Design and Fabrication

This github is the repository for all the assignments for  the University of Toronto Course: Computational Design and Fabrication.

Please use the issue tracker to report problems with the assignment code.

## Brief Explanation of LibFive
You will notice that methods in these assignment take in variables of the type  Kernel::Tree (i.e Kernel::Tree A). 

The code treats A as an implicit function A(x). At runtime every x in the scene is passed to A and the surface is evaluated by collecting all x's for which A(x) < 0.  

You manipulate shapes by crafting new functions. For instance, let's say you wanted to create a new shape H, that involved the sum (this is different from the union) of two other shapes A and B. You would right

H = A + B

For any given x, the value of H will be A(x) + B(x). 

You can see examples of more manipulations on the libfive homepage or in the shapes.cpp in the utils directory.

Libfive supports most standard mathematical operations (+,-,max, min,sin,cos .... ). 
