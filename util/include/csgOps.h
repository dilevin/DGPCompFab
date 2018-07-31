/* Define CSG operations for implicit surfaces here*/
#include <interopUtils.h>

//Union of two shapes: given shapes A,B and C then, if C = A U B, a point x in R3 is inside C if it is inside either A or B
Kernel::Tree unionAB(const Kernel::Tree &A, const Kernel::Tree &B);

//Intersection of two shapes: given shapes A,B and C then if C = A n B, a point x in R3 is inside C if it is inside both A and B
Kernel::Tree intersectAB(const Kernel::Tree &A, const Kernel::Tree &B);

//Difference of two shapes: given shapes A,B and C then if C = A -B, a point x in R3 is inside C if it is inside A and not inside B
Kernel::Tree differenceAB(const Kernel::Tree &A, const Kernel::Tree &B);


