//Define variables for parametric shape
auto scale = 0.5f;
auto radius = 1.5f;
auto thickness = 0.5;

//Functions are defines as C++ lambdas
//sphereGyroidFunc - the function name
//[&scale, &thickness] - read in the global scale and thickness variables so that 
//                       they can be used in the function
//(double radius, Eigen::Vector3f pos) - parameters passed to the function. 
auto sphereGyroidFunc = [&scale, &thickness](double radius, Eigen::Vector3f pos) {
  
  auto gyroidSrf = sin(Kernel::Tree::X() / scale) * cos(Kernel::Tree::Y() / scale) +
  sin(Kernel::Tree::Y() / scale) * cos(Kernel::Tree::Z() / scale) +
  sin(Kernel::Tree::Z() / scale) * cos(Kernel::Tree::X() / scale);

    auto gyroid = shell(gyroidSrf, thickness);
    auto sphere1 = sphere(radius, pos);
    
    auto sphereGyroid = max(sphere1, gyroid);
    sphereGyroid = min(sphereGyroid,
                       min(sphereGyroid ,
                           (sqrt(abs(sphereGyroid)) + sqrt(abs( sphereGyroid ))) - .5));

    return sphereGyroid; 
};


//Make a box with lower left corner and (-2,-2,-2) and upper right corner at (2,2,2)
auto box1 = box({ -2.f,-2.f,-2.f }, { 2.f,2.f,2.f });

//Libfive represents objects as functions over all of 3D space
//Here we build a guroid using a product of sins and cosines acting on
//coordinate axes. The coordinates are represented by Kernel::Tree::X(),  
//Kernel::Tree::Y() and Kernel::Tree::Z(). All objects created in libfive are of 
//the type Kernel::Tree.
auto gyroidSrf = sin(Kernel::Tree::X() / scale) * cos(Kernel::Tree::Y() / scale) +
  sin(Kernel::Tree::Y() / scale) * cos(Kernel::Tree::Z() / scale) +
  sin(Kernel::Tree::Z() / scale) * cos(Kernel::Tree::X() / scale);

auto gyroid = shell(gyroidSrf, thickness);
  
//Combine the box and the gyroid
auto boxGyroid = max(box1, gyroid);
boxGyroid = min(boxGyroid,
                    min(boxGyroid ,
                        (sqrt(abs(boxGyroid)) + sqrt(abs( boxGyroid ))) - .5));


//Call the function defined above
auto sg1 = sphereGyroidFunc(radius, {2.0,2.0,2.0});
auto sg2 = sphereGyroidFunc(radius, {-2.0,2.0,2.0});
auto sg3 = sphereGyroidFunc(radius, {2.0,-2.0,2.0});
auto sg4 = sphereGyroidFunc(radius, {-2.0,-2.0,2.0});
auto sg5 = sphereGyroidFunc(radius, {2.0,2.0,-2.0});
auto sg6 = sphereGyroidFunc(radius, {-2.0,2.0,-2.0});
auto sg7 = sphereGyroidFunc(radius, {2.0,-2.0,-2.0});
auto sg8 = sphereGyroidFunc(radius, {-2.0,-2.0,-2.0});
auto finalShape = min(min(min(min(min(min(min(min(boxGyroid, sg1), sg2), sg3),sg4),sg5),sg6),sg7),sg8);
//Specify a region over which to reconstruct the objects surface mesh.
Region<3> r({ -5, -5, -5 }, { 5, 5, 5 });

//This line of code builds a triangle mesh from an implicit shape representation.
//finalShape is the implicit shape whos triangle mesh will be reconstructued. 
//r is the bounding box inside which to reconstruct the shape.
//the final parameter is the value of the isosurface to extract.
//NOTE: You must have a line of the form auto mesh = Mesh::render(shape, region, threshold)
//      at the end of your script. Omitting this will result in no geometry
//      being created. 
//NOTE 2: the output variable must be named mesh
auto mesh = Mesh::render(finalShape, r, 0.025);