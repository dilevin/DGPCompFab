auto s1 = sphere(2.0, {-0.5, 0.5, 0.5});
auto s2 = sphere(2.0, {1.0, 0.5, 0.5});
auto s1ns2 = differenceAB(s1,s2);

Region<3> r({ -5, -5, -5 }, { 5, 5, 5 });

auto mesh = Mesh::render(s1ns2, r, 0.025);