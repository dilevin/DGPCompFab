auto s1 = sphere(2.0, {-0.5, 0.0, 0.0});
auto s2 = sphere(2.0, {0.5, 0.0, 0.0});
auto s1Us2 = unionAB(s1,s2);

Region<3> r({ -5, -5, -5 }, { 5, 5, 5 });

auto mesh = Mesh::render(s1Us2, r, 0.025);