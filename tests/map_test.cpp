// UFO
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/trans3.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

struct Point : Vec3f {
	Color c;
};

std::ostream& operator<<(std::ostream& out, Point p)
{
	return out << "Point: " << static_cast<Vec3f>(p) << ", Color: " << p.c;
}

TEST_CASE("Map")
{
	Map2D<OccupancyMap, ColorMap> map;

	auto node = map.create(Coord2f{{0.1, 0.0}, 2});
	std::cout << "Node: " << node << " with center: " << map.center(node) << '\n';
	std::cout << "Num nodes: " << (map.size() / map.branchingFactor()) << '\n';

	// auto pred = (pred::Modified{} || pred::Depth() == 5) && pred::Depth() != 16 &&
	//                     pred::Depth() != 6;
	auto pred = true;

	map.occupancySet(Coord2f({0.5, 0.1}, 0), 0.5, true);

	SECTION("With Propagation")
	{
		auto node = map.index(Coord2f({0.5, 0.1}, 1));
		map.occupancySet(node, 0.75, true);
		map.colorSet(map.code(node), 0, 100, 255, 255, true);
		map.colorSet(Coord2f({19.2, 19.2}, 7), 205, 0, 0, 255, true);
		// map.colorSet(Vec3d{19.2, 19.2, 19.2}, 205, 0, 0, 255, true);
		map.saveDotFile(std::filesystem::path("/home/dduberg/ufo/ufo_prop.dot"), pred);
	}

	SECTION("Without Propagation")
	{
		auto node = map.code(Coord2f({0.5, 0.1}, 1));
		map.occupancySet(node, 0.75, false);
		map.colorSet(node, 0, 100, 255, 255, false);
		map.colorSet(Coord2f({19.2, 19.2}, 7), 205, 0, 0, 255, false);
		map.saveDotFile(std::filesystem::path("/home/dduberg/ufo/ufo_no_prop.dot"), pred);
	}

	// std::vector<Point> points(10);

	// points[4].c = Color(42, 35, 4, 255);

	// Trans3f tf(Vec3f(0, 0, 1), {});

	// for (auto p : points) {
	// 	std::cout << p << std::endl;
	// }

	// std::cout << tf << std::endl;

	// // transform(tf, points.begin(), points.end(), points.begin());

	// transformInPlace(tf, points);

	// for (auto p : points) {
	// 	std::cout << p << std::endl;
	// }

	// P v;

	// Vec v2 = v;

	// std::cout << std::is_same_v<Vec3f, decltype(v2)> << std::endl;

	// map.setModified(Coord3f({0.0, 0.5, 0.5}, 0));

	// map.eraseChildren(Coord3f({0.8, 0.8, 0.8}, 4));
}