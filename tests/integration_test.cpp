// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/map/color/map.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Integrate Points")
{
	Integrator                    integrator_;
	Map3D<OccupancyMap, ColorMap> map(0.5, 3);
	PointCloud<3, float, Color>   cloud;

	cloud.emplace_back(Vec3f(0, 0, 0), Color(40, 50, 60));
	// std::cout << "cloud size: " << cloud.size() << std::endl;
	// std::cout << "cloud color: " << cloud[0].get<Color>() << std::endl;

	std::cout << "occupancy before:  " << map.occupancy(Vec3f(0, 0, 0)) << std::endl;
	std::cout << "color before:  " << map.color(Vec3f(0, 0, 0)) << std::endl;

	// std::cout << "map size: " << map.size() << std::endl;
	// integrator_.occupancy_hit = 0.9f;
	integrator_.insertPoints(execution::seq, map, cloud, false);

	// std::cout << "map size:  " << map.size() << std::endl;
	map.propagateModified();

	std::cout << "occupancy d=0:  " << map.occupancy(Vec3f(0, 0, 0)) << std::endl;
	std::cout << "occupancy d=1:  " << map.occupancy(TreeCoord{Vec3f(0, 0, 0), 1}) << std::endl;

	std::cout << "color d=0:  " << map.color(TreeCoord{Vec3f(0, 0, 0), 0}) << std::endl;
	std::cout << "color d=1:  " << map.color(TreeCoord{Vec3f(0, 0, 0), 1}) << std::endl;

	for (auto n : map) {
		std::cout << "n: " << n << ", modified: " << map.modified(n) << std::endl;
	}

	// std::filesystem::path path("/home/ramona/Desktop/ufomap.dot");
	// map.saveDotFile(path);
}