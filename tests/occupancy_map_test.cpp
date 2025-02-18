// UFO
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("OccupacyMap")
{
	Map3D<OccupancyMap> map(0.5, 3);

	Vec3f coord(0, 0, 0);
	float occ = 0.95f;

	// map.occupancySet(coord, occ);

	std::cout << "map.occupancyLogit: " << map.occupancyLogit(0.9f) << std::endl;
	map.occupancyUpdateLogit(coord, map.occupancyLogit(0.9f), false);

    // map.propagateModified();

	for (auto n : map) {
		std::cout << "n: " << n << ", modified: " << map.modified(n) << std::endl;
	}

	REQUIRE(!map.containsUnknown(coord));
	REQUIRE(!map.containsFree(coord));
	REQUIRE(map.containsOccupied(coord));
	// REQUIRE(occ == Catch::Approx(map.occupancy(coord)));

	std::cout << "u: " << map.containsUnknown(coord) << "\nf: " << map.containsFree(coord)
	          << "\no: " << map.containsOccupied(coord) << std::endl;

	auto value = map.occupancy(coord);

	std::cout << "value d=0: " << value << std::endl;
	std::cout << "value d=1: " << map.occupancy(TreeCoord(coord, 2)) << std::endl;
}