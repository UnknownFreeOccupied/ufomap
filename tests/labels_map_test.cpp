// UFO
#include <ufo/map/labels/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("LabelsMap")
{
	SECTION("Propagate ALL")
	{
		Map3D<LabelsMap> map(0.5, 3);
		map.labelsSetPropagationCriteria(LabelsPropagationCriteria::ALL);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);

		map.labelsUpdate(coord, 5);

		auto ls = map.labels(coord);
		REQUIRE(ls.find(5) != ls.end());

		ls = map.labels(coord_r);
		REQUIRE(ls.find(5) != ls.end());

		map.labelsUpdate(coord, 6);

		ls = map.labels(coord);
		REQUIRE(ls.find(5) != ls.end());
		REQUIRE(ls.find(6) != ls.end());

		ls = map.labels(coord_r);
		REQUIRE(ls.find(5) != ls.end());
		REQUIRE(ls.find(6) != ls.end());
	}

	SECTION("Propagate SUMMARY")
	{
		Map3D<LabelsMap> map(0.5, 3);
		map.labelsSetPropagationCriteria(LabelsPropagationCriteria::SUMMARY);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);

		map.labelsUpdate(coord, 5);
		
		auto ls = map.labels(coord);
		REQUIRE(ls.find(5) != ls.end());

		ls = map.labels(coord_r);
		REQUIRE(ls.find(5) != ls.end());

		map.labelsUpdate(coord, 6);

		ls = map.labels(coord);
		REQUIRE(ls.find(5) != ls.end());
		REQUIRE(ls.find(6) != ls.end());

		ls = map.labels(coord_r);
		REQUIRE(ls.size() == 1);
		REQUIRE(*ls.begin() == (5 | 6));
	}
}

TEST_CASE("LabelsMap - Query")
{
	SECTION("Propagate ALL")
	{
		Map3D<LabelsMap> map(0.5, 4);
		map.labelsSetPropagationCriteria(LabelsPropagationCriteria::ALL);

		Vec3f     coord(0, 0, 0);
		Vec3f     coord2(0, 0, 1);
		Vec3f     coord3(1, 0, 0);
		TreeCoord c(coord, 3);

		map.labelsUpdate(coord, 5);

		auto ls         = map.labels(coord);
		bool contains_5 = ls.find(5) != ls.end();
		REQUIRE(contains_5);

		ls                   = map.labels(c);
		bool root_contains_5 = ls.find(5) != ls.end();
		REQUIRE(root_contains_5);

		map.labelsUpdate(coord, 6);
		ls              = map.labels(coord);
		contains_5      = ls.find(5) != ls.end();
		bool contains_6 = ls.find(6) != ls.end();
		REQUIRE(contains_5);
		REQUIRE(contains_6);

		ls                   = map.labels(c);
		root_contains_5      = ls.find(5) != ls.end();
		bool root_contains_6 = ls.find(6) != ls.end();
		REQUIRE(root_contains_5);
		REQUIRE(root_contains_6);

		map.labelsUpdate(coord2, 15);
		map.labelsUpdate(coord3, 5);

		ls = map.labels(coord3);
		REQUIRE(ls.find(5) != ls.end());

		map.labelsUpdate(coord3, 55);
		ls = map.labels(coord3);
		REQUIRE(ls.find(55) != ls.end());

		// for (auto l : ls) {
		// 	std::cout << l << "\n";
		// }

		map.labelsSet(coord3, {71});
		ls = map.labels(coord3);
		REQUIRE(ls.size() == 1);
		REQUIRE(ls.find(71) != ls.end());

		// std::cout << std::endl << std::endl;
		// for (auto l : ls) {
		// 	std::cout << l << "\n";
		// }

		// for (auto n : map.query(ufo::pred::HasLabel())) {
		// 	std::cout << map.center(n) << "\n";
		// }

		// for (auto n : map.query(ufo::pred::Leaf() && ufo::pred::Label(5))) {
		// 	// auto& s = map.labels(n);
		// 	std::cout << map.center(n) << " -> ";
		// 	for (auto l : map.labels(n)) {
		// 		std::cout << l << ",";
		// 	}

		// 	std::cout << "\n";
		// }

		// std::filesystem::path file = "/home/ramona/ufomap.dot";
		// map.saveDotFile(file);
	}
}