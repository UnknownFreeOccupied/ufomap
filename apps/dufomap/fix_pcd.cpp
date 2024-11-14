// UFO
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/points/points_predicate.hpp>
#include <ufo/map/predicate/satisfies.hpp>
#include <ufo/map/predicate/spatial.hpp>
#include <ufo/map/types.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/pose6.hpp>
#include <ufo/util/timing.hpp>

// STL
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "ufo/map/point.hpp"

// STL parallel
#ifdef UFO_TBB
#include <execution>
#endif

bool stob(std::string s, bool throw_on_error = true)
{
	auto result = false;  // failure to assert is false

	std::istringstream is(s);
	// first try simple integer conversion
	is >> result;

	if (is.fail()) {
		// simple integer failed; try boolean
		is.clear();
		is >> std::boolalpha >> result;
	}

	if (is.fail() && throw_on_error) {
		throw std::invalid_argument(s.append(" is not convertable to bool"));
	}

	return result;
}

void doStuff(std::filesystem::path folder, bool should_transform)
{
	if (std::filesystem::exists(folder / "original")) {
		std::ifstream poses_file(folder / "original/poses.csv", std::ios::binary);
		if (std::string line; std::getline(poses_file, line)) {
			std::cout << line << '\n';
		} else {
			std::cerr << "Error processing " << (folder / "original/poses.csv") << '\n';
			return;  // TODO: Handle error
		}

		std::unordered_map<std::size_t, ufo::Pose6d> poses;

		std::size_t index;
		double      timestamp;
		ufo::Pose6d pose;
		char        t;
		while (poses_file) {
			poses_file >> index >> t >> timestamp >> t >> pose.x() >> t >> pose.y() >> t >>
			    pose.z() >> t >> pose.qx() >> t >> pose.qy() >> t >> pose.qz() >> t >>
			    pose.qw();
			poses[index] = pose;
		}

		std::vector<std::filesystem::path> pcds;
		for (const auto& entry : std::filesystem::directory_iterator(folder / "original")) {
			if (".pcd" == entry.path().extension()) {
				pcds.push_back(entry.path().stem());
			}
		}
		std::sort(std::begin(pcds), std::end(pcds));

#ifdef UFO_TBB
		std::for_each(
		    std::execution::seq, std::cbegin(pcds), std::cend(pcds),
		    [&](std::string filename) {
			    std::cout << (filename + '\n');
			    pose = poses[std::stoull(filename)];

			    ufo::Cloud<ufo::Point, ufo::Color, ufo::Intensity> cloud;
			    ufo::readPointCloudPCD(folder / "original" / (filename + ".pcd"), cloud);

			    if (should_transform) {
				    // std::cout << "Applying transformation\n";
				    ufo::applyTransform(cloud, pose);
			    }

			    ufo::writePointCloudPCD(folder / "pcd" / (filename + ".pcd"), cloud, pose);
		    });
#else
		for (std::string filename : pcds) {
			std::cout << filename << '\n';
			pose = poses[std::stoull(filename)];
			std::cout << pose << '\n';

			ufo::Cloud<ufo::Point, ufo::Color, ufo::Intensity> cloud;
			ufo::readPointCloudPCD(folder / "original" / (filename + ".pcd"), cloud);

			if (should_transform) {
				ufo::applyTransform(cloud, pose);
			}

			ufo::writePointCloudPCD(folder / "pcd" / (filename + ".pcd"), cloud, pose);
		}
#endif
	} else {
		// Continue searching in subfolders
		for (const auto& entry : std::filesystem::directory_iterator(folder)) {
			if (std::filesystem::is_directory(entry)) {
				doStuff(entry, should_transform);
			}
		}
	}
}

int main(int argc, char* argv[])
{
	if (3 != argc) {
		std::cout << "Usage: fix_pcd SHOULD_TRANSFORM FOLDER_PATH\n";
		return 0;
	}

	bool should_transform = stob(argv[1]);

	std::filesystem::path folder(argv[2]);

	doStuff(folder, should_transform);

	return 0;
}