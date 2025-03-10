// UFO
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/transform.hpp>

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

// TEST_CASE("Map")
// {
// 	Map2D<OccupancyMap, ColorMap> map;

// 	auto node = map.create(QuadCoord{0.1, 0.0, 2});
// 	std::cout << "Node: " << node << " with center: " << map.center(node) << '\n';
// 	std::cout << "Num nodes: " << (map.size() / map.branchingFactor()) << '\n';

// 	// auto pred = (pred::Modified{} || pred::Depth() == 5) && pred::Depth() != 16 &&
// 	//                     pred::Depth() != 6;
// 	auto pred = true;

// 	map.occupancySet(QuadCoord(0.5, 0.1, 0), 0.5, true);

// 	SECTION("With Propagation")
// 	{
// 		auto node = map.index(QuadCoord(0.5, 0.1, 1));
// 		map.occupancySet(node, 0.75, true);
// 		map.colorSet(map.code(node), 0, 100, 255, 255, true);
// 		map.colorSet(QuadCoord(19.2, 19.2, 7), 205, 0, 0, 255, true);
// 		// map.colorSet(Vec3d{19.2, 19.2, 19.2}, 205, 0, 0, 255, true);
// 		map.saveDotFile(std::filesystem::path("/home/dduberg/ufo/ufo_prop.dot"), pred);
// 	}

// 	SECTION("Without Propagation")
// 	{
// 		auto node = map.code(QuadCoord(0.5, 0.1, 1));
// 		map.occupancySet(node, 0.75, false);
// 		map.colorSet(node, 0, 100, 255, 255, false);
// 		map.colorSet(QuadCoord(19.2, 19.2, 7), 205, 0, 0, 255, false);
// 		map.saveDotFile(std::filesystem::path("/home/dduberg/ufo/ufo_no_prop.dot"), pred);
// 	}

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
// }

#include <map>
// #include <random>
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/cloud/ufo.hpp>
#include <ufo/map/integrator/detail/inverse/map.hpp>
#include <ufo/utility/create_array.hpp>
#include <ufo/utility/index_iterator.hpp>
#include <ufo/utility/spinlock.hpp>

template <class Something>
[[nodiscard]] bool recurs(Something const& something, Transform3f const& pose,
                          unsigned depth, unsigned first_child, unsigned last_child,
                          std::vector<OctCoord>&                    res,
                          std::vector<float> const&                 sq_distances,
                          std::vector<std::vector<unsigned>> const& luts,
                          float min_dist_sq, float max_dist_sq)
{
	std::array<bool, 8> a{};

	auto const& lut = luts[depth];

	for (std::size_t i = 0, child = first_child; last_child > child; ++i, ++child) {
		auto const& s = something[depth][child];

		decltype(s.first_lut) last_lut = something[depth].size() > child + 1
		                                     ? something[depth][child + 1].first_lut
		                                     : luts[depth].size();

		IndexIterator ii(s.first_lut, last_lut);
		a[i] = s.sq_distance <= min_dist_sq ||
		       (s.sq_distance <= max_dist_sq &&
		        std::any_of(ii.begin(), ii.end(),
		                    [sqd = s.sq_distance, &sq_distances, &lut](std::size_t i) {
			                    return sqd <= sq_distances[lut[i]];
		                    }));
	}

	if (0 != depth) {
		for (std::size_t i = 0, child = first_child; last_child > child; ++i, ++child) {
			if (!a[i]) {
				continue;
			}

			auto const& s = something[depth][child];

			decltype(s.first_child) last_child = something[depth].size() > child + 1
			                                         ? something[depth][child + 1].first_child
			                                         : something[depth - 1].size();

			a[i] = recurs(something, pose, depth - 1, s.first_child, last_child, res,
			              sq_distances, luts, min_dist_sq, max_dist_sq);
		}
	}

	if (std::all_of(a.begin(), a.end(), [](bool a) { return a; })) {
		return true;
	}

	for (std::size_t i = 0, child = first_child; last_child > child; ++i, ++child) {
		if (!a[i]) {
			continue;
		}

		auto const& s = something[depth][child];
		res.emplace_back(pose(s.position), depth);
	}
	return false;
}

// template <class Something>
// void recurs(Something const& something, Transform3f const& pose, unsigned depth,
//             std::vector<unsigned> const& children, std::vector<OctCoord>& res,
//             std::vector<float> const&                 sq_distances,
//             std::vector<std::vector<unsigned>> const& luts)
// {
// 	auto const& lut = luts[depth];

// 	if (0 == depth) {
// 		for (std::size_t i{}; children.size() > i; ++i) {
// 			auto const& s = something[depth][children[i]];
// 			for (std::size_t j = s.first; s.last > j; ++j) {
// 				if (s.sq_distance <= sq_distances[lut[j]]) {
// 					res.emplace_back(pose(s.position), depth);
// 					break;
// 				}
// 			}
// 		}
// 	} else {
// 		for (std::size_t i{}; children.size() > i; ++i) {
// 			auto const& s = something[depth][children[i]];
// 			for (std::size_t j = s.first; s.last > j; ++j) {
// 				if (s.sq_distance <= sq_distances[lut[j]]) {
// 					recurs(something, pose, depth - 1, s.children, res, sq_distances, luts);
// 					break;
// 				}
// 			}
// 		}
// 	}
// }

TEST_CASE("[Octree] raycast")
{
	Map3D<detail::InverseMap> map(0.1f, 17);

	// TODO: Should be able to remove Color
	PointCloud<3, float, Color> cloud;

	Transform3f pose;

	readCloudUFO("/home/dduberg/ufo5/lib/viz/resources/kitti/ufo/0000_cloud.ufo", cloud,
	             pose);

	// TODO: Implement

	// std::mt19937                          gen(42);
	// std::uniform_real_distribution<float> dis(-10.0f, 10.0f);
	// for (std::size_t i{}; 100'000 > i; ++i) {
	// 	cloud.emplace_back(Vec3f(dis(gen), dis(gen), dis(gen)));
	// }

	float max_distance = 50.0f;

	bool                  load_from_file   = true;
	std::filesystem::path integration_file = "/home/dduberg/integration.ufo";

	std::vector<float> sq_distances(cloud.size());

	SECTION("Version 0")
	{
		std::cout << "Building base\n";
		auto start = std::chrono::high_resolution_clock::now();

		struct S {
			Vec3f         position;
			float         sq_distance{};
			std::uint32_t first_lut{};
			std::uint32_t first_child = std::numeric_limits<std::uint32_t>::max();

			constexpr S() = default;

			constexpr S(Vec3f position, float sq_distance, std::uint32_t first_lut)
			    : position(position), sq_distance(sq_distance), first_lut(first_lut)
			{
			}

			// Vec3f         position;
			// float         sq_distance{};
			// std::uint32_t first_child = std::numeric_limits<unsigned>::max();
			// std::uint32_t first_lut : 31;
			// std::uint32_t freed     : 1;

			// constexpr S() : first_lut(0u), freed(0u) {}

			// constexpr S(Vec3f position, float sq_distance, unsigned first_lut)
			//     : position(position), sq_distance(sq_distance), first_lut(first_lut),
			//     freed(0u)
			// {
			// }
		};

		std::vector<std::vector<S>>             something(5);
		std::vector<std::vector<std::uint32_t>> lut(something.size());

		std::vector<std::vector<std::uint32_t>> voids(something.size());
		std::vector<std::vector<unsigned char>> freed(something.size());

		if (load_from_file) {
			if (!std::filesystem::exists(integration_file)) {
				load_from_file = false;
			} else {
				std::fstream f(integration_file, std::ios::in | std::ios::binary);

				Vec3d         f_resolution;
				float         f_max_distance;
				std::uint64_t f_num_data;

				f.read(reinterpret_cast<char*>(&f_resolution), sizeof(f_resolution));
				f.read(reinterpret_cast<char*>(&f_max_distance), sizeof(f_max_distance));
				f.read(reinterpret_cast<char*>(&f_num_data), sizeof(f_num_data));

				if (map.length(0) != f_resolution || max_distance != f_max_distance ||
				    cloud.size() != f_num_data) {
					load_from_file = false;
				}
			}
		}

		if (load_from_file) {
			std::fstream f(integration_file, std::ios::in | std::ios::binary);

			Vec3d         f_resolution;
			float         f_max_distance;
			std::uint64_t f_num_data;
			std::uint32_t f_num_levels;

			f.read(reinterpret_cast<char*>(&f_resolution), sizeof(f_resolution));
			f.read(reinterpret_cast<char*>(&f_max_distance), sizeof(f_max_distance));
			f.read(reinterpret_cast<char*>(&f_num_data), sizeof(f_num_data));
			f.read(reinterpret_cast<char*>(&f_num_levels), sizeof(f_num_levels));

			something.resize(f_num_levels);
			lut.resize(f_num_levels);

			for (std::size_t i{}; f_num_levels > i; ++i) {
				std::uint64_t f_size;
				f.read(reinterpret_cast<char*>(&f_size), sizeof(f_size));

				auto& d = something[i];
				d.resize(f_size);
				f.read(reinterpret_cast<char*>(d.data()),
				       f_size * sizeof(remove_cvref_t<decltype(d)>::value_type));
			}

			for (std::size_t i{}; f_num_levels > i; ++i) {
				std::uint64_t f_size;
				f.read(reinterpret_cast<char*>(&f_size), sizeof(f_size));

				auto& d = lut[i];
				d.resize(f_size);
				f.read(reinterpret_cast<char*>(d.data()),
				       f_size * sizeof(remove_cvref_t<decltype(d)>::value_type));
			}
		} else {
			// std::vector<OctCode>   codes;
			// std::vector<TreeIndex> nodes;
			// for (std::size_t i{}; cloud.size() > i; ++i) {
			// 	LineSegment3 line(Vec3f(0, 0, 0),
			// 	                  max_distance * normalize(cloud[i].get<Vec3f>()));
			// 	for (auto n : map.query(pred::PureLeaf() && pred::Intersects(line), false)) {
			// 		codes.push_back(n.code);
			// 	}
			// 	nodes = map.modifiedSet(codes);
			// 	for (std::size_t j{}; codes.size() > j; ++j) {
			// 		map.integrationDistance(nodes[i]) = normSquared(map.center(codes[i]));
			// 		map.integrationIndices(nodes[i]).push_back(i);
			// 	}
			// 	codes.clear();
			// 	nodes.clear();
			// }
			std::vector<OctCode> codes;
			for (std::size_t i{}; cloud.size() > i; ++i) {
				LineSegment3 line(Vec3f(0, 0, 0),
				                  max_distance * normalize(cloud[i].get<Vec3f>()));
				for (auto n : map.query(pred::PureLeaf() && pred::Intersects(line), false)) {
					codes.push_back(n.code);
				}
				for (auto c : codes) {
					auto n                     = map.modifiedSet(c);
					map.integrationDistance(n) = normSquared(map.center(c));
					map.integrationIndices(n).push_back(i);
				}
				codes.clear();
			}
			map.propagateModified();

			for (auto n : map.query(pred::Depth() < something.size(), true, false)) {
				if (map.integrationIndices(n.index).empty()) {
					continue;
				}

				auto d = map.depth(n);

				if (something.size() > d + 1) {
					if (std::numeric_limits<unsigned>::max() ==
					    something[d + 1].back().first_child) {
						something[d + 1].back().first_child = something[d].size();
					}
				}

				something[d].emplace_back(map.center(n), map.integrationDistance(n.index),
				                          lut[d].size());
				lut[d].insert(lut[d].end(), map.integrationIndices(n.index).begin(),
				              map.integrationIndices(n.index).end());
			}

			for (auto& s : something) {
				s.shrink_to_fit();
			}
			for (auto& l : lut) {
				l.shrink_to_fit();
			}
		}

		auto stop = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> ms = stop - start;
		std::cout << "Built base " << ms.count() << " ms\n";

		printf("Depth\tSomething\tLUT \t\t S size (MiB) \t L size (MiB) \t Total (MiB)\n");
		double total_mib{};
		for (std::size_t d{}; something.size() > d; ++d) {
			printf("%lu\t%8lu\t%8lu\t%.2f \t\t %.2f \t\t %.2f\n", d, something[d].size(),
			       lut[d].size(), (something[d].size() * sizeof(S)) / (1024.0 * 1024.0),
			       (lut[d].size() * sizeof(unsigned)) / (1024.0 * 1024.0),
			       (((something[d].size() * sizeof(S)) + (lut[d].size() * sizeof(unsigned))) /
			        (1024.0 * 1024.0)));

			total_mib +=
			    (((something[d].size() * sizeof(S)) + (lut[d].size() * sizeof(unsigned))) /
			     (1024.0 * 1024.0));
		}

		std::cout << "Total memory: " << total_mib << " MiB\n";

		if (!load_from_file) {
			std::ofstream f(integration_file, std::ios::out | std::ios::binary);

			Vec3d         f_resolution   = map.length(0);
			float         f_max_distance = max_distance;
			std::uint64_t f_num_data     = cloud.size();
			std::uint32_t f_num_levels   = something.size();
			f.write(reinterpret_cast<char const*>(&f_resolution), sizeof(f_resolution));
			f.write(reinterpret_cast<char const*>(&f_max_distance), sizeof(f_max_distance));
			f.write(reinterpret_cast<char const*>(&f_num_data), sizeof(f_num_data));
			f.write(reinterpret_cast<char const*>(&f_num_levels), sizeof(f_num_levels));

			for (std::size_t i{}; f_num_levels > i; ++i) {
				auto const&   d      = something[i];
				std::uint64_t f_size = d.size();
				f.write(reinterpret_cast<char const*>(&f_size), sizeof(f_size));
				f.write(reinterpret_cast<char const*>(d.data()),
				        f_size * sizeof(remove_cvref_t<decltype(d)>::value_type));
			}
			for (std::size_t i{}; f_num_levels > i; ++i) {
				auto const&   d      = lut[i];
				std::uint64_t f_size = d.size();
				f.write(reinterpret_cast<char const*>(&f_size), sizeof(f_size));
				f.write(reinterpret_cast<char const*>(d.data()),
				        f_size * sizeof(remove_cvref_t<decltype(d)>::value_type));
			}
		}

		std::vector<OctCoord> res;
		// res.resize(something.size());
		// res.reserve(3470053);
		res.resize(something[0].size() - something[1].size());
		// res.reserve(something[0].size());

		std::deque<std::pair<std::thread::id, std::vector<OctCoord>>> ress;

		std::size_t num_clouds = 4500;

		auto start_whole = std::chrono::high_resolution_clock::now();
		for (std::size_t iter{}; num_clouds > iter; ++iter) {
			// Search
			start = std::chrono::high_resolution_clock::now();

			auto start_sq_norm = std::chrono::high_resolution_clock::now();

			std::transform(std::execution::par_unseq, cloud.begin<Vec3f>(), cloud.end<Vec3f>(),
			               sq_distances.begin(),
			               [](Vec3f const& point) { return normSquared(point); });

			float min_dist_sq = std::numeric_limits<float>::max();
			float max_dist_sq{};
			for (float sq : sq_distances) {
				min_dist_sq = std::min(min_dist_sq, sq);
				max_dist_sq = std::max(max_dist_sq, sq);
			}

			auto stop_sq_norm = std::chrono::high_resolution_clock::now();
			auto start_search = std::chrono::high_resolution_clock::now();

			std::atomic_size_t         ress_size = 0;
			std::mutex                 mutex;
			IndexIterator<std::size_t> ii(0, something.back().size());
			std::for_each(
			    std::execution::par, ii.begin(), ii.end(),
			    [&mutex, &ress, &ress_size, &sq_distances, &lut, &something, &pose, min_dist_sq,
			     max_dist_sq](std::size_t idx) {
				    S const& s = something.back()[idx];

				    auto depth = something.size() - 1;

				    auto const             id  = std::this_thread::get_id();
				    std::vector<OctCoord>* res = nullptr;
				    for (std::size_t i = 0, l = ress_size; l > i; ++i) {
					    if (id == ress[i].first) {
						    res = &ress[i].second;
						    break;
					    }
				    }

				    if (nullptr == res) {
					    std::scoped_lock lock(mutex);
					    if (ress.size() > ress_size) {
						    ress[ress_size].first = id;
						    res                   = &ress[ress_size].second;
						    res->clear();
						    ++ress_size;
					    } else {
						    res       = &ress.emplace_back(id, std::vector<OctCoord>{}).second;
						    ress_size = ress.size();
					    }
					    std::size_t num_threads = std::thread::hardware_concurrency();
					    res->reserve((something[0].size() - something[1].size()) /
					                 std::max(num_threads, ress.size()));
				    }

				    decltype(s.first_lut) last_lut = something.back().size() > idx + 1
				                                         ? something.back()[idx + 1].first_lut
				                                         : lut[depth].size();

				    IndexIterator ii(s.first_lut, last_lut);
				    if (!(s.sq_distance <= min_dist_sq ||
				          (s.sq_distance <= max_dist_sq &&
				           std::any_of(
				               ii.begin(), ii.end(),
				               [sqd = s.sq_distance, &sq_distances, &lut = lut[depth]](
				                   std::size_t i) { return sqd <= sq_distances[lut[i]]; })))) {
					    return;
				    }

				    decltype(s.first_child) last_child =
				        something.back().size() > idx + 1 ? something.back()[idx + 1].first_child
				                                          : something[depth - 1].size();

				    if (recurs(something, pose, depth - 1, s.first_child, last_child, *res,
				               sq_distances, lut, min_dist_sq, max_dist_sq)) {
					    res->emplace_back(pose(s.position), depth);
				    }
			    });

			auto stop_search   = std::chrono::high_resolution_clock::now();
			auto start_flatten = std::chrono::high_resolution_clock::now();

			std::size_t                                                       s{};
			std::vector<std::pair<std::vector<OctCoord> const*, std::size_t>> tmp;
			tmp.reserve(ress.size());
			for (auto const& [_, e] : ress) {
				tmp.emplace_back(&e, s);
				s += e.size();
			}
			// res.resize(s);

			std::for_each(std::execution::par, tmp.begin(), tmp.end(), [&res](auto const& e) {
				std::copy(e.first->begin(), e.first->end(), res.begin() + e.second);
				// std::memmove(res.data() + e.second, e.first->data(),
				//              e.first->size() * sizeof(OctCoord));
			});

			// auto        it = res.begin();
			// std::size_t s{};
			// for (auto const& [_, e] : ress) {
			// s += e.size();
			// it = std::copy(e.begin(), e.end(), it);
			// }
			auto stop_flatten = std::chrono::high_resolution_clock::now();

			stop = std::chrono::high_resolution_clock::now();
			ms   = stop - start;

			std::size_t total{};
			for (std::size_t i{}; s > i; ++i) {
				total += std::pow(8, res[i].depth);
			}

			// std::cout << "Total: " << total << '\n';

			// std::cout << "Search 0: " << s << " nodes in " << ms.count() << " ms ("
			//           << (1000.0 * s / ms.count()) << " nodes/s)\n";

			// std::chrono::duration<double, std::milli> ms_sq_norm = stop_sq_norm -
			// start_sq_norm; std::chrono::duration<double, std::milli> ms_search  = stop_search
			// - start_search; std::chrono::duration<double, std::milli> ms_flatten =
			// stop_flatten - start_flatten;

			// std::cout << "sq_norm: " << ms_sq_norm.count() << " ms\n";
			// std::cout << "search:  " << ms_search.count() << " ms\n";
			// std::cout << "flatten: " << ms_flatten.count() << " ms\n";

			auto stop_whole                        = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> ms_whole = stop_whole - start_whole;

			unsigned digits = num_clouds > 0 ? log10(num_clouds) + 1 : 1;

			static double total_time{};
			total_time += ms.count();

			static double last_total_time = std::numeric_limits<double>::lowest();
			if (0.25 < (total_time / 1000.0) - last_total_time || num_clouds == iter + 1) {
				last_total_time = (total_time / 1000.0);
				printf("\r[Total %.2f s][Current %.2f ms][Mean %.2f ms][%*lu/%lu]",
				       total_time / 1000.0, ms.count(), total_time / (iter + 1), digits, iter + 1,
				       num_clouds);
				fflush(stdout);
			}
		}
		std::cout << '\n';

		// auto                          stop_whole =
		// std::chrono::high_resolution_clock::now(); std::chrono::duration<double> ms_whole
		// = stop_whole - start_whole; std::cout << "Whole: " << ms_whole.count() << " s\n";
	}

	// SECTION("Version 1")
	// {
	// 	struct S {
	// 		Vec3f       position;
	// 		float       sq_distance;
	// 		std::size_t first;
	// 		std::size_t last;

	// 		S() = default;

	// 		S(Vec3f position, float sq_distance, std::size_t first, std::size_t last)
	// 		    : position(position), sq_distance(sq_distance), first(first), last(last)
	// 		{
	// 		}
	// 	};

	// 	std::vector<S>        something;
	// 	std::vector<unsigned> lut;

	// 	std::cout << "Building base\n";
	// 	auto start = std::chrono::high_resolution_clock::now();
	// 	{
	// 		// Create "base"

	// 		struct SS {
	// 			float                 sq_distance;
	// 			std::vector<unsigned> indices;
	// 		};

	// 		std::map<OctKey, SS> base;

	// 		float       step_size = map.length(0);
	// 		std::size_t steps     = max_distance / step_size;
	// 		for (std::size_t i{}; cloud.size() > i; ++i) {
	// 			Vec3f dir = step_size * normalize(cloud[i].get<Vec3f>());
	// 			for (std::size_t s{}; steps > s; ++s) {
	// 				auto k              = map.key(static_cast<float>(s) * dir);
	// 				base[k].sq_distance = (step_size * s) * (step_size * s);
	// 				base[k].indices.push_back(i);
	// 			}
	// 		}

	// 		std::vector<std::pair<OctKey, SS>> bb(base.begin(), base.end());

	// 		// std::sort(bb.begin(), bb.end(), [](auto const& a, auto const& b) {
	// 		// 	return a.second.sq_distance < b.second.sq_distance;
	// 		// });

	// 		std::cout << cloud.size() << std::endl;
	// 		std::cout << base.size() << std::endl;

	// 		for (auto& [key, value] : bb) {
	// 			std::size_t first = lut.size();
	// 			lut.insert(lut.end(), value.indices.begin(), value.indices.end());
	// 			std::size_t last = lut.size();
	// 			something.emplace_back(map.center(key), value.sq_distance, first, last);
	// 		}
	// 	}
	// 	auto stop = std::chrono::high_resolution_clock::now();
	// 	std::chrono::duration<double, std::milli> ms = stop - start;
	// 	std::cout << "Built base " << ms.count() << " ms\n";

	// 	std::vector<Vec3f> res;
	// 	// res.resize(something.size());
	// 	res.reserve(something.size());

	// 	// Search
	// 	start = std::chrono::high_resolution_clock::now();

	// 	std::transform(std::execution::par_unseq, cloud.begin<Vec3f>(), cloud.end<Vec3f>(),
	// 	               sq_distances.begin(),
	// 	               [](Vec3f const& point) { return normSquared(point); });

	// 	// std::atomic_uint s = 0;
	// 	// std::for_each(std::execution::par, something.begin(), something.end(),
	// 	//               [&s, &sq_distances, &lut, &res](S const& e) {
	// 	// 	              for (std::size_t i = e.first; e.last > i; ++i) {
	// 	// 		              if (e.sq_distance <= sq_distances[lut[i]]) {
	// 	// 			              res[s++] = e.position;
	// 	// 			              break;
	// 	// 		              }
	// 	// 	              }
	// 	//               });

	// 	// res.resize(s);

	// 	for (S const& e : something) {
	// 		for (std::size_t i = e.first; e.last > i; ++i) {
	// 			if (e.sq_distance <= sq_distances[lut[i]]) {
	// 				res.push_back(e.position);
	// 				break;
	// 			}
	// 		}
	// 	}

	// 	std::for_each(std::execution::par_unseq, res.begin(), res.end(),
	// 	              [&pose](auto& p) { p = pose(p); });

	// 	stop = std::chrono::high_resolution_clock::now();
	// 	ms   = stop - start;

	// 	std::cout << "Search 1: " << res.size() << " nodes in " << ms.count() << " ms ("
	// 	          << (res.size() / ms.count()) << " nodes/ms)\n";
	// }

	// SECTION("Version 2")
	// {
	// 	std::vector<std::vector<std::pair<float, unsigned>>> lut(cloud.size());

	// 	std::cout << "Building base\n";
	// 	auto start = std::chrono::high_resolution_clock::now();

	// 	// Create "base"

	// 	std::map<OctKey, unsigned> base;

	// 	std::vector<Vec3f> tmp_points;

	// 	float       step_size = map.length(0);
	// 	std::size_t steps     = max_distance / step_size;
	// 	for (std::size_t i{}; cloud.size() > i; ++i) {
	// 		Vec3f dir = step_size * normalize(cloud[i].get<Vec3f>());
	// 		for (std::size_t s{}; steps > s; ++s) {
	// 			auto k = map.key(static_cast<float>(s) * dir);
	// 			if (base.try_emplace(k, tmp_points.size()).second) {
	// 				tmp_points.push_back(map.center(k));
	// 			}
	// 			lut[i].emplace_back((step_size * s) * (step_size * s), base[k]);
	// 		}
	// 	}

	// 	std::vector<std::pair<std::atomic_bool, Vec3f>> points(tmp_points.size());
	// 	for (std::size_t i{}; tmp_points.size() > i; ++i) {
	// 		points[i].first  = false;
	// 		points[i].second = tmp_points[i];
	// 	}

	// 	auto stop = std::chrono::high_resolution_clock::now();
	// 	std::chrono::duration<double, std::milli> ms = stop - start;
	// 	std::cout << "Built base " << ms.count() << " ms\n";

	// 	std::vector<Vec3f> res;
	// 	// res.resize(points.size());
	// 	res.reserve(points.size());

	// 	// Search
	// 	start = std::chrono::high_resolution_clock::now();

	// 	std::transform(std::execution::par_unseq, cloud.begin<Vec3f>(), cloud.end<Vec3f>(),
	// 	               sq_distances.begin(),
	// 	               [](Vec3f const& point) { return normSquared(point); });

	// 	// std::atomic_uint s = 0;

	// 	IndexIterator<std::size_t> ii(0, sq_distances.size());
	// 	std::for_each(std::execution::par, ii.begin(), ii.end(),
	// 	              // [&s, &res, &sq_distances, &points, &lut](std::size_t i) {
	// 	              [&sq_distances, &points, &lut](std::size_t i) {
	// 		              for (auto const& [sq_dist, idx] : lut[i]) {
	// 			              if (sq_dist > sq_distances[i]) {
	// 				              break;
	// 			              }

	// 			              // if (!points[idx].first.exchange(true)) {
	// 			              //   res[s++] = points[idx].second;
	// 			              // }

	// 			              points[idx].first = true;
	// 		              }
	// 	              });

	// 	for (auto const& [enable, point] : points) {
	// 		if (enable) {
	// 			res.push_back(point);
	// 		}
	// 	}

	// 	// std::unordered_map<std::thread::id, std::vector<Vec3f>> ress;
	// 	// std::for_each(std::execution::par_unseq, points.begin(), points.end(),
	// 	//               [&ress](auto const& e) {
	// 	// 	              auto& res = ress[std::this_thread::get_id()];
	// 	// 	              // res.reserve(something[0].size());
	// 	// 	              if (e.first) {
	// 	// 		              res.push_back(e.second);
	// 	// 	              }
	// 	//               });

	// 	// std::size_t s{};
	// 	// for (auto const& [_, e] : ress) {
	// 	// 	s += e.size();
	// 	// }
	// 	// res.resize(s);
	// 	// auto               it = res.begin();
	// 	// std::atomic_size_t ss = 0;
	// 	// std::for_each(std::execution::par_unseq, ress.begin(), ress.end(),
	// 	//               [&res, &ss](auto const& e) {
	// 	// 	              std::size_t s = ss.fetch_add(e.second.size());
	// 	// 	              std::copy(e.second.begin(), e.second.end(), res.begin() + s);
	// 	//               });

	// 	// res.resize(s);

	// 	std::for_each(std::execution::par_unseq, res.begin(), res.end(),
	// 	              [&pose](auto& p) { p = pose(p); });

	// 	stop = std::chrono::high_resolution_clock::now();
	// 	ms   = stop - start;

	// 	std::cout << "Search 2: " << res.size() << " nodes in " << ms.count() << " ms ("
	// 	          << (res.size() / ms.count()) << " nodes/ms)\n";
	// }
}