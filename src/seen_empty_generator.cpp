// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

std::vector<std::vector<std::vector<int>>> gridNums(std::size_t n)
{
	std::vector<std::vector<std::vector<int>>> grid_num(
	    4 * n, std::vector<std::vector<int>>(4 * n, std::vector<int>(4 * n)));

	for (std::size_t x{}; grid_num.size() != x; ++x) {
		for (std::size_t y{}; grid_num[x].size() != y; ++y) {
			for (std::size_t z{}; grid_num[x][y].size() != z; ++z) {
				grid_num[x][y][z] = (x / 4) + n * (y / 4) + n * n * (z / 4);
			}
		}
	}
	return grid_num;
}

std::string binaryToHex(std::string binary)
{
	static std::unordered_map<std::string, char> b2h = {
	    {"0000", '0'}, {"0001", '1'}, {"0010", '2'}, {"0011", '3'},
	    {"0100", '4'}, {"0101", '5'}, {"0110", '6'}, {"0111", '7'},
	    {"1000", '8'}, {"1001", '9'}, {"1010", 'A'}, {"1011", 'B'},
	    {"1100", 'C'}, {"1101", 'D'}, {"1110", 'E'}, {"1111", 'F'}};

	while (binary.size() % 4) {
		binary = '0' + binary;
	}

	std::string hex;
	for (std::size_t i{}; i + 3 < binary.size(); i += 4) {
		hex += b2h[binary.substr(i, 4)];
	}

	hex.erase(0, hex.find_first_not_of('0'));

	return hex.empty() ? "0" : hex;
}

std::uint64_t binaryToUnsigned(std::string binary)
{
	std::uint64_t res{};
	for (auto e : binary) {
		res = res << 1 | ('1' == e);
	}
	return res;
}

std::vector<std::string> gen(std::vector<std::vector<std::vector<int>>> const& grid_num,
                             int const x, int const y, int const z, int const num)
{
	assert(0 <= num);

	// TODO: Fix this so it works for other than 4
	std::vector<std::string> c(27, std::string(64, '0'));

	for (int ix{-num}; num >= ix; ++ix) {
		for (int iy{-num}; num >= iy; ++iy) {
			for (int iz{-num}; num >= iz; ++iz) {
				auto mx = (x + ix) % 4;
				auto my = (y + iy) % 4;
				auto mz = (z + iz) % 4;
				mx      = mx > 1 ? mx + 6 : mx;
				my      = my > 1 ? my + 6 : my;
				mz      = mz > 1 ? mz + 6 : mz;
				c[grid_num[x + ix][y + iy][z + iz]][63 - (mx + (2 * my) + (4 * mz))] = '1';
			}
		}
	}

	return c;
}

int main(int argc, char* argv[])
{
	// Settings
	std::size_t max_unknown_inflate = 4;

	std::ofstream output;

	std::ostringstream res;

	for (std::size_t ui{}; max_unknown_inflate >= ui; ++ui) {
		res << "if constexpr (" << ui << " == UnknownInflate) {\n";

		std::size_t a = std::ceil(ui / 4.0);
		std::size_t n = 1 + 2 * a;

		auto grid_num = gridNums(n);

		res << "\tstd::array<std::uint64_t, " << (n * n * n) << "> g{};\n";
		res << "\t\n";
		res << "\tKey k_o = Code(i, depth + 2);\n";
		res << "\t\n";
		res << "\tfor (int z{-" << a << "}, s = k_o.step(), i{}; " << a << " >= z; ++z) {\n";
		res << "\t\tfor (int y{-" << a << "}; " << a << " >= y; ++y) {\n";
		res << "\t\t\tfor (int x{-" << a << "}; " << a << " >= x; ++x, ++i) {\n";
		res << "\t\t\t\tauto k = k_o;\n";
		res << "\t\t\t\tk.x() += x * s;\n";
		res << "\t\t\t\tk.y() += y * s;\n";
		res << "\t\t\t\tk.z() += z * s;\n";
		res << "\t\t\t\tCode c_k = k;\n";
		res << "\t\t\t\tif (Code::equalAtDepth(code, c_k, Grid::depth() + depth)) {\n";
		res << "\t\t\t\t\tg[i] = free_grid[c_k.toDepth(depth)];\n";
		res << "\t\t\t\t} else if (auto it = free_grids.find(c_k.toDepth(Grid::depth() + "
		       "depth)); "
		       "std::end(free_grids) != it) {\n";
		res << "\t\t\t\t\tg[i] = it->second[c_k.toDepth(depth)];\n";
		res << "\t\t\t\t}\n";
		res << "\t\t\t}\n";
		res << "\t\t}\n";
		res << "\t}\n";
		res << "\n";

		std::size_t offset = 2 * (n - 1);
		for (std::size_t z{offset}; offset + 4 != z; ++z) {
			for (std::size_t y{offset}; offset + 4 != y; ++y) {
				for (std::size_t x{offset}; offset + 4 != x; ++x) {
					std::string node = z % 4 > 1 ? "1" : "0";
					node += y % 4 > 1 ? "1" : "0";
					node += x % 4 > 1 ? "1" : "0";
					node += z % 2 ? "1" : "0";
					node += y % 2 ? "1" : "0";
					node += x % 2 ? "1" : "0";
					auto const u = binaryToUnsigned(node);

					auto const g = gen(grid_num, x, y, z, ui);
					res << "mf[" << (u / 8) << "] |= std::uint_fast8_t((h & (1ull << " << u
					    << ")) && ";
					for (std::size_t i{}; g.size() != i; ++i) {
						auto const h = binaryToHex(g[i]);
						if ("0" != h) {
							res << "isMaskSet(g[" << i << "], 0x" << h << "ull) && ";
						}
					}
					res.seekp(-4, std::ios_base::end);
					res << ") << " << (u % 8) << ";\n";

					// for (int d{}; 5 != d; ++d) {
					// 	auto const g = gen(grid_num, x, y, z, ui);
					// 	res << "if (";
					// 	for (std::size_t i{}; g.size() != i; ++i) {
					// 		auto const h = binaryToHex(g[i]);
					// 		if ("0" != h) {
					// 			res << "isMaskSet(g[" << i << "], 0x" << h << "ull) && ";
					// 		}
					// 	}
					// 	res.seekp(-4, std::ios_base::end);
					// 	res << ") {\n";
					// }

					// auto const u = binaryToUnsigned(node);
					// res << "\tfg[4] |= std::uint64_t(1) << " << u << ";\n";
					// for (int d = 3; -1 != d; --d) {
					// 	res << "} else {\n";
					// 	res << "\tfg[" << d << "] |= std::uint64_t(1) << " << u << ";\n}\n";
					// }
					// res << "}\n\n";
				}
			}
		}

		res << "} else ";
	}
	res.seekp(-5, std::ios_base::end);
	res << "    ";

	output.open("/home/dduberg/ufomap2/include/ufo/map/integration/misses/1.hpp",
	            std::ios::out | std::ios::binary);
	output << res.str();
	output.close();

	res = {};
	res << "\tswitch (unknown_inflate) {\n";
	for (int ui{}; max_unknown_inflate != ui; ++ui) {
		res << "\t\tcase " << ui << ": return getMisses<" << ui
		    << ">(free_grids, hit_grids, depth, num_threads);\n";
	}
	res << "\t\tdefault: return getMisses<" << max_unknown_inflate
	    << ">(free_grids, hit_grids, depth, num_threads);\n";
	res << "\t}\n";

	output.open("/home/dduberg/ufomap2/include/ufo/map/integration/misses/2.hpp",
	            std::ios::out | std::ios::binary);
	output << res.str();
	output.close();

	return 0;
}