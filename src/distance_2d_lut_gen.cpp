#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

void print(std::array<std::array<bool, 13>, 13> const& adj)
{
	for (unsigned i{}; adj.size() > i; ++i) {
		for (unsigned j{}; adj[i].size() > j; ++j) {
			std::cout << adj[i][j] << ' ';
		}
		std::cout << '\n';
	}
}

[[nodiscard]] std::vector<std::pair<unsigned, unsigned>> generate(unsigned idx)
{
	std::array<std::array<bool, 13>, 13> adj{};

	// Center is always there
	adj[4][4] = true;

	for (unsigned i{}; 8 > i; ++i) {
		if (4 <= i) {
			adj[i + 1][i + 1] = (idx & (1u << i)) ? true : false;
		} else {
			adj[i][i] = (idx & (1u << i)) ? true : false;
		}
	}

	for (unsigned i = 9; 13 > i; ++i) {
		adj[i][i] = true;
	}

	// if (1 == idx) {
	// 	std::cout << '\n';
	// 	print(adj);
	// 	exit(0);
	// }

	// 0 -> 1
	// 0 -> 3
	// 0 -> 9
	adj[0][1] = adj[0][0] && adj[1][1];
	adj[0][3] = adj[0][0] && adj[3][3];
	adj[0][9] = adj[0][0] && !(adj[1][1] && adj[3][3]);

	// 1 -> 2
	// 1 -> 4
	// 1 -> 9
	// 1 -> 10
	adj[1][2]  = adj[1][1] && adj[2][2];
	adj[1][4]  = adj[1][1] && ((!adj[0][0] && !adj[3][3]) || (!adj[2][2] && !adj[5][5]));
	adj[1][9]  = adj[1][1] && !adj[0][0] && adj[3][3];
	adj[1][10] = adj[1][1] && !adj[2][2] && adj[5][5];

	// 2 -> 5
	// 2 -> 10
	adj[2][5]  = adj[2][2] && adj[5][5];
	adj[2][10] = adj[2][2] && !(adj[1][1] && adj[5][5]);

	// 3 -> 4
	// 3 -> 6
	// 3 -> 9
	// 3 -> 11
	adj[3][4]  = adj[3][3] && ((!adj[0][0] && !adj[1][1]) || (!adj[6][6] && !adj[7][7]));
	adj[3][6]  = adj[3][3] && adj[6][6];
	adj[3][9]  = adj[3][3] && !adj[0][0] && adj[1][1];
	adj[3][11] = adj[3][3] && !adj[6][6] && adj[7][7];

	// 4 -> 5
	// 4 -> 7
	// 4 -> 9
	// 4 -> 10
	// 4 -> 11
	// 4 -> 12
	// 4, 4 always exists
	adj[4][5]  = ((!adj[1][1] && !adj[2][2]) || (!adj[7][7] && !adj[8][8]));
	adj[4][7]  = ((!adj[3][3] && !adj[6][6]) || (!adj[5][5] && !adj[8][8]));
	adj[4][9]  = adj[0][0] && !(adj[1][1] && adj[3][3]);
	adj[4][10] = adj[2][2] && !(adj[1][1] && adj[5][5]);
	adj[4][11] = adj[6][6] && !(adj[3][3] && adj[7][7]);
	adj[4][12] = adj[8][8] && !(adj[5][5] && adj[7][7]);

	// 5 -> 8
	// 5 -> 10
	// 5 -> 12
	adj[5][8]  = adj[5][5] && adj[8][8];
	adj[5][10] = adj[5][5] && adj[1][1] && !adj[2][2];
	adj[5][12] = adj[5][5] && adj[7][7] && !adj[8][8];

	// 6 -> 7
	// 6 -> 11
	adj[6][7]  = adj[6][6] && adj[7][7];
	adj[6][11] = adj[6][6] && !(adj[3][3] && adj[7][7]);

	// 7 -> 8
	// 7 -> 11
	// 7 -> 12
	adj[7][8]  = adj[7][7] && adj[8][8];
	adj[7][11] = adj[7][7] && adj[3][3] && !adj[6][6];
	adj[7][12] = adj[7][7] && adj[5][5] && !adj[8][8];

	// 8 -> 12
	adj[8][12] = adj[8][8] && !(adj[5][5] && adj[7][7]);

	std::vector<std::pair<unsigned, unsigned>> edges;

	for (unsigned i{}; adj.size() > i; ++i) {
		for (unsigned j = i + 1; adj[i].size() > j; ++j) {
			if (adj[i][i] && adj[j][j] && adj[i][j]) {
				edges.emplace_back(i, j);
			}
		}
	}

	return edges;
}

int main(int argc, char* argv[])
{
	for (unsigned i{}; std::numeric_limits<std::uint8_t>::max() >= i; ++i) {
		// std::cout << "Index: " << i << " (" << std::bitset<8>(i) << ")\n";
		std::cout << R"(std::array<std::uint8_t, 25>{)";
		auto a = generate(i);
		std::cout << (2 * a.size());
		for (auto [a, b] : a) {
			std::cout << ", " << a << ", " << b;
		}
		std::cout << "},\n";
	}

	return 0;
}