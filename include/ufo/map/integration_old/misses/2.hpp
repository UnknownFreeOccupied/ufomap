	switch (unknown_inflate) {
		case 0: return getMisses<0>(free_grids, hit_grids, depth, num_threads);
		case 1: return getMisses<1>(free_grids, hit_grids, depth, num_threads);
		case 2: return getMisses<2>(free_grids, hit_grids, depth, num_threads);
		case 3: return getMisses<3>(free_grids, hit_grids, depth, num_threads);
		default: return getMisses<4>(free_grids, hit_grids, depth, num_threads);
	}
