

			// Check done

			for (std::uint8_t d{}; fg.size() != d; ++d) {
				BitSet<8> const field(static_cast<BitSet<8>::T>(
				    // clang-format off
			       std::uint8_t(isMaskSet(fg[d], 0xFFull)) 
					| (std::uint8_t(isMaskSet(fg[d] >>  8, 0xFFull)) << 1) 
					| (std::uint8_t(isMaskSet(fg[d] >> 16, 0xFFull)) << 2) 
					| (std::uint8_t(isMaskSet(fg[d] >> 24, 0xFFull)) << 3) 
					| (std::uint8_t(isMaskSet(fg[d] >> 32, 0xFFull)) << 4) 
					| (std::uint8_t(isMaskSet(fg[d] >> 40, 0xFFull)) << 5) 
					| (std::uint8_t(isMaskSet(fg[d] >> 48, 0xFFull)) << 6) 
					| (std::uint8_t(isMaskSet(fg[d] >> 56, 0xFFull)) << 7)
				    // clang-format on
				    ));

				if (field.all()) {
					std::get<0>(misses[thread_id]).emplace_back(i, depth + 2);
					std::get<1>(misses[thread_id]).push_back(field);
					std::get<2>(misses[thread_id]).push_back(d + 1);
				} else if (field.any()) {
					std::get<0>(misses[thread_id]).emplace_back(i, depth + 1);
					std::get<1>(misses[thread_id]).push_back(field);
					std::get<2>(misses[thread_id]).push_back(d + 1);
				}

				if (!field.all()) {
					for (code_t j{}, k{}; 64 != j; j += 8, ++k) {
						if (!std::as_const(field)[k] && (0xFF & (fg[d] >> j))) {
							BitSet<8> f(static_cast<BitSet<8>::T>(
							    // clang-format off
					      (0x1ull  & (fg[d] >> j)) 
							| (0x2ull  & (fg[d] >> j)) 
							| (0x4ull  & (fg[d] >> j)) 
							| (0x8ull  & (fg[d] >> j)) 
							| (0x10ull & (fg[d] >> j)) 
							| (0x20ull & (fg[d] >> j)) 
							| (0x40ull & (fg[d] >> j)) 
							| (0x80ull & (fg[d] >> j))
							    // clang-format on
							    ));

							std::get<0>(misses[thread_id]).emplace_back(i | (j << 3 * depth), depth);
							std::get<1>(misses[thread_id]).push_back(f);
							std::get<2>(misses[thread_id]).push_back(d + 1);
						}
					}
				}
			}
		}
	}

	return misses;
}
}  // namespace impl
}  // namespace ufo

#endif  // UFO_MAP_INTEGRATION_MISSES_HPP