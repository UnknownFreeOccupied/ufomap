/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_PREDICATE_DYNAMIC_HPP
#define UFO_MAP_PREDICATE_DYNAMIC_HPP

// UFO
#include <ufo/map/predicate/predicate.hpp>

// STL
#include <memory>
#include <type_traits>

namespace ufo::pred
{
namespace impl
{
template <class Map>
struct DynamicBase {
	virtual ~DynamicBase() {}

	[[nodiscard]] virtual bool valueCheck(typename Map::Base const&,
	                                      typename Map::Node) const = 0;

	[[nodiscard]] virtual bool innerCheck(typename Map::Base const&,
	                                      typename Map::Node) const = 0;

	[[nodiscard]] virtual DynamicBase* clone() const = 0;
};

template <class Map, class Predicate>
struct Dynamic
    : DynamicBase<Map>
    , Predicate {
	template <class... Args>
	Dynamic(Args&&... args) : Predicate(std::forward<Args>(args)...)
	{
	}

	template <class... Args>
	Dynamic(Map const&, Args&&... args) : Predicate(std::forward<Args>(args)...)
	{
	}

	Dynamic(Predicate const& pred) : Predicate(pred) {}

	Dynamic(Predicate&& pred) : Predicate(std::move(pred)) {}

	Dynamic(Map const&, Predicate const& pred) : Predicate(pred) {}

	Dynamic(Map const&, Predicate&& pred) : Predicate(std::move(pred)) {}

	virtual ~Dynamic() {}

	[[nodiscard]] bool valueCheck(typename Map::Base const& map,
	                              typename Map::Node        node) const override
	{
		return ValueCheck<Predicate>::apply(*this, map, node);
	}

	[[nodiscard]] bool innerCheck(typename Map::Base const& map,
	                              typename Map::Node        node) const override
	{
		return InnerCheck<Predicate>::apply(*this, map, node);
	}

 protected:
	Dynamic* clone() const override { return new Dynamic(*this); }
};
}  // namespace impl

template <class Map, class Predicate>
struct ValueCheck<impl::Dynamic<Map, Predicate>> {
	using Pred = impl::Dynamic<Map, Predicate>;

	[[nodiscard]] static bool apply(Pred const& p, typename Map::Base const& m,
	                                typename Map::Node n)
	{
		return p.valueCheck(m, n);
	}
};

template <class Map, class Predicate>
struct InnerCheck<impl::Dynamic<Map, Predicate>> {
	using Pred = impl::Dynamic<Map, Predicate>;

	[[nodiscard]] static bool apply(Pred const& p, typename Map::Base const& m,
	                                typename Map::Node n)
	{
		return p.innerCheck(m, n);
	}
};

template <class Map>
struct Predicate {
 public:
	Predicate() = default;

	Predicate(Predicate const& other)
	{
		if (other.hasPredicate()) {
			predicate_.reset(other.predicate_->clone());
		}
	}

	Predicate(Predicate&& other) = default;

	Predicate(impl::DynamicBase<Map> const& pred) : predicate_(pred.clone()) {}

	Predicate(impl::DynamicBase<Map>&& pred) : predicate_(pred.clone()) {}

	template <class Pred>
	Predicate(Pred const& pred)
	    : predicate_(std::make_unique<impl::Dynamic<Map, Pred>>(pred))
	{
	}

	template <class Pred>
	Predicate(Pred&& pred)
	    : predicate_(std::make_unique<impl::Dynamic<Map, Pred>>(std::forward<Pred>(pred)))
	{
	}

	Predicate& operator=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			predicate_.reset(rhs.predicate_->clone());
		}
		return *this;
	}

	Predicate& operator=(Predicate&&) = default;

	Predicate& operator=(impl::DynamicBase<Map> const& rhs)
	{
		predicate_.reset(rhs.clone());
		return *this;
	}

	Predicate& operator=(impl::DynamicBase<Map>&& rhs)
	{
		predicate_.reset(rhs.clone());
		return *this;
	}

	template <class Pred>
	Predicate& operator=(Pred const& pred)
	{
		predicate_ = std::make_unique<impl::Dynamic<Map, Pred>>(pred);
		return *this;
	}

	template <class Pred>
	Predicate& operator=(Pred&& pred)
	{
		predicate_ = std::make_unique<impl::Dynamic<Map, Pred>>(std::forward<Pred>(pred));
		return *this;
	}

	Predicate& operator&=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = *this && rhs;
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	Predicate& operator|=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = *this || rhs;
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	[[nodiscard]] bool hasPredicate() const { return !!predicate_; }

	[[nodiscard]] bool valueCheck(typename Map::Base const& map,
	                              typename Map::Node        node) const
	{
		return !hasPredicate() || predicate_->valueCheck(map, node);
	}

	[[nodiscard]] bool innerCheck(typename Map::Base const& map,
	                              typename Map::Node        node) const
	{
		return !hasPredicate() || predicate_->innerCheck(map, node);
	}

 private:
	std::unique_ptr<impl::DynamicBase<Map>> predicate_;
};

template <class Map>
struct ValueCheck<Predicate<Map>> {
	using Pred = Predicate<Map>;

	[[nodiscard]] static bool apply(Pred const& p, typename Map::Base const& m,
	                                typename Map::Node n)
	{
		return p.valueCheck(m, n);
	}
};

template <class Map>
struct InnerCheck<Predicate<Map>> {
	using Pred = Predicate<Map>;

	[[nodiscard]] static bool apply(Pred const& p, typename Map::Base const& m,
	                                typename Map::Node n)
	{
		return p.innerCheck(m, n);
	}
};
}  // namespace ufo::pred

#endif  // UFO_MAP_PREDICATE_DYNAMIC_HPP