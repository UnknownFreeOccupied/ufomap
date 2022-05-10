/*
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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

#ifndef UFOMAP_RVIZ_PLUGINS_UFOMAP_RENDER_DATA_H
#define UFOMAP_RVIZ_PLUGINS_UFOMAP_RENDER_DATA_H

// UFO
#include <ufo/map/types.h>
#include <ufomap_rviz_plugins/data.h>
#include <ufomap_rviz_plugins/filter.h>
#include <ufomap_rviz_plugins/heatmap.h>
#include <ufomap_rviz_plugins/render_mode.h>
#include <ufomap_rviz_plugins/voxels.h>

// Ogre
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ufomap_ros::rviz_plugins
{
struct RenderData {
	RenderData();

	~RenderData();

	// Clear the points, but keep selection handler around
	void clear();

	void generateVoxels(RenderMode const& render_mode, Filter const& filter,
	                    Heatmap const& heatmap, double leaf_size);

	Ogre::SceneManager* manager_;

	Ogre::SceneNode* scene_node_ = nullptr;

	std::unordered_map<ufo::map::Depth, boost::shared_ptr<Voxels>> voxels_;
	RenderMode voxels_render_mode_;
	Filter voxels_filter_;
	Heatmap voxels_heatmap_;

	// TODO: selection_handler_;

	std::unordered_map<ufo::map::Depth, Data> transformed_voxels_;

	Ogre::Quaternion orientation_;
	Ogre::Vector3 position_;
};
}  // namespace ufomap_ros::rviz_plugins
#endif  // UFOMAP_RVIZ_PLUGINS_UFOMAP_RENDER_DATA_H