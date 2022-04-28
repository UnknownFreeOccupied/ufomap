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

#ifndef UFOMAP_RVIZ_PLUGINS_RENDERABLE_H
#define UFOMAP_RVIZ_PLUGINS_RENDERABLE_H

// UFO
#include <ufomap_rviz_plugins/render_mode.h>

// OGRE
#include <OGRE/OgreAxisAlignedBox.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreSimpleRenderable.h>
#include <OGRE/OgreString.h>
#include <OGRE/OgreVector3.h>

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <cstdint>
#include <vector>

namespace ufomap_ros::rviz_plugins
{
#define SIZE_PARAMETER 0
#define ALPHA_PARAMETER 1
#define PICK_COLOR_PARAMETER 2
#define NORMAL_PARAMETER 3
#define UP_PARAMETER 4
#define HIGHLIGHT_PARAMETER 5
#define AUTO_SIZE_PARAMETER 6

class Voxels;

class VoxelsRenderable : public Ogre::SimpleRenderable
{
 public:
	VoxelsRenderable(Voxels* parent, int num_voxels, bool use_tex_coords);

	~VoxelsRenderable() override;

	using Ogre::SimpleRenderable::getRenderOperation;

	Ogre::RenderOperation* getRenderOperation() { return &mRenderOp; }

	Ogre::HardwareVertexBufferSharedPtr getBuffer();

	Ogre::Real getBoundingRadius() const override;

	Ogre::Real getSquaredViewDepth(Ogre::Camera const* cam) const override;

	unsigned short getNumWorldTransforms() const override { return 1; }

	void getWorldTransforms(Ogre::Matrix4* xform) const override;

	Ogre::LightList const& getLights() const override;

 private:
	Ogre::MaterialPtr material_;
	Voxels* parent_;
};

using VoxelsRenderablePtr = boost::shared_ptr<VoxelsRenderable>;
using V_VoxelsRenderable = std::vector<VoxelsRenderablePtr>;

class Voxels : public Ogre::MovableObject
{
 public:
	Voxels();

	~Voxels() override;

	void clear();

	struct Voxel {
		Voxel() = default;

		Voxel(Ogre::Real x, Ogre::Real y, Ogre::Real z)
		    : position(x, y, z), color(0.0, 0.0, 0.0, 1.0)
		{
		}

		Voxel(Ogre::Real x, Ogre::Real y, Ogre::Real z, float r, float g, float b,
		      float a = 1.0)
		    : position(x, y, z), color(r, g, b, a)
		{
		}

		Voxel(Ogre::Vector3 const& position, Ogre::ColourValue const& color)
		    : position(position), color(color)
		{
		}

		inline void setColor(float r, float g, float b, float a = 1.0)
		{
			color = Ogre::ColourValue(r, g, b, a);
		}

		Ogre::Vector3 position;
		Ogre::ColourValue color;
	};

	void addVoxels(Voxel* voxels, uint32_t num_voxels);

	void popVoxels(uint32_t num_voxels);

	void setRenderStyle(RenderStyle style);

	void setDimensions(float size);

	void setCommonDirection(Ogre::Vector3 const& vec);

	void setCommonUpVector(Ogre::Vector3 const& vec);

	void setAlpha(float alpha, bool per_voxel_alpha = false);

	void setPickColor(Ogre::ColourValue const& color);

	void setColorByIndex(bool set);

	void setHighlightColor(float red, float green, float blue);

	Ogre::String const& getMovableType() const override { return sm_Type; }

	Ogre::AxisAlignedBox const& getBoundingBox() const override;

	float getBoundingRadius() const override;

	virtual void getWorldTransforms(Ogre::Matrix4* xform) const;

	virtual unsigned short getNumWorldTransforms() const { return 1; }

	void _updateRenderQueue(Ogre::RenderQueue* queue) override;

	void _notifyAttached(Ogre::Node* parent, bool isTagVoxel = false) override;

	void visitRenderables(Ogre::Renderable::Visitor* visitor,
	                      bool debugRenderables) override;

	virtual void setName(std::string const& name) { mName = name; }

 private:
	uint32_t getVerticesPerVoxel();

	VoxelsRenderablePtr createRenderable(int num_voxels);

	void regenerateAll();

	void shrinkRenderables();

	Ogre::AxisAlignedBox bounding_box_;  // The bounding box of this voxel cloud
	float bounding_radius_;              // The bounding radius of this voxel cloud

	using V_Voxel = std::vector<Voxel>;
	// The list of voxels we're displaying. Allocates to a high-water-mark.
	V_Voxel voxels_;
	uint32_t voxel_count_;  // The number of voxels currently in #voxels_

	RenderStyle render_style_;
	float dim_;                       // Dimensions
	Ogre::Vector3 common_direction_;  // See Ogre::BillboardSet::setCommonDirection
	Ogre::Vector3 common_up_vector_;  // See Ogre::BillboardSet::setCommonUpVector

	Ogre::MaterialPtr point_material_;
	Ogre::MaterialPtr square_material_;
	Ogre::MaterialPtr flat_square_material_;
	Ogre::MaterialPtr sphere_material_;
	Ogre::MaterialPtr tile_material_;
	Ogre::MaterialPtr box_material_;
	Ogre::MaterialPtr current_material_;
	float alpha_;

	bool color_by_index_;

	V_VoxelsRenderable renderables_;

	bool current_mode_supports_geometry_shader_;
	Ogre::ColourValue pick_color_;

	static inline Ogre::String sm_Type = "Voxels";  // The "renderable type" used by Ogre
};
}  // namespace ufomap_ros::rviz_plugins

#endif  // UFOMAP_RVIZ_PLUGINS_RENDERABLE_H