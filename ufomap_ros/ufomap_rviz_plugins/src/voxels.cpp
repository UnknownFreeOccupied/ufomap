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

// UFO
#include <ufomap_rviz_plugins/voxels.h>

// QT
#include <qglobal.h>

// OGRE
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreVector3.h>

// ROS
#include <ros/assert.h>
#include <rviz/ogre_helpers/compatibility.h>

namespace ufomap_ros::rviz_plugins
{
//
// VoxelsRenderable
//

VoxelsRenderable::VoxelsRenderable(Voxels* parent, int num_voxels, bool use_tex_coords)
    : parent_(parent)
{
	// Initialize render operation
	mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
	mRenderOp.useIndexes = false;
	mRenderOp.vertexData = new Ogre::VertexData;
	mRenderOp.vertexData->vertexStart = 0;
	mRenderOp.vertexData->vertexCount = 0;

	Ogre::VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;
	size_t offset = 0;

	decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

	if (use_tex_coords) {
		decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 0);
		offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	}

	decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

	Ogre::HardwareVertexBufferSharedPtr vbuf =
	    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
	        mRenderOp.vertexData->vertexDeclaration->getVertexSize(0), num_voxels,
	        Ogre::HardwareBuffer::HBU_DYNAMIC);

	// Bind buffer
	mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
}

VoxelsRenderable::~VoxelsRenderable()
{
	delete mRenderOp.vertexData;
	delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr VoxelsRenderable::getBuffer()
{
	return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

Ogre::Real VoxelsRenderable::getBoundingRadius() const
{
	return Ogre::Math::Sqrt(
	    std::max(mBox.maximum().squaredLength(), mBox.minimum().squaredLength()));
}

Ogre::Real VoxelsRenderable::getSquaredViewDepth(Ogre::Camera const* cam) const
{
	Ogre::Vector3 min = mBox.minimum();
	Ogre::Vector3 max = mBox.maximum();
	Ogre::Vector3 mid = ((max - min) * 0.5) + min;
	Ogre::Vector3 dist = cam->getDerivedPosition() - mid;
	return dist.squaredLength();
}

void VoxelsRenderable::getWorldTransforms(Ogre::Matrix4* xform) const
{
	parent_->getWorldTransforms(xform);
}

Ogre::LightList const& VoxelsRenderable::getLights() const
{
	return parent_->queryLights();
}

//
// Voxels
//

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)

static float g_point_vertices[3] = {0.0f, 0.0f, 0.0f};

static float g_billboard_vertices[6 * 3] = {
    -0.5f, 0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.5f, 0.5f,  0.0f,
    0.5f,  0.5f, 0.0f, -0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f,
};

static float g_billboard_sphere_vertices[3 * 3] = {
    0.0f, 1.0f, 0.0f, -0.866025404f, -0.5f, 0.0f, 0.866025404f, -0.5f, 0.0f,
};

static float g_box_vertices[6 * 6 * 3] = {
    // clang-format off
    // front
    -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
    0.5f, -0.5f, -0.5f,

    // back
    -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f,

    // right
    0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5,

    // left
    -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5,

    // top
    -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5,

    // bottom
    -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5,
};  // clang-format on

Voxels::Voxels()
    : bounding_radius_(0.0f),
      voxel_count_(0),
      common_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z),
      common_up_vector_(Ogre::Vector3::UNIT_Y),
      color_by_index_(false),
      current_mode_supports_geometry_shader_(false)
{
	std::stringstream ss;
	static size_t count = 0;
	ss << "UFOMapMaterial" << count++;
	point_material_ =
	    Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/Point");
	square_material_ =
	    Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/Square");
	flat_square_material_ =
	    Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/FlatSquare");
	sphere_material_ =
	    Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/Sphere");
	// tile_material_ =
	//     Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/Tile");
	box_material_ =
	    Ogre::MaterialManager::getSingleton().getByName("ufomap_rviz_plugins/Box");

	point_material_ = Ogre::MaterialPtr(point_material_)->clone(ss.str() + "Point");
	square_material_ = Ogre::MaterialPtr(square_material_)->clone(ss.str() + "Square");
	flat_square_material_ =
	    Ogre::MaterialPtr(flat_square_material_)->clone(ss.str() + "FlatSquare");
	sphere_material_ = Ogre::MaterialPtr(sphere_material_)->clone(ss.str() + "Sphere");
	// tile_material_ = Ogre::MaterialPtr(tile_material_)->clone(ss.str() + "Tiles");
	box_material_ = Ogre::MaterialPtr(box_material_)->clone(ss.str() + "Box");

	point_material_->load();
	square_material_->load();
	flat_square_material_->load();
	sphere_material_->load();
	// tile_material_->load();
	box_material_->load();

	setAlpha(1.0f);
	setRenderStyle(RenderStyle::BOXES);
	setDimensions(0.01f);

	clear();
}

static void removeMaterial(Ogre::MaterialPtr& material)
{
	Ogre::ResourcePtr resource(material);
	Ogre::MaterialManager::getSingleton().remove(resource);
}

Voxels::~Voxels()
{
	clear();

	removeMaterial(point_material_);
	removeMaterial(square_material_);
	removeMaterial(flat_square_material_);
	removeMaterial(sphere_material_);
	// removeMaterial(tile_material_);
	removeMaterial(box_material_);
}

Ogre::AxisAlignedBox const& Voxels::getBoundingBox() const { return bounding_box_; }

float Voxels::getBoundingRadius() const { return bounding_radius_; }

void Voxels::getWorldTransforms(Ogre::Matrix4* xform) const
{
	*xform = _getParentNodeFullTransform();
}

void Voxels::clear()
{
	voxel_count_ = 0;
	bounding_box_.setNull();
	bounding_radius_ = 0.0f;

	if (getParentSceneNode()) {
		for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
		     ++it) {
			getParentSceneNode()->detachObject(it->get());
		}
		getParentSceneNode()->needUpdate();
	}

	renderables_.clear();
}

void Voxels::regenerateAll()
{
	if (0 == voxel_count_) {
		return;
	}

	V_Voxel voxels;
	voxels.swap(voxels_);
	uint32_t count = voxel_count_;

	clear();

	addVoxels(&voxels.front(), count);
}

void Voxels::setColorByIndex(bool set)
{
	color_by_index_ = set;
	regenerateAll();
}

void Voxels::setHighlightColor(float r, float g, float b)
{
	Ogre::Vector4 highlight(r, g, b, 0.0f);

	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		(*it)->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
	}
}

void Voxels::setRenderStyle(RenderStyle style)
{
	render_style_ = style;

	if (RenderStyle::POINTS == style) {
		current_material_ = Ogre::MaterialPtr(point_material_);
	} else if (RenderStyle::SQUARES == style) {
		current_material_ = Ogre::MaterialPtr(square_material_);
	} else if (RenderStyle::FLAT_SQUARES == style) {
		current_material_ = Ogre::MaterialPtr(flat_square_material_);
	} else if (RenderStyle::SPHERES == style) {
		current_material_ = Ogre::MaterialPtr(sphere_material_);
	} else if (RenderStyle::TILES == style) {
		current_material_ = Ogre::MaterialPtr(tile_material_);
	} else if (RenderStyle::BOXES == style) {
		current_material_ = Ogre::MaterialPtr(box_material_);
	}

	current_material_->load();

	// ROS_INFO("Best technique [%s] [gp=%s]",
	// current_material_->getBestTechnique()->getName().c_str(),
	// current_material_->getBestTechnique()->getPass(0)->getGeometryProgramName().c_str());

	bool geom_support_changed = false;
	Ogre::Technique* best = current_material_->getBestTechnique();
	if (best) {
		if ("gp" == current_material_->getBestTechnique()->getName()) {
			if (!current_mode_supports_geometry_shader_) {
				geom_support_changed = true;
			}
			current_mode_supports_geometry_shader_ = true;
			// ROS_INFO("Using geometry shader");
		} else {
			if (current_mode_supports_geometry_shader_) {
				geom_support_changed = true;
			}
			current_mode_supports_geometry_shader_ = false;
		}
	} else {
		geom_support_changed = true;
		current_mode_supports_geometry_shader_ = false;

		ROS_ERROR("No techniques available for material [%s]",
		          current_material_->getName().c_str());
	}

	if (geom_support_changed) {
		renderables_.clear();
	}

	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		rviz::setMaterial(**it, current_material_);
	}

	// FIXME: Not needed as we use geometry shaders
	// regenerateAll();
}

void Voxels::setDimensions(float dim)
{
	dim_ = dim;

	Ogre::Vector4 size(dim_, dim_, dim_, 0.0f);

	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		(*it)->setCustomParameter(SIZE_PARAMETER, size);
	}
}

void Voxels::setCommonDirection(const Ogre::Vector3& vec)
{
	common_direction_ = vec;

	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		(*it)->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(vec));
	}
}

void Voxels::setCommonUpVector(const Ogre::Vector3& vec)
{
	common_up_vector_ = vec;

	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		(*it)->setCustomParameter(UP_PARAMETER, Ogre::Vector4(vec));
	}
}

void setAlphaBlending(const Ogre::MaterialPtr& mat)
{
	if (mat->getBestTechnique()) {
		mat->getBestTechnique()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
		mat->getBestTechnique()->setDepthWriteEnabled(false);
	}
}

void setReplace(const Ogre::MaterialPtr& mat)
{
	if (mat->getBestTechnique()) {
		mat->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
		mat->getBestTechnique()->setDepthWriteEnabled(true);
	}
}

void Voxels::setAlpha(float alpha, bool per_voxel_alpha)
{
	alpha_ = alpha;

	if (alpha < 0.9998 || per_voxel_alpha) {
		setAlphaBlending(point_material_);
		setAlphaBlending(square_material_);
		setAlphaBlending(flat_square_material_);
		setAlphaBlending(sphere_material_);
		// setAlphaBlending(tile_material_);
		setAlphaBlending(box_material_);
	} else {
		setReplace(point_material_);
		setReplace(square_material_);
		setReplace(flat_square_material_);
		setReplace(sphere_material_);
		// setReplace(tile_material_);
		setReplace(box_material_);
	}

	Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		(*it)->setCustomParameter(ALPHA_PARAMETER, alpha4);
	}
}

void Voxels::addVoxels(Voxel* voxels, uint32_t num_voxels)
{
	if (num_voxels == 0) {
		return;
	}

	Ogre::Root* root = Ogre::Root::getSingletonPtr();

	if (voxels_.size() < voxel_count_ + num_voxels) {
		voxels_.resize(voxel_count_ + num_voxels);
	}

	Voxel* begin = &voxels_.front() + voxel_count_;
#if defined(__GNUC__) && (__GNUC__ >= 8)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
	memcpy(begin, voxels, sizeof(Voxel) * num_voxels);
#if defined(__GNUC__) && (__GNUC__ >= 8)
#pragma GCC diagnostic pop
#endif

	uint32_t vpv = getVerticesPerVoxel();

	Ogre::RenderOperation::OperationType op_type;
	if (current_mode_supports_geometry_shader_) {
		op_type = Ogre::RenderOperation::OT_POINT_LIST;
	} else {
		if (RenderStyle::POINTS == render_style_) {
			op_type = Ogre::RenderOperation::OT_POINT_LIST;
		} else {
			op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
		}
	}

	float* vertices = nullptr;
	if (current_mode_supports_geometry_shader_) {
		vertices = g_point_vertices;
	} else {
		if (RenderStyle::POINTS == render_style_) {
			vertices = g_point_vertices;
		} else if (RenderStyle::SQUARES == render_style_) {
			vertices = g_billboard_vertices;
		} else if (RenderStyle::FLAT_SQUARES == render_style_) {
			vertices = g_billboard_vertices;
		} else if (RenderStyle::SPHERES == render_style_) {
			vertices = g_billboard_sphere_vertices;
		} else if (RenderStyle::TILES == render_style_) {
			vertices = g_billboard_vertices;
		} else if (RenderStyle::BOXES == render_style_) {
			vertices = g_box_vertices;
		}
	}

	VoxelsRenderablePtr rend;
	Ogre::HardwareVertexBufferSharedPtr vbuf;
	void* vdata = nullptr;
	Ogre::RenderOperation* op = nullptr;
	float* fptr = nullptr;

	Ogre::AxisAlignedBox aabb;
	aabb.setNull();
	uint32_t current_vertex_count = 0;
	bounding_radius_ = 0.0f;
	uint32_t vertex_size = 0;
	uint32_t buffer_size = 0;
	for (uint32_t current_voxel = 0; current_voxel != num_voxels; ++current_voxel) {
		// if we didn't create a renderable yet,
		// or we've reached the vertex limit for the current renderable,
		// create a new one.
		while (!rend || current_vertex_count >= buffer_size) {
			if (rend) {
				ROS_ASSERT(current_vertex_count == buffer_size);

				op->vertexData->vertexCount =
				    rend->getBuffer()->getNumVertices() - op->vertexData->vertexStart;
				ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <=
				           rend->getBuffer()->getNumVertices());
				vbuf->unlock();
				rend->setBoundingBox(aabb);
				bounding_box_.merge(aabb);
			}

			buffer_size =
			    std::min<int>(VERTEX_BUFFER_CAPACITY, (num_voxels - current_voxel) * vpv);

			rend = createRenderable(buffer_size);
			vbuf = rend->getBuffer();
			vdata = vbuf->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE);

			op = rend->getRenderOperation();
			op->operationType = op_type;
			current_vertex_count = 0;

			vertex_size = op->vertexData->vertexDeclaration->getVertexSize(0);
			fptr = (float*)((uint8_t*)vdata);  // TODO: Use C++, reinterpret cast?

			aabb.setNull();
		}

		const Voxel& p = voxels[current_voxel];

		uint32_t color;
		if (color_by_index_) {
			// convert to ColourValue, so we can then convert to the rendersystem-specific color
			// type
			color = (current_voxel + voxel_count_ + 1);
			Ogre::ColourValue c;
			c.a = 1.0f;
			c.r = ((color >> 16) & 0xff) / 255.0f;
			c.g = ((color >> 8) & 0xff) / 255.0f;
			c.b = (color & 0xff) / 255.0f;
			root->convertColourValue(c, &color);
		} else {
			root->convertColourValue(p.color, &color);
		}

		aabb.merge(p.position);
		bounding_radius_ = std::max(bounding_radius_, p.position.squaredLength());

		float x = p.position.x;
		float y = p.position.y;
		float z = p.position.z;

		for (uint32_t j = 0; j != vpv; ++j, ++current_vertex_count) {
			*fptr++ = x;
			*fptr++ = y;
			*fptr++ = z;

			if (!current_mode_supports_geometry_shader_) {
				*fptr++ = vertices[(j * 3)];
				*fptr++ = vertices[(j * 3) + 1];
				*fptr++ = vertices[(j * 3) + 2];
			}

			uint32_t* iptr = (uint32_t*)fptr;
			*iptr = color;
			++fptr;

			ROS_ASSERT((uint8_t*)fptr <=
			           (uint8_t*)vdata + rend->getBuffer()->getNumVertices() * vertex_size);
			Q_UNUSED(vertex_size);
		}
	}

	op->vertexData->vertexCount = current_vertex_count - op->vertexData->vertexStart;
	rend->setBoundingBox(aabb);
	bounding_box_.merge(aabb);
	ROS_ASSERT(op->vertexData->vertexCount + op->vertexData->vertexStart <=
	           rend->getBuffer()->getNumVertices());

	vbuf->unlock();

	voxel_count_ += num_voxels;

	shrinkRenderables();

	if (getParentSceneNode()) {
		getParentSceneNode()->needUpdate();
	}
}

void Voxels::popVoxels(uint32_t num_voxels)
{
	uint32_t vpv = getVerticesPerVoxel();

	ROS_ASSERT(num_voxels <= voxel_count_);
	voxels_.erase(voxels_.begin(), voxels_.begin() + num_voxels);

	voxel_count_ -= num_voxels;

	// Now clear out popped voxels
	uint32_t popped_count = 0;
	while (popped_count < num_voxels * vpv) {
		VoxelsRenderablePtr rend = renderables_.front();
		Ogre::RenderOperation* op;
		op = rend->getRenderOperation();

		uint32_t popped =
		    std::min((size_t)(num_voxels * vpv - popped_count), op->vertexData->vertexCount);
		op->vertexData->vertexStart += popped;
		op->vertexData->vertexCount -= popped;

		popped_count += popped;

		if (op->vertexData->vertexCount == 0) {
			renderables_.erase(renderables_.begin(), renderables_.begin() + 1);

			op->vertexData->vertexStart = 0;
			renderables_.push_back(rend);
		}
	}
	ROS_ASSERT(popped_count == num_voxels * vpv);

	// reset bounds
	bounding_box_.setNull();
	bounding_radius_ = 0.0f;
	for (uint32_t i = 0; i < voxel_count_; ++i) {
		Voxel& p = voxels_[i];
		bounding_box_.merge(p.position);
		bounding_radius_ = std::max(bounding_radius_, p.position.squaredLength());
	}

	shrinkRenderables();

	if (getParentSceneNode()) {
		getParentSceneNode()->needUpdate();
	}
}

void Voxels::shrinkRenderables()
{
	while (!renderables_.empty()) {
		VoxelsRenderablePtr rend = renderables_.back();
		Ogre::RenderOperation* op;
		op = rend->getRenderOperation();
		if (0 == op->vertexData->vertexCount) {
			renderables_.pop_back();
		} else {
			break;
		}
	}
}

void Voxels::_updateRenderQueue(Ogre::RenderQueue* queue)
{
	for (auto it = std::begin(renderables_), end = std::end(renderables_); it != end;
	     ++it) {
		queue->addRenderable((*it).get());
	}
}

void Voxels::_notifyAttached(Ogre::Node* parent, bool isTagVoxel)
{
	MovableObject::_notifyAttached(parent, isTagVoxel);
}

uint32_t Voxels::getVerticesPerVoxel()
{
	if (current_mode_supports_geometry_shader_) {
		return 1;
	}

	switch (render_style_) {
		case RenderStyle::POINTS:
			return 1;
		case RenderStyle::SQUARES:
		case RenderStyle::FLAT_SQUARES:
		case RenderStyle::TILES:
			return 6;
		case RenderStyle::SPHERES:
			return 3;
		case RenderStyle::BOXES:
			return 36;
		default:
			return 1;
	}
}

void Voxels::setPickColor(Ogre::ColourValue const& color)
{
	pick_color_ = color;
	Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

	V_VoxelsRenderable::iterator it = renderables_.begin();
	V_VoxelsRenderable::iterator end = renderables_.end();
	for (; it != end; ++it) {
		(*it)->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
	}
	getUserObjectBindings().setUserAny(
	    "pick_handle", Ogre::Any((int(color.r * 255) << 16) | (int(color.g * 255) << 8) |
	                             int(color.b * 255)));
}

VoxelsRenderablePtr Voxels::createRenderable(int num_voxels)
{
	VoxelsRenderablePtr rend(
	    new VoxelsRenderable(this, num_voxels, !current_mode_supports_geometry_shader_));
	rviz::setMaterial(*rend, current_material_);
	Ogre::Vector4 size(dim_, dim_, dim_, 0.0f);
	Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
	Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
	Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);
	rend->setCustomParameter(SIZE_PARAMETER, size);
	rend->setCustomParameter(ALPHA_PARAMETER, alpha);
	rend->setCustomParameter(HIGHLIGHT_PARAMETER, highlight);
	rend->setCustomParameter(PICK_COLOR_PARAMETER, pick_col);
	rend->setCustomParameter(NORMAL_PARAMETER, Ogre::Vector4(common_direction_));
	rend->setCustomParameter(UP_PARAMETER, Ogre::Vector4(common_up_vector_));
	if (getParentSceneNode()) {
		getParentSceneNode()->attachObject(rend.get());
	}
	renderables_.push_back(rend);

	return rend;
}

void Voxels::visitRenderables(Ogre::Renderable::Visitor* /*visitor*/,
                              bool /*debugRenderables*/)
{
}

}  // namespace ufomap_ros::rviz_plugins