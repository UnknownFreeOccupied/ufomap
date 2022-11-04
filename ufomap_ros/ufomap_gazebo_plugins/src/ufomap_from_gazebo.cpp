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
#include <ufo/geometry/triangle.h>

// UFO ROS
#include <ufomap_gazebo_plugins/ufomap_from_gazebo.h>
#include <ufomap_msgs/UFOMap.h>
#include <ufomap_msgs/conversions.h>

// Gazebo
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

// Ogre
#include <OGRE/OgreMaterialManager.h>

namespace ufomap_gazebo_plugins
{
template <class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param,
                 const T& default_value, const bool& verbose = false)
{
	if (sdf->HasElement(name)) {
		param = sdf->GetElement(name)->Get<T>();
		return true;
	} else {
		param = default_value;
		if (verbose)
			gzerr << "[ufomap_gazebo_plugins] Please specify a value for parameter \"" << name
			      << "\".\n";
	}
	return false;
}

UFOMapFromGazebo::UFOMapFromGazebo() : nh_(""), nh_priv_("~") {}

UFOMapFromGazebo::~UFOMapFromGazebo() {}

void UFOMapFromGazebo::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	gzdbg << __FUNCTION__ << "() called.\n";

	world_ = _parent;

	std::string service_name = "get_map";
	std::string pub_topic_name = "map";
	bool pub_topic_latched = true;
	getSdfParam<std::string>(_sdf, "ServiceName", service_name, service_name);
	getSdfParam<std::string>(_sdf, "PubTopicName", pub_topic_name, pub_topic_name);
	getSdfParam<bool>(_sdf, "PubTopicLatched", pub_topic_latched, pub_topic_latched);

	gzlog << "Advertising service: " << service_name << '\n';
	get_map_srv_ =
	    nh_priv_.advertiseService(service_name, &UFOMapFromGazebo::getMapCallback, this);
	pub_ = nh_priv_.advertise<ufomap_msgs::UFOMap>(pub_topic_name, 1,
	                                                      pub_topic_latched);
}

bool UFOMapFromGazebo::getMapCallback(GetMap::Request& req, GetMap::Response& res)
{
	ufo::map::OccupancyMapColor map(req.resolution, req.depth_levels);

	ufo::geometry::AABB bv = ufomap_msgs::msgToUfo(req.aabb);

	createMap2(map, bv, req.as_unknown, req.depth);

	std::stringstream data(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	data.exceptions(std::stringstream::failbit | std::stringstream::badbit);
	data.imbue(std::locale());

	map.write(data, req.depth, req.compress, 1, req.compression_level);

	res.map.data.reserve(data.tellp());
	std::copy(std::istreambuf_iterator<char>(data), std::istreambuf_iterator<char>(),
	          std::back_inserter(res.map.data));
	res.success = true;

	if ("" != req.filename) {
		std::ofstream file;
		file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios_base::out | std::ios_base::binary);
		file << data.rdbuf();
	}

	if (req.publish) {
		ufomap_msgs::UFOMap msg;
		msg.header.frame_id = req.publish_frame;
		msg.header.stamp = ros::Time::now();
		msg.map = res.map;
		pub_.publish(msg);
	}

	return true;
}

void UFOMapFromGazebo::createMap2(ufo::map::OccupancyMapColor& map,
                                  ufo::geometry::AABB const& bv, bool as_unknown,
                                  unsigned int depth)
{
	float size = map.getNodeSize(depth);
	float half_size = map.nodeHalfSize(depth);

	gazebo::common::MeshManager* mesh_manager = gazebo::common::MeshManager::Instance();

	if (!mesh_manager) {
		return;
	}

	gazebo::physics::Collision_V collisions;

	for (auto const& model : world_->Models()) {
		for (auto const& link : model->GetLinks()) {
			auto c = link->GetCollisions();
			collisions.reserve(collisions.size() + c.size());
			collisions.insert(std::end(collisions), std::begin(c), std::end(c));
		}
	}

	std::for_each(
	    std::begin(collisions), std::end(collisions),
	    [&](gazebo::physics::CollisionPtr const& collision) {
		    gazebo::msgs::Geometry geometry_msg;
		    collision->GetShape()->FillMsg(geometry_msg);

		    if (!geometry_msg.has_type()) {
			    gzwarn << "Geometry type not available\n";
			    return;
		    }

		    std::string geometry_type_str =
		        gazebo::msgs::ConvertGeometryType(geometry_msg.type());

		    if ("box" != geometry_type_str && "cylinder" != geometry_type_str &&
		        "sphere" != geometry_type_str && "plane" != geometry_type_str &&
		        "mesh" != geometry_type_str) {
			    gzwarn << "Not yet able to precess shapes of type: " << geometry_type_str
			           << '\n';
			    return;
		    }

		    if (!mesh_manager) {
			    gzwarn << "Could not get pointer to MeshManager\n";
			    return;
		    }

		    gazebo::common::Mesh const* mesh_ptr;
		    std::string mesh_name;
		    if ("mesh" == geometry_type_str) {
			    std::string mesh_base_name = geometry_msg.mesh().filename();
			    gzlog << "Attempting to load mesh " << mesh_base_name << '\n';

			    std::string const prefix = "file://";
			    size_t idx_prefix = mesh_base_name.find(prefix);
			    if (mesh_base_name.size() > idx_prefix &&
			        mesh_base_name.size() > prefix.size()) {
				    mesh_base_name.erase(std::begin(mesh_base_name),
				                         std::next(std::begin(mesh_base_name), prefix.size()));
			    }
			    mesh_ptr = mesh_manager->Load(mesh_base_name);
			    if (mesh_ptr) {
				    gzlog << "Loading file '" << mesh_name << "' successful\n";
			    } else {
				    mesh_name = geometry_msg.mesh().filename();
				    gzwarn << "Loading file '" << mesh_name << "' failed\n";
			    }
		    } else {
			    mesh_name = "unit_" + geometry_type_str;
			    mesh_ptr = mesh_manager->GetMesh(mesh_name);
		    }

		    if (!mesh_ptr) {
			    gzwarn << "Could not get pointer to mesh '" << mesh_name << "'\n";
			    return;
		    }

		    // gazebo::common::Image image(
		    //     "/home/dduberg/.gazebo/models/house_1/materials/textures/"
		    //     "House_1_Diffuse.png");

		    // Ogre::MaterialPtr material =
		    //     Ogre::MaterialManager::getSingleton().getByName("House_1/Diffuse");
		    // for (auto const& p : material->getParameters()) {
		    //   gzerr << p.name << '\n';
		    // }

		    for (size_t submesh_id = 0; mesh_ptr->GetSubMeshCount() != submesh_id;
		         ++submesh_id) {
			    gazebo::common::SubMesh submesh(mesh_ptr->GetSubMesh(submesh_id));

			    if (gazebo::common::SubMesh::TRIANGLES != submesh.GetPrimitiveType()) {
				    gzwarn << "Encountered a mesh with a type that is currently not supported\n";
				    return;
			    }

			    ignition::math::Vector3d geometry_size;
			    if ("box" == geometry_type_str) {
				    geometry_size = gazebo::msgs::ConvertIgn(geometry_msg.box().size());
			    } else if ("sphere" == geometry_type_str) {
				    double diameter = 2.0 * geometry_msg.sphere().radius();
				    geometry_size.Set(diameter, diameter, diameter);
			    } else if ("cylinder" == geometry_type_str) {
				    double diameter = 2.0 * geometry_msg.cylinder().radius();
				    double length = geometry_msg.cylinder().length();
				    geometry_size.Set(diameter, diameter, length);
			    } else if ("plane" == geometry_type_str) {
				    gazebo::msgs::Vector2d dimensions = geometry_msg.plane().size();
				    geometry_size.Set(dimensions.x(), dimensions.y(), 1.0);
			    } else if ("mesh" == geometry_type_str) {
				    if (collision->GetShape()->GetSDF()->HasElement("scale")) {
					    collision->GetShape()->GetSDF()->GetElement("scale")->GetValue()->Get(
					        geometry_size);
				    } else {
					    geometry_size = collision->GetShape()->Scale();
				    }
			    } else {
				    gzwarn << "Could not get geometry size of '" << geometry_type_str << "'\n";
				    return;
			    }

			    ignition::math::Pose3d const transform = collision->WorldPose();

			    for (size_t vertex_idx = 0; submesh.GetVertexCount() != vertex_idx;
			         ++vertex_idx) {
				    ignition::math::Vector3d new_vertex = submesh.Vertex(vertex_idx);

				    new_vertex *= geometry_size;
				    new_vertex = transform.Rot() * new_vertex;
				    new_vertex += transform.Pos();

				    submesh.SetVertex(vertex_idx, new_vertex);
			    }

			    size_t num_faces = submesh.GetIndexCount() / 3;
			    gzlog << "Integrating " << num_faces << " faces\n";
			    for (size_t triangle_idx = 0; num_faces != triangle_idx; ++triangle_idx) {
				    size_t const index_a = submesh.GetIndex(triangle_idx * 3);
				    size_t const index_b = submesh.GetIndex(triangle_idx * 3 + 1);
				    size_t const index_c = submesh.GetIndex(triangle_idx * 3 + 2);

				    ufo::math::Vector3 a(submesh.Vertex(index_a).X(), submesh.Vertex(index_a).Y(),
				                         submesh.Vertex(index_a).Z());
				    ufo::math::Vector3 b(submesh.Vertex(index_b).X(), submesh.Vertex(index_b).Y(),
				                         submesh.Vertex(index_b).Z());
				    ufo::math::Vector3 c(submesh.Vertex(index_c).X(), submesh.Vertex(index_c).Y(),
				                         submesh.Vertex(index_c).Z());

				    ufo::math::Vector3 min(std::min({a.x(), b.x(), c.x()}),
				                           std::min({a.y(), b.y(), c.y()}),
				                           std::min({a.z(), b.z(), c.z()}));
				    ufo::math::Vector3 max(std::max({a.x(), b.x(), c.x()}),
				                           std::max({a.y(), b.y(), c.y()}),
				                           std::max({a.z(), b.z(), c.z()}));

				    min = map.toCoord(map.toKey(min, depth), depth);
				    max = map.toCoord(map.toKey(max, depth), depth);
				    max += half_size;

				    ufo::geometry::Triangle triangle(a, b, c);

				    // gzerr << submesh.TexCoord(index_a) << '\n';
				    // gzerr << submesh.TexCoord(index_b) << '\n';
				    // gzerr << submesh.TexCoord(index_c) << '\n';

				    for (float x = min.x(); x < max.x(); x += size) {
					    for (float y = min.y(); y < max.y(); y += size) {
						    for (float z = min.z(); z < max.z(); z += size) {
							    ufo::geometry::AAEBB box(ufo::geometry::Point(x, y, z), half_size);
							    if (ufo::geometry::intersects(box, triangle)) {
								    map.setOccupancyLogit(
								        x, y, z, map.getOccupancyClampingThresMaxLogit(), false, depth);

								    uint32_t c;
								    if (0 < mesh_ptr->GetMaterialCount()) {
									    c = mesh_ptr->GetMaterial(submesh.GetMaterialIndex())
									            ->Ambient()
									            .AsARGB();
									    map.setColor(
									        x, y, z,
									        ufo::map::Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF),
									        false, depth);
								    }
								    // auto sadasd = submesh.TexCoord(index_a);
								    // auto asd = image.Pixel(
								    //     (image.GetWidth() - 1) - (sadasd.X() * (image.GetWidth() - 1)),
								    //     (image.GetHeight() - 1) - (sadasd.Y() * (image.GetHeight() -
								    //     1)));
								    // c = asd.AsARGB();
								    // map.setColor(
								    //     x, y, z,
								    //     ufo::map::Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF),
								    //     false, depth);
							    }
						    }
					    }
				    }
			    }
		    }
	    });

	map.updateModifiedNodes();

	printf("\rMap generation complete!\n");
}

void UFOMapFromGazebo::createMap(ufo::map::OccupancyMapColor& map,
                                 ufo::geometry::AABB const& bv, bool as_unknown,
                                 unsigned int depth)
{
	std::vector<ignition::math::Vector3d> points;

	if (!as_unknown) {
		// Set everything inside bbx to free
		// map_.setValueVolume(bv, map_.getClampingThresMin(), depth);
	}

	// Other

	printf("Rasterizing world and checking collisions\n");
	gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
	engine->InitForThread();
	gazebo::physics::RayShapePtr ray =
	    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
	        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

	float step = map.getNodeSize(depth);
	float res = step;
	float half_step = map.nodeHalfSize(depth);
	ufo::math::Vector3 min = bv.min() - half_step;
	ufo::math::Vector3 max = bv.max() + half_step;

	int num_samples = 1;  // FIXME: Should this be a parameter?
	step /= num_samples;

	auto dim_steps = (max - min) / step;
	float total_steps =
	    std::abs(dim_steps.y()) + std::abs(dim_steps.x()) + std::abs(dim_steps.x());
	float cur_step = 0.0;

	// Do it along x
	for (float y = min.y(); y < max.y(); y += step) {
		cur_step += 1.0;
		printf("\rProgress %3.0f%%", 100.0 * (cur_step / total_steps));
		for (float z = min.z(); z < max.z(); z += step) {
			ignition::math::Vector3d from(min.x(), y, z);
			ignition::math::Vector3d to(max.x(), y, z);
			getHits(points, ray, from, to, res);
		}
	}
	// Do it along y
	for (float x = min.x(); x < max.x(); x += step) {
		cur_step += 1.0;
		printf("\rProgress %3.0f%%", 100.0 * (cur_step / total_steps));
		for (float z = min.z(); z < max.z(); z += step) {
			ignition::math::Vector3d from(x, min.y(), z);
			ignition::math::Vector3d to(x, max.y(), z);
			getHits(points, ray, from, to, res);
		}
	}
	// Do it along z
	for (float x = min.x(); x < max.x(); x += step) {
		cur_step += 1.0;
		printf("\rProgress %3.0f%%", 100.0 * (cur_step / total_steps));
		for (float y = min.y(); y < max.y(); y += step) {
			ignition::math::Vector3d from(x, y, min.z());
			ignition::math::Vector3d to(x, y, max.z());
			getHits(points, ray, from, to, res);
		}
	}

	std::for_each(std::begin(points), std::end(points), [&](auto const& point) {
		map.setOccupancyLogit(point.X(), point.Y(), point.Z(),
		                      map.getOccupancyClampingThresMaxLogit(), false, depth);
	});

	// std::vector<float> x_indices((max.x() - min.x()) / step);
	// std::generate(begin(x_indices), end(x_indices), [n = min.x(), step]() mutable {
	// 	auto t = n;
	// 	n += step;
	// 	return t;
	// });
	// std::for_each(begin(x_indices), end(x_indices), [&](auto x) {
	// 	printf("\rProgress %3.0f%%", 100 * (x - min.x()) / (max.x() - min.x()));
	// 	for (float y = min.y(); y < max.y(); y += step) {
	// 		for (float z = min.z(); z < max.z(); z += step) {
	// 			ignition::math::Vector3d point(x, y, z);
	// 			if (checkIfInterest(point, ray, step)) {
	// 				map_.setOccupancyLogit(x, y, z, map_.getOccupancyClampingThresMaxLogit(),
	// false, 				                       depth);
	// 			}
	// 		}
	// 	}
	// });

	map.updateModifiedNodes();

	printf("\rMap generation complete!\n");
}

void UFOMapFromGazebo::getHits(std::vector<ignition::math::Vector3d>& hits,
                               gazebo::physics::RayShapePtr ray,
                               ignition::math::Vector3d from,
                               ignition::math::Vector3d const& to, double resolution)
{
	size_t idx = from.X() != to.X() ? 0 : from.Y() != to.Y() ? 1 : 2;

	double dist;
	std::string entity_name;

	while (from[idx] < to[idx]) {
		ray->SetPoints(from, to);
		ray->GetIntersection(dist, entity_name);

		if (dist > from.Distance(to)) {
			return;
		}

		from[idx] += dist;
		hits.push_back(from);
		from[idx] += resolution;  // FIXME: What should this be?
	}
}

bool UFOMapFromGazebo::checkIfInterest(ignition::math::Vector3d const& central_point,
                                       gazebo::physics::RayShapePtr ray,
                                       double resolution)
{
	ignition::math::Vector3d start_point = central_point;
	ignition::math::Vector3d end_point = central_point;

	double dist;
	std::string entity_name;

	start_point.X() += resolution / 2;
	end_point.X() -= resolution / 2;
	ray->SetPoints(start_point, end_point);
	ray->GetIntersection(dist, entity_name);

	if (dist <= resolution) return true;

	start_point = central_point;
	end_point = central_point;
	start_point.Y() += resolution / 2;
	end_point.Y() -= resolution / 2;
	ray->SetPoints(start_point, end_point);
	ray->GetIntersection(dist, entity_name);

	if (dist <= resolution) return true;

	start_point = central_point;
	end_point = central_point;
	start_point.Z() += resolution / 2;
	end_point.Z() -= resolution / 2;
	ray->SetPoints(start_point, end_point);
	ray->GetIntersection(dist, entity_name);

	if (dist <= resolution) return true;

	return false;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(UFOMapFromGazebo)

}  // namespace ufomap_gazebo_plugins
