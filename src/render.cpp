#ifdef UFO_TBB
// #undef UFO_TBB
#endif
#ifdef UFO_OMP
// #undef UFO_OMP
#endif

// UFO
#include <ufo/map/color/map.hpp>
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/vision/image.hpp>
// #include <ufo/util/timing.hpp>
#include <ufo/map/distance/map.hpp>
#include <ufo/map/integrator/integrator.hpp>
#include <ufo/pcl/cloud.hpp>
#include <ufo/pcl/ply.hpp>
#include <ufo/pcl/ufo.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/utility/index_iterator.hpp>
#include <ufo/vision/camera.hpp>

// STL
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv2/core.hpp>
// #include <opencv2/dnn_superres.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// PCL
// #include <pcl/io/ply_io.h>
// #include <pcl/kdtree/kdtree_flann.h>

// TOML
#include "toml.hpp"

using namespace ufo;

std::atomic_bool map_changed = false;

struct Time {
	Time(std::string const& tag) : tag_(tag) {}

	template <class T>
	void add(T const& start)
	{
		double dur = std::chrono::duration<double, std::milli>(
		                 std::chrono::high_resolution_clock::now() - start)
		                 .count();

		total_ += dur;
		last_ = dur;
		min_  = std::min(min_, dur);
		max_  = std::max(max_, dur);
		++num_;
	}

	[[nodiscard]] double total() const { return total_; }

	[[nodiscard]] double mean() const { return num_ ? total_ / num_ : 0.0; }

	[[nodiscard]] double last() const { return last_; }

	[[nodiscard]] double min() const { return num_ ? min_ : 0.0; }

	[[nodiscard]] double max() const { return max_; }

	[[nodiscard]] std::size_t count() const { return num_; }

	[[nodiscard]] std::tuple<std::string, double, double, double, double, double,
	                         std::size_t>
	data() const
	{
		return {tag_, last(), mean(), total(), min(), max(), count()};
	}

 private:
	std::string tag_;
	double      total_{};
	double      last_{};
	double      min_ = std::numeric_limits<double>::max();
	double      max_{};
	std::size_t num_{};
};

struct Timings {
	Time reading{"Reading"};
	Time insert{"Insert"};
	Time propagate{"Propagate"};
	Time render{"Render"};
	Time upscale{"Upscale"};

	void print()
	{
		std::array data{reading.data(), insert.data(), propagate.data(), render.data(),
		                upscale.data()};

		// clang-format off
		std::cout << '\n';
		std::cout << "UFO Timings [ms]\n";

		std::cout << "+------------+------------+------------+-------------+-------------+-------------+---------------+\n";
  	std::cout << "|    What    |    Last    |    Mean    |    Total    |     Min     |     Max     |     Count     |\n";
  	std::cout << "+------------+------------+------------+-------------+-------------+-------------+---------------+\n";

		for (auto const& [what, last, mean, total, min, max, count] : data) {
			std::cout << "| " << std::left << std::setw(10) << what << " " << std::right
								<< "| " << std::setw(10) << std::fixed << std::setprecision(4) << last << " "
								<< "| " << std::setw(10) << std::fixed << std::setprecision(4) << mean << " "
								<< "| " << std::setw(11) << std::fixed << std::setprecision(4) << total << " "
								<< "| " << std::setw(11) << std::fixed << std::setprecision(4) << min << " "
								<< "| " << std::setw(11) << std::fixed << std::setprecision(4) << max << " "
								<< "| " << std::setw(13) << std::fixed << std::setprecision(4) << count << " |\n";
  	}

  	std::cout << "+------------+------------+------------+-------------+-------------+-------------+---------------+\n";
		// clang-format on
	}
};

Transform3f deserializeTF(std::filesystem::path const& file)
{
	Transform3f transform;

	auto config = toml::parse_file(file.string());

	auto const& rotation = *config["rotation"].as_table();
	Quatf       q;
	q.w                = rotation["w"].value_or(0.0f);
	q.x                = rotation["x"].value_or(0.0f);
	q.y                = rotation["y"].value_or(0.0f);
	q.z                = rotation["z"].value_or(0.0f);
	transform.rotation = static_cast<Mat3x3f>(q);

	auto const& translation = *config["translation"].as_table();
	transform.translation.x = translation["x"].value_or(0.0f);
	transform.translation.y = translation["y"].value_or(0.0f);
	transform.translation.z = translation["z"].value_or(0.0f);

	return transform;
}

template <class Map>
void load(Map& map, Timings& timings, std::filesystem::path const& path,
          std::size_t max_number = std::numeric_limits<std::size_t>::max(),
          std::size_t step       = 1)
{
	Integrator integrator;
	integrator.sample_method   = DownSamplingMethod::FIRST;
	integrator.occupancy_hit   = 0.7f;
	integrator.occupancy_miss  = 0.35f;
	integrator.miss_depth      = 0;
	integrator.max_distance    = 10.0f;
	// integrator.counted         = true;
	// integrator.distance_offset = 0.2f;

	std::filesystem::path resource_dir = RESOURCE_DIR;

	std::vector<std::filesystem::path> files;
	for (auto const& e : std::filesystem::directory_iterator(resource_dir / path)) {
		auto path = e.path();
		if (e.is_regular_file() && ".ufo" == path.extension()) {
			files.push_back(path);
		}
	}
	std::sort(files.begin(), files.end());

	max_number = std::min(files.size(), max_number);
	for (std::size_t i{}; max_number > i; i += step) {
		auto const reading = std::chrono::high_resolution_clock::now();

		auto file = files[i];

		Cloud<Vec3f, Color> cloud;
		readCloudUFO(file, cloud);
		auto stem = file.stem().string();
		stem      = stem.substr(0, stem.find('_'));
		file.replace_filename(stem + "_pose.toml");
		auto tf = deserializeTF(file);
		transformInPlace(execution::par, tf, cloud);

		timings.reading.add(reading);

		auto const insert = std::chrono::high_resolution_clock::now();

		std::cout << tf.translation << std::endl;

		integrator.insertRays(execution::par, map, cloud, tf.translation, false);
		// integrator.insertPoints(execution::par, map, cloud, false);

		timings.insert.add(insert);

		auto const propagate = std::chrono::high_resolution_clock::now();

		map.propagateModified(execution::par);

		timings.propagate.add(propagate);

		map_changed = true;

		timings.print();
	}
}

template <class Map>
std::optional<std::future<void>> kitti(
    Map& map, Timings& timings,
    std::size_t max_number = std::numeric_limits<std::size_t>::max(),
    std::size_t step       = 1)
{
	return std::async(std::launch::async,
	                  [&]() { load<Map>(map, timings, "kitti/ufo", max_number, step); });
}

template <class Map>
std::optional<std::future<void>> euroc(
    Map& map, Timings& timings,
    std::size_t max_number = std::numeric_limits<std::size_t>::max(),
    std::size_t step       = 1)
{
	return std::async(std::launch::async,
	                  [&]() { load<Map>(map, timings, "euroc/ufo", max_number, step); });
}

int main(int argc, char* argv[])
{
	Timings timings;

	Map3D<OccupancyMap, ColorMap> map(0.05, 19);
	map.occupancySetThres(0.8f, 0.5f);
	map.occupancySetClampingThres(0.97f);

	// auto f = kitti(map, timings);
	auto f = euroc(map, timings);

	float yaw   = M_PI_4;
	float pitch = 0;
	float roll  = 0.0f;  // M_PI_2;

	Camera camera;
	camera.vertical_fov = radians(60.0f);
	camera.near_clip    = 0.01;
	camera.far_clip     = 100.0;

	std::size_t rows = 720;   // 360;  // 720;   // 1080;
	std::size_t cols = 1280;  // 640;  // 1280;  // 1920;

	std::size_t output_rows = rows / 1;
	std::size_t output_cols = cols / 1;

	rows /= 2;
	cols /= 2;

	double fps = 30.0;

	cv::Mat img_low_res(rows, cols * 2, CV_8UC3);
	cv::Mat img_full_res(output_rows, output_cols * 2, CV_8UC3);
	cv::Mat img_high_res(output_rows * 4, output_cols * 4 * 2, CV_8UC3);
	// cv::Mat rgb(rows, cols, CV_8UC3);
	// cv::Mat mask(rows, cols, CV_8UC3);

	double      ufo_total_ms{};
	double      interpolate_total_ms{};
	std::size_t iterations{};

	// float angle_x = 0.0f;
	float angle_x = M_PI;
	// float angle_y = M_PI_2;
	float angle_y = M_PI_2 - 0.8f;
	// float angle_y = M_PI_2 + 0.4f;
	float zoom = -0.0f;

	Image<std::pair<Color, float>> image_low_res(rows, cols);
	Image<std::pair<Color, float>> image_full_res(output_rows, output_cols);
	Image<std::pair<Color, float>> image_high_res(output_rows * 4, output_cols * 4);

	// f->wait();

	// Vec3f center(-0.85f, 16.1f, 200.84f);
	Vec3f center(-10, 0, 10);
	camera.lookAt(center, Vec3f(0, 0, 0));

	float linear_vel  = 1.0f;
	float angular_vel = 0.1f;

	std::size_t iter_since_moved        = 0;
	bool        show_free_space         = false;
	bool        show_free_space_changed = false;

	for (iterations = 1; true; ++iterations) {
		auto const render = std::chrono::high_resolution_clock::now();

		// Vec3f center(5.0f, 4.0f, 1.5f);
		// Vec3f center(4.59316f, -43.7646f, 36.0135f);
		// Vec3f center(-0.85f, 16.1f, 200.84f);

		// camera.pose.translation = position;  // (max - min) / 2.0f;
		float cx     = std::cos(angle_x);
		float cy     = std::cos(angle_y);
		float sx     = std::sin(angle_x);
		float sy     = std::sin(angle_y);
		Vec3f offset = Vec3f(sy * cx, sy * sx, cy) * std::exp(-zoom);
		// camera.lookAt(position + offset);
		// auto roll_mat = rotate(Mat4f(), roll, normalize(offset));
		// roll += 0.1f;
		// camera.up = Mat3f(roll_mat) * Vec3f(0.0f, 0.0f, 1.0f);
		camera.pose =
		    static_cast<Transform3f>(lookAt<float, true>(center + offset, center, camera.up));
		yaw -= 0.01;
		// angle_x += 0.01f;
		// angle_y += 0.05f;
		// pitch                   = 0;
		// roll                    = 0;  // M_PI_2;
		// camera.pose.orientation = Quat(yaw, roll, pitch);

		auto pred = pred::Z() > 0.0 && pred::Z() < 2.0;

		std::size_t const full_res_thres = 30;
		std::size_t const high_res_thres = 100;

		auto& image =
		    full_res_thres <= iter_since_moved
		        ? (high_res_thres <= iter_since_moved ? image_high_res : image_full_res)
		        : image_low_res;
		cv::Mat& img =
		    full_res_thres <= iter_since_moved
		        ? (high_res_thres <= iter_since_moved ? img_high_res : img_full_res)
		        : img_low_res;

		if (map_changed || 0 == iter_since_moved || full_res_thres == iter_since_moved ||
		    high_res_thres == iter_since_moved || show_free_space_changed) {
			iter_since_moved        = map_changed ? full_res_thres : iter_since_moved;
			map_changed             = false;
			show_free_space_changed = false;

			map.render(
			    execution::par, camera, image,
			    [&map, pred, show_free_space](auto node, Ray3 /* ray */,
			                                  float /* distance */) -> bool {
				    if (show_free_space) {
					    return map.containsOccupied(node) || map.containsFree(node);
				    } else {
					    return map.containsOccupied(node);
				    }
			    },
			    [&map, pred, show_free_space](auto node, Ray3 ray, float distance) {
				    if (show_free_space) {
					    bool unknown  = map.containsUnknown(node);
					    bool free     = map.containsFree(node);
					    bool occupied = map.containsOccupied(node);
					    bool leaf     = map.isLeaf(node);

					    bool leaf_and_occupied = leaf && occupied;
					    bool only_free         = (leaf && free) || (free && !occupied && !unknown);

					    Color c;
					    if (leaf_and_occupied) {
						    c = map.color(node);
					    } else if (only_free) {
						    c.green = 255;
					    }

					    return std::make_pair(leaf_and_occupied || only_free,
					                          std::make_pair(c, distance));
				    } else {
					    bool occupied = map.containsOccupied(node);
					    bool leaf     = map.isLeaf(node);

					    bool leaf_and_occupied = leaf && occupied;

					    Color c;
					    if (leaf_and_occupied) {
						    c = map.color(node);
					    }

					    return std::make_pair(leaf_and_occupied, std::make_pair(c, distance));
				    }
			    },
			    std::make_pair(Color{0, 0, 0, 0}, std::numeric_limits<float>::infinity()));

			timings.render.add(render);
		}

		auto const upscale = std::chrono::high_resolution_clock::now();

		float min_distance = std::numeric_limits<float>::max();
		float max_distance{};
		for (auto const& [_, distance] : image) {
			if (!std::isinf(distance)) {
				min_distance = std::min(min_distance, distance);
				max_distance = std::max(max_distance, distance);
			}
		}
		max_distance -= min_distance;

		auto f = [&](std::size_t i) {
			for (std::size_t j{}; j < image.cols(); ++j) {
				auto ufo_c              = image(i, j).first;
				img.at<cv::Vec3b>(i, j) = {ufo_c.blue, ufo_c.green, ufo_c.red};
				bool inf                = std::isinf(image(i, j).second);
				auto distance           = image(i, j).second - min_distance;
				auto dist_c             = 255 * (max_distance - distance) / max_distance;
				// dist_c                  = std::pow(dist_c, 1.0f / 2.2f);
				img.at<cv::Vec3b>(i, image.cols() + j) = cv::Vec3b::all(inf ? 0 : dist_c);

				// rgb.at<cv::Vec3b>(i, j)  = img.at<cv::Vec3b>(i, j);
				// mask.at<cv::Vec3b>(i, j) = cv::Vec3b::all(inf ? 0 : 255);
			}
		};

#if defined(UFO_TBB)
		IndexIterator<std::size_t> row_it(0, image.rows());
		std::for_each(std::execution::par_unseq, row_it.begin(), row_it.end(), f);
#else
#if defined(UFO_OMP)
#pragma omp parallel for
#endif
		for (std::size_t i = 0; i < image.rows(); ++i) {
			f(i);
		}
#endif

		cv::Mat result;
		cv::resize(img, result, cv::Size(), double(output_rows) / double(image.rows()),
		           double(output_cols) / double(image.cols()),
		           cv::INTER_AREA);  // INTER_LANCZOS4

		timings.upscale.add(upscale);

		timings.print();

		// cv::imwrite(RESOURCE_DIR "/ufo_highres.png", img);

		cv::imshow("UFOViz", result);

		auto total = std::chrono::duration<double, std::milli>(
		                 std::chrono::high_resolution_clock::now() - render)
		                 .count();
		std::cout << "Waiting for "
		          << std::max(1, static_cast<int>(1000 * (1.0 / fps) - total)) << " ms\n";
		// Wait for a keystroke in the window
		int k = cv::waitKey(std::max(1, static_cast<int>(1000 * (1.0 / fps) - total)));

		// cv::waitKey();

		++iter_since_moved;

		switch (k) {
			case 'h':
				center += inverse(camera.pose.rotation) * Vec3f(0, 0, -linear_vel * 10);
				iter_since_moved = 0;
				break;
			case 82:
			case 'w':
				center += inverse(camera.pose.rotation) * Vec3f(0, 0, -linear_vel);
				iter_since_moved = 0;
				break;
			case 'a':
				center += inverse(camera.pose.rotation) * Vec3f(-linear_vel, 0, 0);
				iter_since_moved = 0;
				break;
			case 84:
			case 's':
				center += inverse(camera.pose.rotation) * Vec3f(0, 0, linear_vel);
				iter_since_moved = 0;
				break;
			case 'd':
				center += inverse(camera.pose.rotation) * Vec3f(linear_vel, 0, 0);
				iter_since_moved = 0;
				break;
			case 'r':
				center += inverse(camera.pose.rotation) * Vec3f(0, linear_vel, 0);
				iter_since_moved = 0;
				break;
			case 'f':
				center += inverse(camera.pose.rotation) * Vec3f(0, -linear_vel, 0);
				iter_since_moved = 0;
				break;
			case 'i':
				angle_y += angular_vel;
				iter_since_moved = 0;
				break;
			case 81:
			case 'j':
				angle_x += angular_vel;
				iter_since_moved = 0;
				break;
			case 'k':
				angle_y += -angular_vel;
				iter_since_moved = 0;
				break;
			case 83:
			case 'l':
				angle_x += -angular_vel;
				iter_since_moved = 0;
				break;
			case 'q': linear_vel /= 1.1f; break;
			case 'e': linear_vel *= 1.1f; break;
			case 'u': angular_vel /= 1.1f; break;
			case 'o': angular_vel *= 1.1f; break;
			case 'g':
				show_free_space         = !show_free_space;
				show_free_space_changed = true;
				break;
			default: break;
		}

		// for (auto node : map.query(pred::Occupied() && pred::Intersects<BS<3, float>>(
		//                                                    BS<3, float>(center, 1.0f)))) {
		// 	std::cout << "We are collision\n";
		// 	break;
		// }

		std::cout << "Velocity: Linear: " << linear_vel << ", Angular: " << angular_vel
		          << ", Key Pressed: " << k << '\n';
	}
	return 0;
}
