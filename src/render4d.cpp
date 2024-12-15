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
#include <ufo/math/trans2.hpp>
#include <ufo/vision/image.hpp>
// #include <ufo/util/timing.hpp>
#include <ufo/pcl/cloud.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/vision/camera.hpp>

// STL
#include <bitset>
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
#include <vector>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Happly
#include <ufo/happly.h>

using namespace ufo;

int main(int argc, char* argv[])
{
	// Vec4f v1;
	// Vec4f v2;

	// Mat4x4f m;
	// std::cout << "Before\n" << m << '\n';
	// std::cout << "After\n" << inverse(m) << '\n';
	// exit(1);

	Map4D<OccupancyMap, ColorMap> map(0.01, 16);

	// ufo::Timing timer;

	Vec3f min(100000, 100000, 100000);
	Vec3f max(-100000, -100000, -100000);

	happly::PLYData ply_in(RESOURCE_DIR "/scene0000_00_vh_clean.ply");
	auto            v_pos = ply_in.getVertexPositions();
	auto            v_col = ply_in.getVertexColors();
	auto            f_ind = ply_in.getFaceIndices<std::size_t>();
	std::unordered_map<HexCode, std::vector<Color>> points;
	for (auto f : f_ind) {
		Triangle3 t(Vec3f(v_pos[f[0]][0], v_pos[f[0]][1], v_pos[f[0]][2]),
		            Vec3f(v_pos[f[1]][0], v_pos[f[1]][1], v_pos[f[1]][2]),
		            Vec3f(v_pos[f[2]][0], v_pos[f[2]][1], v_pos[f[2]][2]));

		std::array c{Color(v_col[f[0]][0], v_col[f[0]][1], v_col[f[0]][2]),
		             Color(v_col[f[1]][0], v_col[f[1]][1], v_col[f[1]][2]),
		             Color(v_col[f[2]][0], v_col[f[2]][1], v_col[f[2]][2])};

		for (std::size_t i{}; 3 > i; ++i) {
			min = ufo::min(min, t[i]);
			max = ufo::max(max, t[i]);
			points[map.code(Vec4f(t[i], 0.004f))].emplace_back(c[i]);
			points[map.code(Vec4f(t[i], 1.004f))].emplace_back(255, c[i].green, c[i].blue,
			                                                   c[i].alpha);
			points[map.code(Vec4f(t[i], 2.004f))].emplace_back(c[i].red, 255, c[i].blue,
			                                                   c[i].alpha);
			points[map.code(Vec4f(t[i], 3.004f))].emplace_back(c[i].red, c[i].green, 255,
			                                                   c[i].alpha);
		}
	}

	auto f = std::async(std::launch::async, [&map, &points]() {
		for (auto const& [code, colors] : points) {
			map.occupancySet(code, 1.0, true);
			map.colorSet(code, Color::blend(colors, ColorBlendingMode::SQ_MEAN), true);
		}
	});

	// f.wait();

	// std::ofstream xyzrgb("/home/dduberg/ufo/xyzrgb.txt");
	// for (auto node : map.query(pred::Leaf() && pred::Occupied())) {
	// 	auto center = map.center(node.index());
	// 	auto color  = map.color(node.index());
	// 	xyzrgb << center.x << ' ' << center.y << ' ' << 0.0f << ' ' << +color.red << ' '
	// 	       << +color.green << ' ' << +color.blue << '\n';
	// }
	// xyzrgb.close();

	// auto inner_f = [&map](auto node, Ray2 /* ray */, float /* distance */) -> bool {
	// 	return map.containsOccupied(node);
	// };

	// auto hit_f = [&map](auto node, Ray2 ray,
	//                     float distance) -> std::pair<bool, std::pair<Vec2f, Color>> {
	// 	float max_distance = 7.0f;
	// 	return std::make_pair(map.isLeaf(node) && map.containsOccupied(node),
	// 	                      std::make_pair(map.center(node), map.color(node)));
	// };

	// Ray2 ray(Vec2f(max - min) / 2.0f, Vec2f(1, 0));
	// for (float theta{}; 2 * numbers::pi_v<float> > theta;
	//      theta += (2 * numbers::pi_v<float>) / 360.0f) {
	// 	ray.direction.x = std::cos(theta);
	// 	ray.direction.y = std::sin(theta);

	// 	auto [p, c] = map.trace(
	// 	    ray, inner_f, hit_f,
	// 	    std::make_pair(Vec2f(std::numeric_limits<float>::quiet_NaN()), Color()));

	// 	if (std::isnan(p.x)) {
	// 		continue;
	// 	}

	// 	xyzrgb << p.x << ' ' << p.y << ' ' << 0.0f << ' ' << +c.red << ' ' << +c.green << '
	// '
	// 	       << +c.blue << '\n';
	// }
	// xyzrgb.close();

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

	std::vector<std::size_t> row_idx(rows);
	std::iota(row_idx.begin(), row_idx.end(), 0);

	std::vector<std::size_t> output_row_idx(output_rows);
	std::iota(output_row_idx.begin(), output_row_idx.end(), 0);

	Image<std::pair<Color, Color>> image(rows, cols);

	cv::Mat img(output_rows, output_cols * 2, CV_8UC3);

	double      ufo_total_ms{};
	double      interpolate_total_ms{};
	std::size_t iterations{};

	float angle_x = 0.0f;
	float angle_y = M_PI_2;
	float zoom    = -0.0f;

	float w = 0.004f;

	for (iterations = 1; true; ++iterations) {
		auto const t1 = std::chrono::high_resolution_clock::now();

		if (0 == iterations % 50) {
			w = w + 1.0f;
			if (4 < w) {
				w = 0.004f;
			}
		}

		Vec3f center(5.0f, 4.0f, 1.5f);

		// camera.pose.translation = position;  // (max - min) / 2.0f;
		float cx     = std::cos(angle_x);
		float cy     = std::cos(angle_y);
		float sx     = std::sin(angle_x);
		float sy     = std::sin(angle_y);
		Vec3f offset = Vec3f(sy * cx, sy * sx, cy) * std::exp(-zoom);
		// camera.lookAt(position + offset);
		camera.pose =
		    static_cast<Trans3f>(lookAt<float, true>(center, center + offset, camera.up));
		angle_x += 0.01f;

		map.render(
		    execution::par, w, camera, image,
		    [&map](auto node, Ray4 /* ray */, float /* distance */) -> bool {
			    return map.containsOccupied(node);
		    },
		    [&map](auto node, Ray4 ray,
		           float distance) -> std::pair<bool, std::pair<Color, Color>> {
			    float max_distance = 7.0f;
			    return std::make_pair(
			        map.isLeaf(node) && map.containsOccupied(node),
			        std::make_pair(map.color(node),
			                       Color(256 - 256 * distance / max_distance,
			                             256 - 256 * distance / max_distance,
			                             256 - 256 * distance / max_distance)));
		    },
		    std::make_pair(Color{}, Color{}));

		auto const t2 = std::chrono::high_resolution_clock::now();

		double row_scale = image.rows() / static_cast<double>(output_rows);
		double col_scale = image.cols() / static_cast<double>(output_cols);

		auto f = [&](std::size_t i) {
			for (std::size_t j{}; j < output_cols; ++j) {
				std::size_t i_scaled                     = std::floor(i * row_scale);
				std::size_t j_scaled                     = std::floor(j * col_scale);
				img.at<cv::Vec3b>(i, j)[2]               = image(i_scaled, j_scaled).first.red;
				img.at<cv::Vec3b>(i, j)[1]               = image(i_scaled, j_scaled).first.green;
				img.at<cv::Vec3b>(i, j)[0]               = image(i_scaled, j_scaled).first.blue;
				img.at<cv::Vec3b>(i, output_cols + j)[2] = image(i_scaled, j_scaled).second.red;
				img.at<cv::Vec3b>(i, output_cols + j)[1] = image(i_scaled, j_scaled).second.green;
				img.at<cv::Vec3b>(i, output_cols + j)[0] = image(i_scaled, j_scaled).second.blue;
			}
		};

#if defined(UFO_TBB)
		std::for_each(std::execution::par_unseq, output_row_idx.begin(), output_row_idx.end(),
		              f);
#else
#if defined(UFO_OMP)
#pragma omp parallel for
#endif
		for (std::size_t i = 0; i < output_rows; ++i) {
			f(i);
		}
#endif

		auto const t3 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> const ufo_ms         = t2 - t1;
		std::chrono::duration<double, std::milli> const interpolate_ms = t3 - t2;

		ufo_total_ms += ufo_ms.count();
		interpolate_total_ms += interpolate_ms.count();

		std::chrono::duration<double, std::milli> const total_ms = t3 - t1;

		std::cout << std::setw(28) << std::left
		          << "UFO took:         " << (ufo_total_ms / iterations) << " ms\n";
		std::cout << std::setw(28) << std::left
		          << "Interpolate took: " << (interpolate_total_ms / iterations) << " ms\n";
		std::cout << std::setw(28) << std::left << "Total:            "
		          << ((ufo_total_ms + interpolate_total_ms) / iterations) << " ms\n";

		cv::imshow("UFOMap Rendered View", img);
		std::cout << "Wait "
		          << std::max(1, static_cast<int>(1000 * (1.0 / fps) - total_ms.count()))
		          << " ms\n";
		// Wait for a keystroke in the window
		int k =
		    cv::waitKey(std::max(1, static_cast<int>(1000 * (1.0 / fps) - total_ms.count())));

		// cv::waitKey();
	}

	return 0;
}