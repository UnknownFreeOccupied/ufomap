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
#include <ufo/map/distance/map.hpp>
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

static Color color(double value, double min_value, double max_value, double color_factor)
{
	double const s = 1;
	double const v = 1;

	value = std::clamp((value - min_value) / (max_value - min_value), 0.0, 1.0);

	double h = (1 - value) * color_factor;
	h -= std::floor(h);
	h *= 6;

	int const i = std::floor(h);

	double const f = (i & 1) ? h - i : 1 - (h - i);

	double const m = v * (1 - s);

	double const n = v * (1 - s * f);

	switch (i) {
		case 6:
		case 0: return Color(255 * m, 255 * n, 255 * v);
		case 1: return Color(255 * m, 255 * v, 255 * n);
		case 2: return Color(255 * n, 255 * v, 255 * m);
		case 3: return Color(255 * v, 255 * n, 255 * m);
		case 4: return Color(255 * v, 255 * m, 255 * n);
		case 5: return Color(255 * n, 255 * m, 255 * v);
		default: return Color(255 * 0.5, 255 * 0.5, 255 * 1);
	}
}

int main(int argc, char* argv[])
{
	// Vec4f v1;
	// Vec4f v2;

	// Mat4x4f m;
	// std::cout << "Before\n" << m << '\n';
	// std::cout << "After\n" << inverse(m) << '\n';
	// exit(1);

	Map2D<OccupancyMap, ColorMap, DistanceMap> map(0.01, 17);

	// ufo::Timing timer;

	Vec3f min(100000, 100000, 100000);
	Vec3f max(-100000, -100000, -100000);

	happly::PLYData ply_in(RESOURCE_DIR "/scene0000_00_vh_clean.ply");
	auto            v_pos = ply_in.getVertexPositions();
	auto            v_col = ply_in.getVertexColors();
	auto            f_ind = ply_in.getFaceIndices<std::size_t>();
	std::unordered_map<QuadCode, std::pair<std::vector<Vec2f>, std::vector<Color>>> points;

	auto const t1 = std::chrono::high_resolution_clock::now();
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
			// if (1.0f < t[i].z && 2.0 > t[i].z) {
			auto& [a, b] = points[map.code(Vec2f(t[i]))];
			a.emplace_back(Vec2f(t[i]));
			b.emplace_back(c[i]);
			// }
		}
	}

	auto const t2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> const preprocess_ms = t2 - t1;

	std::cout << std::setw(28) << std::left << "Preprocess: " << preprocess_ms.count()
	          << " ms\n";

	auto f = std::async(std::launch::async, [&map, &points]() {
		auto const t1 = std::chrono::high_resolution_clock::now();

		for (auto const& [code, elem] : points) {
			auto const& [points, colors] = elem;
			map.occupancySet(code, 1.0, false);
			map.colorSet(code, Color::blend(colors, ColorBlendingMode::SQ_MEAN), false);
			Vec2f p = std::accumulate(points.begin(), points.end(), Vec2f());
			p /= points.size();
			map.distanceSet(code, p, points.size(), false);
		}

		auto const t2 = std::chrono::high_resolution_clock::now();

		map.propagateModified();

		auto const t3 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> const set_ms       = t2 - t1;
		std::chrono::duration<double, std::milli> const propagate_ms = t3 - t2;
		std::chrono::duration<double, std::milli> const total_ms     = t3 - t1;

		std::cout << std::setw(28) << std::left << "Set:        " << set_ms.count()
		          << " ms\n";
		std::cout << std::setw(28) << std::left << "Propagate:  " << propagate_ms.count()
		          << " ms\n";
		std::cout << std::setw(28) << std::left << "Total:      " << total_ms.count()
		          << " ms\n";
	});

	f.wait();

	// std::ofstream xyzrgb("/home/dduberg/ufo/xyzrgb.txt");
	// for (auto node : map.query(pred::Leaf() && pred::Occupied())) {
	// 	auto center = map.center(node.index());
	// 	auto color  = map.color(node.index());
	// 	xyzrgb << center.x << ' ' << center.y << ' ' << 0.0f << ' ' << +color.red << ' '
	// 	       << +color.green << ' ' << +color.blue << '\n';
	// }
	// xyzrgb.close();

	// std::ofstream distancergb("/home/dduberg/ufo/distancergb.txt");
	// for (auto node : map.query(pred::Leaf() && pred::Occupied())) {
	// 	auto point = map.distanceInfo(node.index()).point;
	// 	auto color = map.color(node.index());
	// 	distancergb << point.x << ' ' << point.y << ' ' << 0.0f << ' ' << +color.red << ' '
	// 	            << +color.green << ' ' << +color.blue << '\n';
	// }
	// distancergb.close();

	// exit(0);

	// std::ofstream xyzrgb("/home/dduberg/ufo/xyzrgb.txt");
	// for (auto node : map.query(pred::Leaf() && pred::Occupied())) {
	// 	auto center = map.center(node.index());
	// 	auto color  = map.color(node.index());
	// 	xyzrgb << center.x << ' ' << center.y << ' ' << 0.0f << ' ' << +color.red << ' '
	// 	       << +color.green << ' ' << +color.blue << '\n';
	// }
	// xyzrgb.close();

	auto inner_f = [&map](auto node, Ray2 /* ray */, float /* distance */) -> bool {
		// return map.containsOccupied(node);
		return true;
	};

	auto hit_f = [&map](auto node, Ray2 ray,
	                    float distance) -> std::pair<bool, std::pair<Vec2f, Color>> {
		float max_distance = 7.0f;
		// return std::make_pair(map.isLeaf(node) && map.containsOccupied(node),
		//                       std::make_pair(map.center(node), map.color(node)));

		auto  d = map.distance<false>(ray.origin);
		Color c(255 * d / max_distance, 255 * d / max_distance, 255 * d / max_distance);

		return std::make_pair(true, std::make_pair(map.center(node), c));
	};

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
	camera.vertical_fov    = radians(60.0f);
	camera.near_clip       = 0.30310887f;
	camera.far_clip        = 4.7810888f;
	camera.projection_type = ProjectionType::PERSPECTIVE;

	std::size_t rows = 720;   // 360;  // 720;   // 1080;
	std::size_t cols = 1280;  // 640;  // 1280;  // 1920;

	std::size_t output_rows = rows / 1;
	std::size_t output_cols = cols / 1;

	rows /= 1;
	cols /= 1;

	double fps = 90.0;

	std::vector<std::size_t> row_idx(rows);
	std::iota(row_idx.begin(), row_idx.end(), 0);

	std::vector<std::size_t> output_row_idx(output_rows);
	std::iota(output_row_idx.begin(), output_row_idx.end(), 0);

	Image<std::pair<Color, Color>> image(rows, cols);

	cv::Mat img(output_rows, output_cols * 2, CV_8UC3);

	cv::Mat rgb(output_rows, output_cols, CV_8UC3);
	cv::Mat mask(output_rows, output_cols, CV_8UC3);

	double      ufo_total_ms{};
	double      interpolate_total_ms{};
	std::size_t iterations{};

	float angle_x = 0.0f;
	float angle_y = -0.0f;
	float zoom    = -1.8f;

	for (iterations = 1; true; ++iterations) {
		auto const t1 = std::chrono::high_resolution_clock::now();

		Vec3f eye(5.17831f, 4.56809f, 5.571f);
		Vec3f target(5.17831f, 4.568091f, 0.0f);

		// camera.pose.translation = position;  // (max - min) / 2.0f;
		float cx     = std::cos(angle_x);
		float cy     = std::cos(angle_y);
		float sx     = std::sin(angle_x);
		float sy     = std::sin(angle_y);
		Vec3f offset = Vec3f(sy * cx, sy * sx, cy) * std::exp(-zoom);
		// camera.lookAt(position + offset);
		// camera.lookAt(eye + offset, target);
		// camera.up     = normalize(camera.up + Vec3f(0.0f, 1.0f, 0.0f));
		auto roll_mat = rotate(Mat4f(), radians(-20.0f), normalize(target - eye));
		camera.up     = Mat3f(roll_mat) * camera.up;
		std::cout << roll_mat << '\n';
		std::cout << camera.up << '\n';
		camera.lookAt(eye, target);
		// angle_y += 0.01f;

		float max_dist{};
		map.render(
		    execution::par, camera, image,
		    [&map](auto node, Ray3 ray, float distance) -> bool {
			    // return map.containsOccupied(node);
			    return true;
		    },
		    [&map, &max_dist](auto node, Ray3 ray,
		                      float distance) -> std::pair<bool, std::pair<Color, Color>> {
			    float max_distance = 9.07359f;

			    if (map.isLeaf(node)) {
				    float d  = max_distance;
				    float d2 = max_distance;
				    // if (map.distanceContainsSurface(node)) {
				    auto [a, b] = map.distancePoint<true>(Vec2f(ray.step(distance)));
				    d           = map.distanceSomething(b, Vec2f(ray.step(distance)));
				    d2          = a;
				    // } else {
				    //   // d = map.distance<true>(Vec2f(ray.step(distance)));
				    // }

				    // d  = 0.015 > d ? d : max_distance;
				    // d2 = 0.015 > d2 ? d2 : max_distance;

				    max_dist = std::max(max_dist, d);
				    Color c  = color(d, 0.0, max_distance, 0.7);
				    Color c2 = color(d2, 0.0, max_distance, 0.7);

				    // if (0.01 > d) {
				    c2 = map.color(node);
				    // }

				    return std::make_pair(true, std::make_pair(c2, c));
			    } else {
				    return std::make_pair(false, std::make_pair(Color{}, Color{}));
			    }
		    },
		    std::make_pair(Color{}, Color{}));

		std::cout << max_dist << '\n';

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
				rgb.at<cv::Vec3b>(i, j)[2]               = image(i_scaled, j_scaled).first.red;
				rgb.at<cv::Vec3b>(i, j)[1]               = image(i_scaled, j_scaled).first.green;
				rgb.at<cv::Vec3b>(i, j)[0]               = image(i_scaled, j_scaled).first.blue;
				mask.at<cv::Vec3b>(i, j)[2] = image(i_scaled, j_scaled).first.empty() ? 0 : 255;
				mask.at<cv::Vec3b>(i, j)[1] = image(i_scaled, j_scaled).first.empty() ? 0 : 255;
				mask.at<cv::Vec3b>(i, j)[0] = image(i_scaled, j_scaled).first.empty() ? 0 : 255;
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

		imwrite("/home/dduberg/ufo/render/topdown.png", rgb);
		imwrite("/home/dduberg/ufo/render/topdown_mask.png", mask);
		exit(0);

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