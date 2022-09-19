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
#include <ufomap_rviz_plugins/render_display.h>

namespace ufomap_ros::rviz_plugins
{
RenderDisplay::RenderDisplay(rviz::Property* parent, std::string name,
                             RenderStyle default_style,
                             ColoringMode default_coloring_mode, QColor default_color,
                             float default_alpha, bool enabled, bool has_color,
                             bool has_time, bool has_semantics)
    : has_color_(has_color), has_time_(has_time), has_semantics_(has_semantics)
{
	// Should render
	should_render_ = new rviz::BoolProperty(
	    QString::fromStdString(name), enabled,
	    QString::fromStdString("Enable rendering of " + name + " space"), parent);
	should_render_->setDisableChildrenIfFalse(false);

	// Style
	style_ = new rviz::EnumProperty(
	    "Style", QString::fromStdString(std::string(getStr(default_style))),
	    "Select voxel style", should_render_);
	for (RenderStyle style :
	     {RenderStyle::POINTS, RenderStyle::SQUARES, RenderStyle::FLAT_SQUARES,
	      RenderStyle::SPHERES, RenderStyle::TILES, RenderStyle::BOXES}) {
		style_->addOptionStd(std::string(getStr(style)),
		                     static_cast<std::underlying_type<ColoringMode>::type>(style));
	}

	// Coloring
	coloring_ = new rviz::EnumProperty(
	    "Coloring", QString::fromStdString(std::string(getStr(default_coloring_mode))),
	    "Select voxel coloring mode", should_render_);
	updateColoring(default_coloring_mode);

	// Color
	for (auto coloring : {ColoringMode::OCCUPANCY_COLOR, ColoringMode::FIXED_COLOR}) {
		color_[coloring] = new rviz::ColorProperty("Color", default_color,
		                                           "Occupied space color", coloring_);
		color_[coloring]->setHidden(ColoringMode(coloring_->getOptionInt()) != coloring);
	}

	// Color factor
	for (auto coloring : {ColoringMode::time_COLOR, ColoringMode::X_AXIS_COLOR,
	                      ColoringMode::Y_AXIS_COLOR, ColoringMode::Z_AXIS_COLOR}) {
		color_factor_[coloring] = new rviz::FloatProperty("Factor", 0.8, "", coloring_);
		color_factor_[coloring]->setMin(0.0);
		color_factor_[coloring]->setMax(1.0);
		color_factor_[coloring]->setHidden(ColoringMode(coloring_->getOptionInt()) !=
		                                   coloring);
	}

	// Color normalize range
	for (auto coloring : {ColoringMode::time_COLOR, ColoringMode::X_AXIS_COLOR,
	                      ColoringMode::Y_AXIS_COLOR, ColoringMode::Z_AXIS_COLOR}) {
		color_normalize_range_[coloring] =
		    new rviz::BoolProperty("Normalize Range", true,
		                           "If set to true, will try to estimate the range of "
		                           "possible values from the received map",
		                           coloring_);
		color_normalize_range_[coloring]->setHidden(ColoringMode(coloring_->getOptionInt()) !=
		                                            coloring);
	}

	// Color normalize min/max
	for (auto coloring : {ColoringMode::X_AXIS_COLOR, ColoringMode::Y_AXIS_COLOR,
	                      ColoringMode::Z_AXIS_COLOR}) {
		color_normalize_min_value_[coloring] = new rviz::FloatProperty(
		    "Min Value", 0.0, "Value which will displayed as min color value",
		    color_normalize_range_[coloring]);
		color_normalize_max_value_[coloring] = new rviz::FloatProperty(
		    "Max Value", 1.0, "Value which will displayed as max color value",
		    color_normalize_range_[coloring]);
		color_normalize_min_value_[coloring]->setHidden(
		    color_normalize_range_[coloring]->getBool());
		color_normalize_max_value_[coloring]->setHidden(
		    color_normalize_range_[coloring]->getBool());
	}

	// Time step normalize min/max
	time_normalize_min_value_ = new rviz::IntProperty(
	    "Min Value", 0, "Value which will displayed as min color value",
	    color_normalize_range_[ColoringMode::time_COLOR]);
	time_normalize_max_value_ = new rviz::IntProperty(
	    "Max Value", std::pow(2, 24), "Value which will displayed as max color value",
	    color_normalize_range_[ColoringMode::time_COLOR]);
	time_normalize_min_value_->setMin(0);
	time_normalize_min_value_->setMax(std::pow(2, 24));
	time_normalize_max_value_->setMin(0);
	time_normalize_max_value_->setMax(std::pow(2, 24));
	time_normalize_min_value_->setHidden(
	    color_normalize_range_[ColoringMode::time_COLOR]->getBool());
	time_normalize_max_value_->setHidden(
	    color_normalize_range_[ColoringMode::time_COLOR]->getBool());

	// Semantics
	use_semantic_ = new rviz::BoolProperty("Use Semantics", false, "", should_render_);
	use_semantic_->setDisableChildrenIfFalse(true);
	use_semantic_->setHidden(!has_semantics_);

	// Alpha
	alpha_ = new rviz::FloatProperty("Alpha", default_alpha, "Set alpha (transparency)",
	                                 should_render_);
	alpha_->setMin(0.0);
	alpha_->setMax(1.0);

	// Scale
	scale_ = new rviz::FloatProperty("Scale", 1.0, "Set scale", should_render_);
	scale_->setMin(0.0);

	// Depth
	depth_ = new rviz::IntProperty("Min. Depth", 0, "Set render depth", should_render_);
	depth_->setMin(0);
	depth_->setMax(21);

	//
	// Connection
	//

	// Coloring
	QObject::connect(coloring_, &rviz::EnumProperty::changed, [this]() {
		for (auto& [_, value] : color_) {
			value->hide();
		}
		for (auto& [_, value] : color_factor_) {
			value->hide();
		}
		for (auto& [_, value] : color_normalize_range_) {
			value->hide();
		}

		ColoringMode mode = static_cast<ColoringMode>(coloring_->getOptionInt());
		switch (mode) {
			case ColoringMode::VOXEL_COLOR:
			case ColoringMode::SEMANTIC_COLOR:
				break;
			case ColoringMode::time_COLOR:
			case ColoringMode::X_AXIS_COLOR:
			case ColoringMode::Y_AXIS_COLOR:
			case ColoringMode::Z_AXIS_COLOR:
				color_factor_.at(mode)->show();
				color_normalize_range_.at(mode)->show();
				break;
			case ColoringMode::OCCUPANCY_COLOR:
			case ColoringMode::FIXED_COLOR:
				color_.at(mode)->show();
				break;
		}
	});

	// Color normalize range
	for (auto coloring : {ColoringMode::time_COLOR, ColoringMode::X_AXIS_COLOR,
	                      ColoringMode::Y_AXIS_COLOR, ColoringMode::Z_AXIS_COLOR}) {
		QObject::connect(color_normalize_range_[coloring], &rviz::BoolProperty::changed,
		                 [this]() {
			                 for (auto& [key, value] : color_normalize_range_) {
				                 switch (key) {
					                 case ColoringMode::time_COLOR:
						                 time_normalize_min_value_->setHidden(value->getBool());
						                 time_normalize_max_value_->setHidden(value->getBool());
						                 break;
					                 case ColoringMode::X_AXIS_COLOR:
					                 case ColoringMode::Y_AXIS_COLOR:
					                 case ColoringMode::Z_AXIS_COLOR:
						                 color_normalize_min_value_[key]->setHidden(value->getBool());
						                 color_normalize_max_value_[key]->setHidden(value->getBool());
						                 break;
					                 default:
						                 break;
				                 }
			                 }
		                 });
	}
}

bool RenderDisplay::isEnabled() const { return should_render_->getBool(); }

void RenderDisplay::enableVoxelColor(bool enable)
{
	has_color_ = enable;
	updateColoring(static_cast<ColoringMode>(coloring_->getOptionInt()));
}

void RenderDisplay::enableTime(bool enable)
{
	has_time_ = enable;
	updateColoring(static_cast<ColoringMode>(coloring_->getOptionInt()));
}

void RenderDisplay::enableSemantics(bool enable)
{
	has_semantics_ = enable;
	updateColoring(static_cast<ColoringMode>(coloring_->getOptionInt()));
}

RenderMode RenderDisplay::getRenderMode(double min_occ, double max_occ) const
{
	RenderMode options;

	options.should_render = should_render_->getBool();

	options.style = static_cast<RenderStyle>(style_->getOptionInt());

	options.coloring_mode = static_cast<ColoringMode>(coloring_->getOptionInt());
	if (ColoringMode::OCCUPANCY_COLOR == options.coloring_mode ||
	    ColoringMode::FIXED_COLOR == options.coloring_mode) {
		options.color = color_.at(options.coloring_mode)->getOgreColor();
	}

	for (auto option : {ColoringMode::time_COLOR, ColoringMode::X_AXIS_COLOR,
	                    ColoringMode::Y_AXIS_COLOR, ColoringMode::Z_AXIS_COLOR}) {
		if (option == options.coloring_mode) {
			options.color_factor = color_factor_.at(option)->getFloat();
			break;
		}
	}

	for (auto option : {ColoringMode::X_AXIS_COLOR, ColoringMode::Y_AXIS_COLOR,
	                    ColoringMode::Z_AXIS_COLOR}) {
		if (option == options.coloring_mode) {
			options.normalized_value = color_normalize_range_.at(option)->getBool();
			options.min_normalized_value = color_normalize_min_value_.at(option)->getFloat();
			options.max_normalized_value = color_normalize_max_value_.at(option)->getFloat();
			break;
		}
	}

	if (ColoringMode::time_COLOR == options.coloring_mode) {
		options.normalized_value =
		    color_normalize_range_.at(ColoringMode::time_COLOR)->getBool();
		options.min_normalized_value = time_normalize_min_value_->getInt();
		options.max_normalized_value = time_normalize_max_value_->getInt();
	}

	if (ColoringMode::OCCUPANCY_COLOR == options.coloring_mode) {
		options.normalized_value = false;
		options.min_normalized_value = min_occ;
		options.max_normalized_value = max_occ;
	}

	options.alpha = alpha_->getFloat();
	options.scale = scale_->getFloat();
	options.depth = depth_->getInt();

	return options;
}

void RenderDisplay::updateColoring(ColoringMode wanted)
{
	coloring_->clearOptions();

	std::vector<ColoringMode> options;
	if (has_color_) {
		options.push_back(ColoringMode::VOXEL_COLOR);
	}
	if (has_time_) {
		options.push_back(ColoringMode::time_COLOR);
	}
	if (has_semantics_) {
		options.push_back(ColoringMode::SEMANTIC_COLOR);
	}
	options.push_back(ColoringMode::X_AXIS_COLOR);
	options.push_back(ColoringMode::Y_AXIS_COLOR);
	options.push_back(ColoringMode::Z_AXIS_COLOR);
	options.push_back(ColoringMode::OCCUPANCY_COLOR);
	options.push_back(ColoringMode::FIXED_COLOR);

	for (auto option : options) {
		coloring_->addOptionStd(
		    std::string(getStr(option)),
		    static_cast<std::underlying_type<ColoringMode>::type>(option));
	}
	// coloring_->sortOptions();

	if (ColoringMode::VOXEL_COLOR == wanted) {
		if (has_color_) {
			coloring_->setStdString(std::string(getStr(wanted)));
		} else {
			coloring_->setStdString(std::string(getStr(ColoringMode::Z_AXIS_COLOR)));
		}

	} else if (ColoringMode::time_COLOR == wanted) {
		if (has_time_) {
			coloring_->setStdString(std::string(getStr(wanted)));

		} else {
			coloring_->setStdString(std::string(getStr(ColoringMode::Z_AXIS_COLOR)));
		}
	} else if (ColoringMode::SEMANTIC_COLOR == wanted) {
		if (has_semantics_) {
			coloring_->setStdString(std::string(getStr(wanted)));

		} else {
			coloring_->setStdString(std::string(getStr(ColoringMode::Z_AXIS_COLOR)));
		}
	} else {
		coloring_->setStdString(std::string(getStr(wanted)));
	}
}
}  // namespace ufomap_ros::rviz_plugins