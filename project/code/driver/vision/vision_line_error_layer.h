#ifndef VISION_LINE_ERROR_LAYER_H_
#define VISION_LINE_ERROR_LAYER_H_

#include "zf_common_headfile.h"

#include <cstddef>

void vision_line_error_layer_reset();

int vision_line_error_layer_compute_from_ipm_shifted_centerline(const uint16 *ipm_center_x,
                                                                 const uint16 *ipm_center_y,
                                                                 int ipm_center_count,
                                                                 const uint16 *src_center_x,
                                                                 const uint16 *src_center_y,
                                                                 int src_center_count,
                                                                 int ipm_center_x_ref);

void vision_line_error_layer_set_source(int source);
int vision_line_error_layer_source();

void vision_line_error_layer_set_method(int method);
int vision_line_error_layer_method();

void vision_line_error_layer_set_fixed_index(int point_index);
int vision_line_error_layer_fixed_index();

void vision_line_error_layer_set_weighted_points(const int *point_indices,
                                                 const float *weights,
                                                 size_t count);
size_t vision_line_error_layer_weighted_point_count();

void vision_line_error_layer_set_speed_formula(float speed_k, float speed_b);
void vision_line_error_layer_get_speed_formula(float *speed_k, float *speed_b);

void vision_line_error_layer_set_index_range(int index_min, int index_max);
void vision_line_error_layer_get_index_range(int *index_min, int *index_max);

void vision_line_error_layer_set_prefix_exp_params(float prefix_ratio, float exp_lambda);
void vision_line_error_layer_get_prefix_exp_params(float *prefix_ratio, float *exp_lambda);

void vision_line_error_layer_get_track_point(bool *valid, int *x, int *y);
int vision_line_error_layer_track_index();

void vision_line_error_layer_set_curvature_step(int step);
int vision_line_error_layer_curvature_step();
void vision_line_error_layer_set_curvature_enabled(bool enabled);
bool vision_line_error_layer_curvature_enabled();
float vision_line_error_layer_mean_abs_offset();
int vision_line_error_layer_selected_centerline_count();
int vision_line_error_layer_required_last_index_for_straight();
float vision_line_error_layer_abs_error_sum_to_required_index();
float vision_line_error_layer_front_weighted_abs_error_sum(int point_count);
float vision_line_error_layer_segmented_blended_abs_error(float split_ratio,
                                                          float front_weight,
                                                          float rear_weight);
void vision_line_error_layer_speed_index_tail_mean_target_point(float start_index_offset_b,
                                                                bool *valid,
                                                                int *x,
                                                                int *y);

#endif
