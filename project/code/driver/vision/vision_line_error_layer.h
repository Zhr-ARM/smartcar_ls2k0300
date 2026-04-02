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

void vision_line_error_layer_get_track_point(bool *valid, int *x, int *y);
int vision_line_error_layer_track_index();

void vision_line_error_layer_set_curvature_step(int step);
int vision_line_error_layer_curvature_step();
void vision_line_error_layer_get_selected_centerline_curvature(const float **curvature, int *count);
void vision_line_error_layer_get_curvature_lookahead_debug(float *speed_v,
                                                           float *k_eff,
                                                           float *eta,
                                                           int *lookahead_index);
void vision_line_error_layer_get_curvature_weighted_error_debug(float *weighted_error,
                                                                bool *lookahead_point_valid,
                                                                int *lookahead_point_x,
                                                                int *lookahead_point_y);
void vision_line_error_layer_get_curvature_speed_limit_debug(float *kappa_max,
                                                             float *delta_kappa_max,
                                                             float *curve_base_speed,
                                                             float *v_curve_raw,
                                                             float *v_curve_after_dkappa,
                                                             float *v_error_limit,
                                                             float *v_target);

int vision_line_error_layer_weighted_first_point_error();
int vision_line_error_layer_weighted_current_spacing();
void vision_line_error_layer_get_ipm_weighted_decision_point(bool *valid, int *x, int *y);
void vision_line_error_layer_get_src_weighted_decision_point(bool *valid, int *x, int *y);

#endif
