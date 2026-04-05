# LoongCar Dashboard Data Contract

Source of truth: `project/code/driver/vision/vision_transport.cpp`

## Notes
- Fields controlled by `udp_web_tcp_send_*` switches are conditional.
- Frontend must tolerate missing fields and render `--` or `count=0`.

## Scalar fields
- `ts_ms`
- `line_error`
- `base_speed`
- `adjusted_base_speed`
- `left_target_count`
- `right_target_count`
- `left_current_count`
- `right_current_count`
- `left_filtered_count`
- `right_filtered_count`
- `left_error`
- `right_error`
- `left_feedforward`
- `right_feedforward`
- `left_correction`
- `right_correction`
- `left_decel_assist`
- `right_decel_assist`
- `left_duty`
- `right_duty`
- `left_hardware_duty`
- `right_hardware_duty`
- `left_dir_level`
- `right_dir_level`
- `otsu_threshold`
- `capture_wait_us`
- `preprocess_us`
- `otsu_us`
- `maze_us`
- `total_us`
- `maze_left_points_raw`
- `maze_right_points_raw`
- `red_found`
- `roi_valid`
- `ipm_track_valid`
- `ipm_track_method`
- `ipm_centerline_source`
- `ipm_track_index`
- `ipm_weighted_first_point_error`
- `web_data_profile`

## Array fields
- `gray_size`
- `ipm_size`
- `red`
- `roi`
- `ipm_track_point`
- `ipm_weighted_decision_point`
- `src_weighted_decision_point`
- `left_boundary`
- `right_boundary`
- `left_auxiliary_line`
- `right_auxiliary_line`
- `left_auxiliary_seed`
- `right_auxiliary_seed`
- `left_boundary_corner`
- `right_boundary_corner`
- `ipm_left_boundary`
- `ipm_right_boundary`
- `ipm_left_boundary_corner`
- `ipm_right_boundary_corner`
- `ipm_centerline_selected_shift`
- `src_centerline_selected_shift`
- `ipm_centerline_selected_curvature`
- `ipm_left_boundary_curvature`
- `ipm_right_boundary_curvature`
- `ipm_left_boundary_angle_cos`
- `ipm_right_boundary_angle_cos`

## Counters
- `ipm_centerline_selected_count`
- `src_centerline_selected_count`

## Frontend policy
- Frontend uses backend field names directly.
- No alias mapping layer.
- Missing field should render as `--` or `count=0` without breaking charts.
