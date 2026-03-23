#ifndef VISION_EXTRACT_MAIN_H
#define VISION_EXTRACT_MAIN_H

#include "headfile.h"
#include "imgproc.h"
#include "utils.h"
#include "flash_param.h"
#include <stdbool.h>

#define ROAD_WIDTH      (0.45f)
#define POINTS_MAX_LEN  (MT9V03X_CSI_H)
#define FAR_POINTS_MAX_LEN (POINTS_MAX_LEN)

extern image_t img_raw;
extern image_t img_thres;
extern image_t img_line;

extern bool line_show_sample;
extern bool line_show_blur;
extern bool track_left;
extern float angle;

extern int ipts0[POINTS_MAX_LEN][2];
extern int ipts1[POINTS_MAX_LEN][2];
extern int ipts0_num, ipts1_num;

extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern int rpts0_num, rpts1_num;

extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern int rpts0b_num, rpts1b_num;

extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern int rpts0s_num, rpts1s_num;

extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern int rpts0a_num, rpts1a_num;

extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern int rpts0an_num, rpts1an_num;

extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern int rptsc0_num, rptsc1_num;

extern float (*rpts)[2];
extern int rpts_num;

extern float rptsn[POINTS_MAX_LEN][2];
extern int rptsn_num;

extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
extern bool Ypt0_found, Ypt1_found;

extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

extern bool far_Lpt0_found, far_Lpt1_found;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
extern int far_ipts0[FAR_POINTS_MAX_LEN][2];
extern int far_ipts1[FAR_POINTS_MAX_LEN][2];
extern int far_ipts0_num, far_ipts1_num;
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0_num, far_rpts1_num;
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0b_num, far_rpts1b_num;
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0s_num, far_rpts1s_num;
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern int far_rpts0a_num, far_rpts1a_num;
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern int far_rpts0an_num, far_rpts1an_num;

extern bool is_straight0, is_straight1;

enum track_type_e {
    TRACK_LEFT,
    TRACK_RIGHT,
};
extern enum track_type_e track_type;

#endif
