#pragma once
#include "const_xslam.h"

#ifdef MOVI_C
//#include <config.hpp>
#include "ccommon.h"
#include "container.h"
#else
#include "config.h"
#include "ccommon.h"
#include "container.h"

#endif

void mat33_times_mat33(Float a[9], Float const b[9], Float const c[9]);
void mat33_transpose_times_mat33(Float a[9], Float const b[9], Float const c[9]);
void mat33_time_vec3_plus_vec3(Float a[3], Float const b[9], Float const c[3], Float const d[3]);
void mat33_time_vec3(Float a[3], Float const b[9], Float const c[3]);
void mat33_transpose_time_vec3(Float a[3], Float const b[9], Float const c[3]);

void vec3_minus_vec3(Float a[3], Float const b[3], Float const c[3]);
void from_world_to_camera_frame(XPose const* pose, XVec3 const* p3d, XVec3 * p3d_camera);
Float squared_norm3(Float const v3[3]);
Float norm(Float const v3[3]);
void normalized(Float const v3[3], Float out[3]);


XVec3 rotation_log(Float const r[9]);

void compose_rt(XCameraPose * camera_pose, XPose const* pose, XCamera const* xcamera);

int project(XCameraPose const* camera_pose, XVec3 const* p3d, XVec2 * p2d);
int project_in_bound(XCameraPose const* camera_pose, XVec3 const* p3d, XVec2 * p2d);


