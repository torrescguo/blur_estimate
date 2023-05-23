#pragma once
#include "ccommon.h"

int cwrite_image (char *folder, char *filename, char *image, int width, int height);


#ifdef WRITE_BUFFER_SAVE_MEMORY
int cwrite_coordinates (char *folder, char *filename, Coordinates *ccoordinates);

int cwrite_descriptors (char *folder, char *filename, Descriptors *cdescriptors);

int cwrite_coordinates_cid (char *folder, char *filename, Coordinates *ccoordinates, int camera_id);

int cwrite_descriptors_cid (char *folder, char *filename, Descriptors *cdescriptors, int camera_id);

int cwrite_feature (char *folder, char *filename, Features *map_features);

int cwrite_enable (char *folder, char *filename, Enable *map_enabls);

#else
int cwrite_coordinates (char *folder, char *filename, Coordinates *ccoordinates, char *holder);

int cwrite_descriptors (char *folder, char *filename, Descriptors *cdescriptors, char *holder);

int cwrite_coordinates_cid (char *folder, char *filename, Coordinates *ccoordinates, int camera_id, char *holder);

int cwrite_descriptors_cid (char *folder, char *filename, Descriptors *cdescriptors, int camera_id, char *holder);

int cwrite_feature (char *folder, char *filename, Features *map_features, char *holder);

int cwrite_enable (char *folder, char *filename, Enable *map_enables, char *holder);

int cwrite_match (char *folder, char *filename, Matches *matches_c, char *holder);

#endif

int cwrite_geo_output (char *folder, char *filename, XVec2 *p2d1_mat, XVec2 *p2d2_mat, XMat26 *mat261_mat, XMat26 *mat262_mat, int nb_test, char *holder);

int cwrite_loc_output (char *folder, char *filename, LocOutput locOutput);

int cwrite_map_output (char *folder, char *filename, MapOutput mapOutput);

int cwrite_comb_loc_output (char *folder, char *filename, int iter, BufferHolder *buf_holder, char *holder);

int cload_image (char *folder, char *filename, char *image, int width, int height);

int cload_debug_struct_buffer(char *folder, char *filename, char *dbg_buf, int size);

int cload_feature (char *folder, char *filename, Features *map_features, char *holder);

int cload_enable (char *folder, char *filename, Enable *map_enables, char *holder);

int cload_match (char *folder, char *filename, Matches *matches_c, char *holder);

int cload_cimage (char *folder, char *filename, CImage *cimage,  char *holder);

int cload_coordinates (char *folder, char *filename, Coordinates *ccoordinates, char *holder);

int cload_descriptors (char *folder, char *filename, Descriptors *cdescriptors, char *holder);

int cload_coordinates_cid (char *folder, char *filename, Coordinates *ccoordinates, char *holder);

int cload_descriptors_cid (char *folder, char *filename, Descriptors *cdescriptors, char *holder);


int cload_range2 (char *folder, char *filename, Range2 *crange2, char *holder);

int cload_floatimage (char *folder, char *filename, FloatImage *floatimage, char *holder);

int cload_calib (char *folder, char *filename, XStereo *stereo, char *holder);
int cload_geo_input (char *folder, char *filename, XVec3 *pose_angle_mat, XVec3 *pose_translation_mat, XVec3 *p3d_mat,  char *holder);

int cload_geo_output (char *folder, char *filename, XVec2 *p2d1_mat, XVec2 *p2d2_mat, XMat26 *mat261_mat, XMat26 *mat262_mat, int nb_test, char *holder);

int cload_loc_input (char *folder, char *filename, MatchedBase *cbase_ptr, XPose *pose_ptr, char *holder);

int cload_loc_output (char *folder, char *filename, LocOutput *locOutput, char *holder);

int cload_map_input (char *folder, char *filename, Map *map_ptr, char *holder);

int cload_map_output (char *folder, char *filename, MapOutput *mapOutput, char *holder);

int load_for_localize (char *folder, CFrame *cframes, FloatImage *floatimage, XStereo *stereo, int width, int height, char *holder);

int load_for_slam (char *folder, FloatImage *floatimage, XStereo *stereo, int width, int height, char *holder);

int load_data_to_cstruct_matching(char *folder,
						Features *map_features,
						Features *cur_features,
						Enable *map_enables,
						Enable *cur_enables,
						Matches *matches_c);

int load_data_to_cstruct_descriptor (char *folder, CFrame *cframes, Coordinates *ccoordinates, \
                                      Range2 *crange2,  FloatImage *floatimage, Descriptors *cdescriptors);

int check_descriptors (Descriptors *cdescriptors1, Descriptors *cdescriptors2);

int check_coordinates (Coordinates *coordinates1, Coordinates *coordinates2);

int check_geoOutput (GeoOutput *geoOutput1, GeoOutput *geoOutput2, int nb_test);

int check_locOutput (LocOutput *locOutput1, LocOutput *locOutput2);
int check_mapOutput (MapOutput *mapOutput1, MapOutput *mapOutput2);

int cload_check_comb_loc_output (char *folder, char *filename, int nb_iter, BufferHolder *buf_holder, char *holder);

int load_and_allocate_data_for_combined (char *folder, CFrame *cframes, FloatImage *floatimage, \
                                      Coordinates2 *ccoordinates2, Descriptors2 *cdescriptors2, \
                                      Coordinates2 *coordinates_cmp2, Descriptors2 *descriptors_cmp2, \
                                      int data_size, int width, int height);


