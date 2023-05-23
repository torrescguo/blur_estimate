#pragma once
#include "const_xslam.h"
#include <stdint.h>
#include <stdbool.h>

// Range
typedef struct {
    int begin;
    int end; 
} Range;

// 2 ranges
typedef struct 
{
  Range data[2];
} Range2;


typedef struct {
	u16 first;
	u16 second;
} Match;

typedef struct {
	int size;
	Match * pData;
} Matches;	// container, pData point to data buffer


typedef struct {
	u16 x;
	u16 y;
} Coordinate;

typedef struct {
	s16 data[DESCRIPTOR_LENGTH];
} Descriptor;


typedef struct
{
  unsigned char tab[512];
  int threshold;
  int cpt;
} AreaThreshold;

typedef struct
{
  int vd,hd;
  AreaThreshold thresholds[4];
  int detector_is_initialized;
} AreaThresholds;


typedef struct {
	// int num_feature_left;
	// int num_feature_rite;
  Range2 range2;
	Coordinate * pCoordinate;
	Descriptor * pDescriptor;
} Features;	// container, pCoordinate and pDescriptor point to data buffer

typedef struct {
	int size;
	char * pData;
} Enable;	// container, pData point to data buffer

typedef struct {
	int radius_x;
	int radius_y;
	float score_min;
} MatchConfig;

typedef struct {
	int size;
	Coordinate * pCoordinate;
} Coordinates;	// container, pCoordinate point to data buffer

typedef struct {
	int size;
	Descriptor * pDescriptor;
} Descriptors;	// container, pCoordinate and pDescriptor point to data buffer


typedef struct {
  Coordinates Coord2[2];
} Coordinates2;	// container, pCoordinate point to data buffer

typedef struct {
	Descriptors Desc2[2];
} Descriptors2;	// container, pCoordinate and pDescriptor point to data buffer

typedef struct
{
  int id_pose,id_camera,begin,end;
} PoseRange;

typedef struct 
{
  unsigned char *ptr;
  int rows,cols;
} CImage;

// A tile is a rectangular region (e.g. upper left quarter ) of an image frame
typedef struct 
{
  int tileWidth;  // width of tile including max apron
  int tileHeight; // hieght of tile including max apron
  int blockW;     // actual data block size, excluding apron
  int blockH;     // actual data block size, excluding apron
  int frameStride;// width of the frame to which the tile belongs
  int tileSize;   // size of tile (tileWidth*tileHeight)
  int NW_x;       // x offset refering to global coordinate in frame
  int NW_y;       // y offset refering to global coordinate in frame

  int apronL;     // apron left, number of pixels within tile
  int apronU;     // apron up, number of lines from the top within tile
  int apronR;     // apron right, number of pixels within tile
  int apronD;     // apron down, number of lines from the top within tile

  void* pData;    // pointer to actual image data
} CTile;


typedef struct
{
  int row_start, row_end;
  int col_start, col_end;
} CTile_;

// A frame contains 2 images
typedef struct
{
  CImage data[2];
  Float timestamp;
  int frameNo;
} CFrame;


typedef struct
{
  s16 data[_13x13];
} FloatImage13x13;

typedef struct
{
  s16 *ptr;
  int rows,cols;
} FloatImage;


typedef struct 
{
  Float data[36];
} XMat66;

typedef struct 
{
  Float data[18];
} XMat63;

typedef struct 
{
  Float data[12];
} XMat26;

typedef struct 
{
  Float data[6];
} XMat23;

typedef struct 
{
  Float data[9];
} XMat33;

typedef struct 
{
  Float data[81];
} XMat99;

typedef struct 
{
  Float data[27];
} XMat39;

typedef struct 
{
  Float data[7];
} XVec7;

typedef struct 
{
  Float data[6];
} XVec6;

typedef struct 
{
  Float data[5];
} XVec5;

typedef struct 
{
  Float data[3];
} XVec3;


typedef struct 
{
  Float data[8];
} XVec8;

typedef struct 
{
  Float data[9];
} XVec9;

typedef struct 
{
  Float data[2];
} XVec2;

typedef struct 
{
  int data[5];
} IVec5;

typedef struct 
{
  int data[2];
} IVec2;

typedef struct 
{
  XMat33 rotation;
  XVec3 translation;
} XPose;

typedef struct
{
  XMat33 rotation;
  XVec3 translation;
  XVec3 velocity;
} XPose9;

typedef struct 
{
  XPose Rt;
  XVec7 intrinsic;
} XCamera;


typedef struct 
{
  XPose pose;
  XVec7 const *intrinsic;
} XCameraPose;


typedef struct 
{
  XCamera cameras[2];
} XStereo;




typedef struct
{
  int capacity;
  int element_size;
  int nb_element;
  void *data;
} Vector;


typedef struct 
{
  Vector vector;
} VectorInt;

typedef struct 
{
  Vector vector;
} VectorInt2;

typedef struct
{
  XCamera* camera;
  XPose* pose;
  XVec3* p3d;
  XVec2* p2d;
} Observation;


typedef struct
{
  XCamera* camera;
  XPose* pose;
  XVec3* p3d;
  Coordinate* p2d;
} Observation2;


// Range with pose and camera id
typedef struct 
{ 
  int id_pose,id_camera,begin,end; 
} XPoseRange ;

typedef struct {
  int size;
  XVec3 * pData;
} XVec3s;  // container, pData point to data buffer

typedef struct {
  int size;
  XVec2 * pData;
} XVec2s;  // container, pData point to data buffer

typedef struct {
  int size;
  IVec2 * pData;

} IVec2s;  // container, pData point to data buffer


typedef struct
{
  XVec3s vector_of_xvec3;
  IVec2s vector_of_ivec2;
  XPoseRange pose_range2[2];
  XStereo stereo;
#ifdef LOCALMAP
  // List of the 3D points indexes in the MAP
  int i3ds[MAX_NUM_FEATURE_IN_LOCAL_BASE];
#endif
} MatchedBase;



typedef struct 
{
  XVec3 *pose_angle_ptr;
  XVec3 *pose_translation_ptr;
  XVec3 *p3d_ptr;
} GeoInput;

typedef struct 
{
  XVec2 *p2d1_ptr;
  XVec2 *p2d2_ptr;
  XMat26 *mat261_ptr;
  XMat26 *mat262_ptr;
} GeoOutput;

// Range with pose and camera id
typedef  struct { float id_pose,id_camera,begin,end; } CPoseRange ;

typedef struct {
   Float curr;
   Float rho;
} Curr_Rho;

  
typedef struct
{
   int n_iter;
   Float error_before_opt;
   Curr_Rho *curr_rho;
} MapOutput;


typedef struct
{
   Float C;
   XVec6 delta;
   Float lambda;
   Float error_before_opt;
   Float curr;
   Float rho;
   Float alpha;
   int printAlpha;
} LocOutput;


typedef struct
{
  Vector p3ds;
  Vector descriptors;
  Range2 range2;
#ifdef LOCALMAP
  // List of the 3D points indexes in the MAP
  int i3ds[MAX_NUM_FEATURE_IN_LOCAL_BASE];
  // List of the 3D points inliers/used in the last localization
  int used_i3d[MAX_NUM_FEATURE_IN_LOCAL_BASE];
  // Number of element in used_i3d
  int used_i3d_size;
#endif
} LocalMap;


typedef struct
{
  XStereo xstereo;
  Vector vector_of_pose;
  Vector vector_of_p3d;
  Vector vector_of_p2d;
  Vector vector_of_observation_index;
} Map;


#ifdef LOCALMAP

// Structure list of the keyframe used to build the local map
typedef struct
{
  short data[NB_MAX_KF_IN_LOCALMAP];
  short size;
} LocalMapKeyframe;

// Structure list of the keyframes for one 3D point
typedef struct
{
  short data[NB_MAX_KF_PER_P3D];
  short size;
} ListOfKeyFrames;

#endif



typedef struct
{
  XStereo stereo;

  Vector poses;
  Vector p3ds;
  Vector p2ds;
  Vector descriptors;
  VectorInt indice2d3d;
  Vector ranges;
#ifdef LOCALMAP
  // The list of the keyframe used to build the local map
  LocalMapKeyframe local_map_kf;
  // The lists of index poses for each 3D points
  ListOfKeyFrames indice3d_to_vector_of_poses[TOTAL_NUM_P3D];
#endif
} XMap;


typedef struct 
{
  CFrame const* frame;
  XPose const* pose;
  XStereo const* stereo;
  Coordinate const* coordinates;
  Descriptor const* descriptors;
  Range2 const* feature_range;
  Matches matches[2];
  Matches rematches[2];
  int nb_candidate1;
  LocOutput loc_output1;
  int nb_candidate2;
  LocOutput loc_output2;
  LocalMap const* local_map;
  
  Vector inliers_3d;
  Range2 range_inliers_3d;

  int nb_inlier;
  int mapStartState;
} ProxyOutput;


typedef struct 
{
  int nb_inlier;
  int matches_size;
  int p3ds_size;
  int feature_size;
  XPose device_pose;
} BufferHolder;





typedef enum {
  STANDARD_MATCHING,
  CROSS_MATCHING,
  EPIPOLAR_MATCHING,
}MatchingMode;

typedef struct {
  float COMPARE_DESCRIPTOR_THRESHOLD;
  float MAPPING_ADD_P3D_THRESHOLD;
  float LOC_INLIER_THRESHOLD;
  int EPIPOLAR_DISTANCE;
  int PLL_CONFIG_MHZ;
  int IMU_CONFIG_FPS;
  int NON_MAX_SUPRESSION_RANGE;
  int COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES;
  int MATCH_RADIUS_LOC_1;
  int MATCH_RADIUS_LOC_2;
  int MAP_MATCHING_PREVIOUS_1;
  int MAP_MATCHING_PREVIOUS_2;
  int MAP_STEREO_MATCHING;
  int MINIMUM_INLIER_TO_BE_LOCALIZED;
  int legacy_padding1;
  uint16_t DETECTOR_TARGET_FEATURES;
  uint8_t DETECTOR_THRESHOLD_LOWER_LIMIT;
  uint8_t DETECTOR_THRESHOLD_UPPER_LIMIT;
  bool enable_6dof_post_filter;
  bool enable_posegraph;
  bool enable_circular_map;
  bool enable_map_correction;
  MatchingMode localization_matching_mode; 
  MatchingMode match3d2d_match1_matching_mode;
  MatchingMode match3d2d_match2_matching_mode;
  MatchingMode stereo_matching_mode;
  float INLIER_RATIO;  // 51, 0.1992 in Q8.8
  float MOVE_DIST;  // 15, in minimeter, MOVE_DIST_SQ = MOVE_DIST^2 / 10^6
  float MOVE_ANGLE_DEGREE;  // 57, in degree,  MOVE_ANGLE = MOVE_ANGLE_DEGREE / 180 * pi
  int DETECTOR_STEP_SIZE;  // 3
  float DETECTOR_TOLERANCE;  // 51, 0.0498 in Q8.10
  float POSE_FILTER_RATIO_ROTATION;  // 26; 0.101 in Q8.8
  float POSE_FILTER_RATIO_TRANSLATION;  //  5; 0.0195 in Q8.8
  int _NB_KF_IN_LOCALMAP;    //  5, maximum
  int _NUM_FEATURE_IN_LOCAL_BASE;  // 384*3, maximum of 384x5
  int _CIRCULAR_BUFFER_SIZE;        // 150
  int _NB_OPT_POSE;   // 4, maximum of 20
  int _NB_OPT_3D_RANGE_SIZE; // 10;, maximum of 40
  int _NB_ITERATION_MAP_CORRECTION;   // 2, maximum 10
  int _NB_ITERATION_COMPUTE_POSE; // 5. maximum 10 
  int CAMERA_TARGET_FPS;
  bool enable_power_saving;
  bool enable_uart;
  bool enable_vsc_feed;
  bool enable_gamma_correction;
  bool enable_simeon_filter; 
  bool enable_imu_extraplate;
  int ACC_RANGE;// only 2,4,8,16 G
}slam_config_t;


typedef struct 
{
  float COMPARE_DESCRIPTOR_THRESHOLD;  //0.8
  uint16_t MAPPING_ADD_P3D_THRESHOLD;  // 10
  uint16_t LOC_INLIER_THRESHOLD;  // 6
  uint16_t EPIPOLAR_DISTANCE;  // 0
  uint16_t PLL_CONFIG_MHZ;  // 0 EXTRA_CONFIG2
  uint16_t IMU_CONFIG_FPS;  // 0 EXTRA_CONFIG3
  uint16_t NON_MAX_SUPRESSION_RANGE_ACC_RANGE;  // 0
  uint8_t COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES;  //20
  uint8_t MATCH_RADIUS_LOC_1;  //30
  uint8_t MATCH_RADIUS_LOC_2;  //12
  uint8_t MAP_MATCHING_PREVIOUS_1;  //8
  uint8_t MAP_MATCHING_PREVIOUS_2;  //60
  uint8_t MAP_STEREO_MATCHING;  //80
  uint8_t MINIMUM_INLIER_TO_BE_LOCALIZED; //10
  uint8_t FEATURE_MASK; //0
  uint16_t DETECTOR_TARGET_FEATURES;  //250
  uint8_t DETECTOR_THRESHOLD_LOWER_LIMIT;  //7
  uint8_t DETECTOR_THRESHOLD_UPPER_LIMIT;  //120
  uint8_t INLIER_RATIO;  // 51, 0.1992 in Q8.8
  uint8_t MOVE_DIST;  // 15, in minimeter, MOVE_DIST_SQ = MOVE_DIST^2 / 10^6
  uint8_t MOVE_ANGLE_DEGREE;  // 57, in degree,  MOVE_ANGLE = MOVE_ANGLE_DEGREE / 180 * pi
  uint8_t DETECTOR_STEP_SIZE;  // 3
  uint8_t DETECTOR_TOLERANCE;  // 51, 0.0498 in Q8.10
  uint8_t POSE_FILTER_RATIO_ROTATION;  // 26; 0.101 in Q8.8
  uint8_t POSE_FILTER_RATIO_TRANSLATION;  //  5; 0.0195 in Q8.8
  uint8_t _NB_KF_IN_LOCALMAP;    //  5, maximum
  uint16_t _NUM_FEATURE_IN_LOCAL_BASE;  // 384*3, maximum of 384x5
  uint16_t _CIRCULAR_BUFFER_SIZE;        // 150
  uint8_t _NB_OPT_POSE;   // 4, maximum of 20
  uint8_t _NB_OPT_3D_RANGE_SIZE; // 10;, maximum of 40
  uint8_t _NB_ITERATION_MAP_CORRECTION;   // 2, maximum 10
  uint8_t _NB_ITERATION_COMPUTE_POSE; // 5. maximum 10
  uint8_t CAMERA_TARGET_FPS;
  uint8_t ENABLE_MASK;

}slam_config_hid_t;


#include "hidCommonMode.h"


struct SlamStruct
{
  XPose device_pose;
  float ts;
  XVec3 velocity;
  XMap map;
  LocalMap local_map;
  AreaThresholds frame_area_thresholds[2];
  Features c_current_features;
  Features local_map_features;
  MatchedBase matched_base2[2];
  FloatImage base;
  DeviceState device_state;
  ProxyOutput poutput;
};


typedef struct 
{
  int converge;
  float factor;
  int bias_init;
  int cnt;
} IMU_Initialization;

enum{
  NORMAL_OPER = 0,
  SLAM_START = 1,
  PRE_BLACK,
  POST_BLACK,
  FEED_VSC
};

typedef struct
{
  uint32_t signature;
	uint8_t debug_mode;
}shared_mem_dat_t;
