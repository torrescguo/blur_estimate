#ifndef __HIDCOMMONMODE_H__
#define __HIDCOMMONMODE_H__
#pragma once


#define CHECK_VERBOSE_OFF 0
#define CHECK_VERBOSE_MINIMAL 1
#define CHECK_VERBOSE_ESSENTIAL 2
#define CHECK_VERBOSE_FULL 3

// DeviceState, use define instead of enum, for enum always take 4 bytes
typedef int DeviceState;
#define LOC_OFF 0
#define LOC_INITIALIZING 1
#define LOC_GOOD 2
#define LOC_BAD 3
#define LOC_LOST 4 
#define LOC_RECOVERING   5
 

//  MapState, use define instead of enum, for enum always take 4 bytes 
typedef int MapState;
#define MAP_OFF 0
#define MAP_READY 1
#define ADD_FEATURES 2
#define MATCH_3D2D 3 
#define MATCH_2D2D 4
#define STEREO_MATCH 5
#define INIT_MAP_CORRECTION 6
#define DERIVATIVE 7
#define LINEAR_SYSTEM 8 
#define LOCAL_MAP_UPDATE 9


#define u16 unsigned short

typedef enum {
  MAPPING_OFF,
  MAPPING_READY,
  MAPPING_STARTING,
  MAPPING_ON,
  MAPPING_DONE
} MAP_START_STATE;


#define Q_32_7      128.0f
#define Q_32_14     16384.0f 
#define one_million     1000.0f
#define Q_32_6         64.0f
//typedef enum { DEVICE_INITIALIZING, DEVICE_LOCALIZED, DEVICE_LOST } DeviceState;
//typedef enum { LOC_OFF, LOC_INITIALIZING, LOC_GOOD, LOC_BAD, LOC_LOST, LOC_RECOVERING } DeviceState;
//typedef enum { MAP_OFF, ADD_FEATURES, MATCH_3D2D, MATCH_2D2D, STEREO_MATCH, INIT_MAP_CORRECTION, DERIVATIVE, LINEAR_SYSTEM, LOCAL_MAP_UPDATE} MapState;



// Structure used to configure the operation mode of the SLAM code in movidius. The structure is sent from the PC to the Cougar board 
// through the HID channel
// It configures the content to be sent to the PC from Cougar board on the UVC channel and HID channel and how soon it it sent.
// It also configure the source of the video stream

typedef struct   // total of 10 bytes
{
  unsigned char HIDMode;
/*  
    HID channel Mode: 
      0: No HID info from Movidius
      1: HID filled with HIDInfoStruct, defined below
*/
  unsigned char  UVCMode;  
/*   
    UVC channel Mode:
      0: UVC has no Video Stream
      1: UVC has Video in NV12 format, Video stream in Y plane, nothing in UV plane
      2: UVC has Video in NV12 format, Video stream in Y plane, Demo information in UV plane, DemoStruct defined below
      3: UVC has Video in NV12 format, Video stream in Y plane, Debug information in UV plane, ebugStruct defined below
*/ 
  unsigned char   HIDRate;
/*
    HID frame rate:
      0:   100 fps
      1:   200 fps
      2:   400 fps
      3:   30 fps
      4:   60  fps
      5:   120  fps
      6:   240 fps

*/
  unsigned char  UVCRate;
/*
    UVC frame rate:
      0:   100 fps
      1:   30  fps
      2:   60  fps

*/
  unsigned char videoSource;
/*   
    Source of the video Stream
      0: Video stream from the the camera
      1: Video stream from the the DDR, preloaded from SD card
      2: Video stream from the SD card in real time.
*/
} OutputModeControl;

// DeviceCalibration is the structure for holding calibratoin information sent to cougar board. 
// The first 152 bytes are the same as XStereo structure


typedef struct
{
  // left camera
  float R1[9];
  float T1[3];
  float K1[7];

  // right camera
  float R2[9];
  float T2[3];
  float K2[7];

  // IMU bias
  float time_shift;
  float gyro_bias[3];
  float accel_bias[3];
} DeviceCalibration;

typedef struct                      // 76 bytes
{
  int device_state;                 //  4 byte 
  int map_state;                    //  4 byte  
  float time;                       //  4 bytes, time in second of the pose
  unsigned int frameNo;             //  4 bytes 
  float pose_rotation[9];           //  36 bytes
  float pose_translation[3];        //  12 bytes
  float pose_angle[3];              //  12 bytes

}  Motion;

typedef struct                      // 44 bytes 
{
  char device_state;                //  1 byte
  char map_state;                   //  1 byte 
  unsigned short frameNo;                    //  2 bytes 
  float time;                       //  4 bytes, time in second of the pose
  short pose_rotation[9];           //  9 x 2 = 18 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
  float pose_translation[3];        //  3 x 4 = 12 bytes
  short pose_angle[3];              //  3 x 2 = 6 bytes,   in Q16.7 format, with resolution of 0.0078 degree, range 0-360


} MotionCompact;


typedef struct             //  Total 26 bytes
{           
  short nb_features_left;            //  < 384
  short nb_features_right;           //  < 384
  short nb_matches_loc1_left;        //  < 384
  short nb_matches_loc1_right;       //  < 384
  short nb_matches_loc2_left;        //  < 384
  short nb_matches_loc2_right;       //  < 384
  short nb_inliers;                  //  < 384*2
  short nb_local_map_size;           //  < 10000
  short nb_map_keyframe;             //  < 1000
  int nb_map_features;               //  < 1000000
  int nb_map_3D;                     //  < 100000
} Metrics;


typedef struct              //  total 36 bytes 
{
   float accelData[3];      //  12 bytes
   float gyroData[3];       //  12 bytes
   float magData[3];    //  12 bytes
} IMUInfo;

typedef struct              //  total 12 bytes 
{
   short accelData[3];      //  6 bytes Accelaretion data, in Q16.7 format, with resolution of 0.0078 degree, range 0-360
   short gyroData[3];       //  6 bytes Gyroscope data, , in Q16.7 format, with resolution of 0.0078 degree, range 0-360
} IMUInfoCompact;

typedef struct              // 56 bytes;  sent through HID channel
{
  MotionCompact motion_data;       // 44 bytes
  IMUInfoCompact imu_data;         // 12 bytes
} HIDInfoStructDeprecated;

typedef struct
{
  int16_t quaternion[4];
  int16_t eular_angle[3];
  int16_t extra[2];
} QE;

typedef union
{
  int16_t rotation_matrix[9];
  QE q_e;
} Rotation_info;

typedef struct 
//60 bytes
{

uint32_t time;
// 4 bytes, time in micros second of the pose

int32_t pose_translation[3];
// 3 x 4 = 12 bytes, in Q32.14 format, with a resulotion of 0.000061 (0.061 mm),

// and range of [131071, -131072] meter;

Rotation_info rotation_info;// 9 x 2 = 18 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
//int16_t pose_rotation[9];

// 9 x 2 = 18 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)

uint16_t frameNo;
// 2 bytes, frame number

int16_t accelData[3];
// 6 bytes Accelaretion data, in Q16.7 format, with resolution of 0.0078 m/ss

int16_t gyroData[3];
// 6 bytes Gyroscope data, , in Q16.7 format, with resolution of 0.0078 degree, range 0-360

int16_t magData[3];
// 6 bytes Mag data, in Q16.7 format, with resolution of 0.0078 degree, range 0-360

uint8_t device_state;
// 1 byte

uint8_t confidence_level;
// 1 byte, confidence level, 0 - 100,

uint8_t reserved[4];
// 4 bytes, reserved for future use.

} HIDInfoStruct;


typedef struct 
//26 bytes
{
uint32_t time;
int32_t pose_translation[3];
// 3 x 4 = 12 bytes, in Q32.14 format, with a resulotion of 0.000061 (0.061 mm),
int16_t quaternion[4];;// 4 x 2 = 8 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
// 9 x 2 = 18 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
uint8_t device_state;
// 1 byte
uint8_t confidence_level;
// 1 byte, confidence level, 0 - 100,
} HIDInfoSimpleStruct;

typedef struct 
//20 bytes
{
uint8_t time;
uint8_t pose_translation1[3];
uint8_t pose_translation2[3];
uint8_t pose_translation3[3];
// 3 x 4 = 12 bytes, in Q32.14 format, with a resulotion of 0.000061 (0.061 mm),
int16_t quaternion[4];;// 4 x 2 = 8 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
// 9 x 2 = 18 bytes, in Q16.14 format, with resolution 0.000061, range [-2,2)
uint8_t device_state;
// 1 byte
uint8_t confidence_level;
// 1 byte, confidence level, 0 - 100,
} HIDInfoSimpleCompStruct;





typedef struct                        // total 174 bytes
{
  // check add_feature
  short nb_added_feature;             // 2 bytes
  XPose added_pose;                   // 9+3=12 bytes

  // check matching_the_last_features_with_previous
  MatchConfig match_config_1;         // 2 ints + 1 float: 12 bytes
  int nb_enable1_with_previous[2];    // 8 bytes
  int nb_projected_with_previous[2];  // 8 bytes
  int nb_match1_with_previous[2];     // 8 bytes
  int nb_tracked_with_previous[2];    // 8 bytes

  MatchConfig match_config_2;         // 2 ints + 1 float: 12 bytes
  int nb_enable2_with_previous[2];    // 8 bytes
  int nb_match2_with_previous[2];     // 8 bytes
  int nb_new3d_with_previous[2];      // 8 bytes

  MatchConfig match_config_3;         // 2 ints + 1 float: 12 bytes
  int nb_match_with_stereo_matching;  // 4 bytes
  int nb_new3d_with_stereo;           // 4 bytes

  int nb_observation;                 // 4 bytes
  int nb_opt_pose;                    // 4 bytes

  int n_iter;                         // 4 bytes
  int error;                          // 4 bytes
  float cr;                           // 4 bytes

  Range2 local_map_range;             // 4 ints: 16 bytes
  XVec3 first_3d, last_3d;            // 2x3 floats: 24 bytes

} DebugMap;

typedef struct {
	int size;
	Match pData[MAX_NUM_FEATURE];
} MatchesBuff;	// container, pData point to data buffer


typedef struct                                 // total of 77710 bytes
{
  Motion motion_data;                          // 60 bytes
  IMUInfo imu_data;                            // 36 bytes
  Metrics metrics_data;                        // 18 bytes

  XStereo stereo;                              // 152 bytes, defined in ccommon.h

  // check the detection+description
  Range2 coordinates_range2;                   // 4 integers = 16 bytes
  Coordinate coordinates[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*2*sizeof(int)= 6144 bytes
  Descriptor descriptors[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*16*sizeof(float)=49152 bytes
  
  // check the first matching
  int nb_candidate1;                           // 4 bytes
  MatchesBuff match1_left;                     // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes
  MatchesBuff match1_right;                    // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes

  // check the first compute_pose
  LocOutput loc_output1;                       // 13*sizeof(float|int) = 52 bytes

  // check the second matching
  int nb_candidate2;                           // 4 bytes
  MatchesBuff match2_left;                     // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes
  MatchesBuff match2_right;                    // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes

  // check the second compute pose
  LocOutput loc_output2;                       // 13*sizeof(float|int) = 52 bytes

  //check the inlier 3D points
  Range2 inliers_3d_range2;                    // 4 integers = 16 bytes
  XVec3 inliers_3d[MAX_NUM_FEATURE*2];         // MAX_NUM_FEATURES*2*3*sizeof(float) = 9216 bytes

  // check the mapping
  DebugMap debug_map;                          // 174 bytes

  int mapStartState; 
  float tmpBuff[1999];

} DebugStruct;

typedef struct              //  total 24 bytes 
{
   float accelData[3];      //  12 bytes
   float gyroData[3];       //  12 bytes
} IMUInfoMCCI;

typedef struct                                 // total of 77710 bytes
{
  Motion motion_data;                          // 60 bytes
  IMUInfoMCCI imu_data;                            // 36 bytes
  Metrics metrics_data;                        // 18 bytes

  XStereo stereo;                              // 152 bytes, defined in ccommon.h

  // check the detection+description
  Range2 coordinates_range2;                   // 4 integers = 16 bytes
  Coordinate coordinates[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*2*sizeof(int)= 6144 bytes
  Descriptor descriptors[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*16*sizeof(float)=49152 bytes
  
  // check the first matching
  int nb_candidate1;                           // 4 bytes
  MatchesBuff match1_left;                     // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes
  MatchesBuff match1_right;                    // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes

  // check the first compute_pose
  LocOutput loc_output1;                       // 13*sizeof(float|int) = 52 bytes

  // check the second matching
  int nb_candidate2;                           // 4 bytes
  MatchesBuff match2_left;                     // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes
  MatchesBuff match2_right;                    // (1+2*MAX_NUM_FEATURES)*sizeof(int) = 3076 bytes

  // check the second compute pose
  LocOutput loc_output2;                       // 13*sizeof(float|int) = 52 bytes

  //check the inlier 3D points
  Range2 inliers_3d_range2;                    // 4 integers = 16 bytes
  XVec3 inliers_3d[MAX_NUM_FEATURE*2];         // MAX_NUM_FEATURES*2*3*sizeof(float) = 9216 bytes

  // check the mapping
  DebugMap debug_map;                          // 174 bytes

  int mapStartState; 
  float tmpBuff[1999];

} DebugStructMCCI;
// 60+24+26 + 152 + 16+6144+49152 +4+3076+3076 + 52 + 4+3076+3076 + 52 + 9216+16 + 174 + 8000 = 85396 bytes;


typedef struct                                 // total of 55632 bytes
{
  Motion motion_data;                          // 76 bytes
  IMUInfo imu_data;                            // 36 bytes
  Metrics metrics_data;                        // 26 bytes

  XStereo stereo;                              // 152 bytes, defined in ccommon.h

  // check the detection+description
  Range2 coordinates_range2;                   // 4 integers = 16 bytes
  Coordinate coordinates[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*2*sizeof(int)= 3072 bytes
  Descriptor descriptors[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*16*sizeof(float)=24576 bytes
  
  float nb_p3ds;                                           // 4 bytes

  uint64_t camera_time;                         // 8 bytes

    //check the inlier 3D points
  Range2 inliers_3d_range2;                    // 4 integers = 16 bytes
  XVec3 inliers_3d[MAX_NUM_FEATURE*2];         // MAX_NUM_FEATURES*2*3*sizeof(float) = 9216 bytes
  
  float p3ds[MAX_NUM_FEATURE_IN_LOCAL_BASE*3];   //  384*3*3 = 384*9 so 13824 bytes
  
  uint32_t i3ds[MAX_NUM_FEATURE_IN_LOCAL_BASE]; // 384*3 int so 4608 bytes


} MixedModeStruct;
// 76 + 36 + 28 + 152 + 16 + 3072 + 24576 + 4 + 8 + 16 + 9216 + 13824 + 4608 = 55632

#define MixedModeStructSize sizeof(MixedModeStruct)

#define DebugStructSize sizeof(DebugStruct) // = 85452

typedef struct                                 // total of 77710 bytes
{
  Motion motion_data;                          // 60 bytes
  IMUInfo imu_data;                            // 24 bytes
  Metrics metrics_data;                        // 18 bytes

  XStereo stereo;                              // 152 bytes, defined in ccommon.h

  // check the detection+description
  Range2 coordinates_range2;                   // 4 integers = 16 bytes
  Coordinate coordinates[MAX_NUM_FEATURE*2];   // MAX_NUM_FEATURES*2*2*sizeof(int)= 6144 bytes

  //check the inlier 3D points
  Range2 inliers_3d_range2;                    // 4 integers = 16 bytes
  XVec3* inliers_3d[MAX_NUM_FEATURE*2];        // MAX_NUM_FEATURES*2*3*sizeof(float) = 9216 bytes

} DemoStruct;


// 60 + 24 + 18 + 152 + 6144 + 4 + 9216 +16 = 15634 bytes;


#define DemoStructSize 16000


struct ConfigMotion
{
  short interpolation;// 0: default(?) 1: 100 hz, 2: 200 hz , 3: 400 hz, 4 800hz ...
  short requested_output;
  // 0: device state + 6dof
  // 1: device state + verification
  // 2: device state + calibration
  // 3: 6_dof_with_2D3D_display_information
};

#endif



