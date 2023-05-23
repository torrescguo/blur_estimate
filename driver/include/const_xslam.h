#pragma once
#define u16 unsigned short
#define s16 short
#define INT16 short
#define s32 int


#if defined(STEREO_1280X800)
#define MAX_WIDTH				1280
#define MAX_HEIGHT				800
#elif defined(STEREO_320X200)
#define MAX_WIDTH				320
#define MAX_HEIGHT				200
#else
#define MAX_WIDTH				640
#define MAX_HEIGHT				400
#endif

#define MAX_DATA_SIZE_MATCHING	             307200
#define MAX_DATA_SIZE_DESCRIPTOR	     307200*3
#define MAX_DATA_SIZE_COMBINED  	     307200*3

#define MAX_RGB_WIDTH				640
#ifdef ENABLE_HANDSKELETON
#define MAX_RGB_HEIGHT				480
#else
#define MAX_RGB_HEIGHT				480
#endif
#define MAX_TOF_WIDTH				224
#define MAX_TOF_HEIGHT				172

//#define IMPROVED

#ifdef IMPROVED
  #define CURRENT_FEATURE_ARE_ORDERED
  #define IMPROVED_MATCHING
  #define IMPROVED_DETECTOR
#endif



//#define MAX_NUM_FEATURE			512
#define MAX_NUM_FEATURE     384


#ifdef IMPROVED
  #define FEATURES_UPPER_LIMIT 600
  #define FEATURES_LOWER_LIMIT 400
#else
  #define FEATURES_UPPER_LIMIT 668
  #define FEATURES_LOWER_LIMIT 568
#endif

#define MAX_KEY_FRAME 500
#define TOTAL_NUM_FEATURE (MAX_NUM_FEATURE * MAX_KEY_FRAME * 2)
#define TOTAL_NUM_P3D (MAX_NUM_FEATURE * MAX_KEY_FRAME / 2)


#define SERIALIZE_MAPPING
#define FIX_ DTHRESH

//#define USE_UNIT_TEST_CONFIG

#define MINIMAL_MAP_CORRECTION

#define MAP_VECT_NOT_ON_SHAVE

#ifdef USE_UNIT_TEST_CONFIG
  #define NB_MAX_OPT_POSE 20
#else
  #define NB_MAX_OPT_POSE 1
#endif

#define NB_OPT_3D_RANGE_SIZE 6
#define NB_MAX_OPT_P3D MAX_NUM_FEATURE*NB_OPT_3D_RANGE_SIZE


#define NB_MAX_OBS NB_OPT_3D_RANGE_SIZE*MAX_NUM_FEATURE
#define NB_MAX_OBS_PER_SHAVE (NB_MAX_OBS * 3 / 8)


#define FEATURE_SIZE          7   // must be odd
#define DESCRIPTOR_LENGTH		16
#define _13x13 169
#define DEBUG_PRINTF

//#define MOVI_C
#define WRITE_BUFFER_SAVE_MEMORY

#define NB_ITERATION 10

#if 0
//#ifdef __MOVICOMPILE__
#define double_64 long double
#else
#define double_64 double
#endif

#define Float float

#ifdef MOVI_C
//#define USE_RADIX_SORT

#define VECTORIZED

#else
#define restrict    

#endif

//#define USE_MAPPING
#define USE_COMB_LOCALIZE
//#define USE_LOCALIZATION

#define HEAP_STAT


//#define SKIP_MAP_CORRECTION


//#define COMPUTE_POSE_DIVIDE

//#define FAKE


//this variable enables the improved LOCALMAP:
// the keyframes used to build the local map are chosen from the 3D points seen in the last frames
// thoses 3D points are mostly seen in some keyframes
// thoses keyframes are used to build the LOCALMAP
//#define LOCALMAP

#define MAX_NUM_FEATURE_IN_LOCAL_BASE 384*3

#ifdef LOCALMAP

  #define LOCALMAP_SHARED_DESCRIPTORS

// the local map is build using NB_MAX_KF_IN_LOCALMAP keyframes
  #define NB_MAX_KF_IN_LOCALMAP 2

// we consider that each 3D points can be seen in NB_MAX_KF_PER_P3D
  #define NB_MAX_KF_PER_P3D 10

  #define MAX_NUM_FEATURE_IN_LOCAL_BASE 384
  
#endif

#define RELOC_TWO_PASSES

//#define USE_EIGEN

