#pragma once

#include "hidapi.h"

#include <string>
#include <cassert>

#include "../../include/ccommon.h"
#include "../../include/hidCommonMode.h"

// 2 0xe3 0x22 0x01 0x01 0x01 0x?? 0x27 0x27

// disable_auto mode: 1 : auto exposure disabled, 0 auto exposure enabled
// gain : 0 -> 24 ?

// gain. P4 has a range 0 - 24, with 24 being the highest gain. 
//integration: Integration time. 66 is MSB, p5 is LSB . The maximum allowed value of P6_p5  is 10100. Normally, we keep p5 = 0, p6= 39
#ifdef MULTI_API_VERSION
#define WEIZ_HID (1u)
#endif

typedef struct imuRawAxisData_t
{
  int32_t x;
  int32_t y;
  int32_t z;
} imuRawAxisData_t;

typedef struct imuAxisData_t
{
  float x;
  float y;
  float z;
} imuAxisData_t;

typedef struct imuTempData_t
{
  float t;
} imuTempData_t;

typedef struct
{
  union {
    imuAxisData_t axisData;
    imuRawAxisData_t rawAxisData;
    imuTempData_t temperatureData;
  };
} mvImuData_t2;

typedef struct
{
  uint64_t tymeStamp;
  int32_t frameCount;
  mvImuData_t2 accelData;
  mvImuData_t2 gyroData;
} HIDoutData;

typedef struct
{
	uint8_t event_enable;
	uint8_t  event_type;
	uint8_t  type_state;  
}event_str_t;


typedef struct
{
  float quarternion[4];
  float extra[2];
} QE_float;

typedef struct 
{
  float x;
  float y;
  float z;
} imuAxisDAta_t;


typedef struct 
{
  uint32_t imu_time;
  uint32_t camera_time;
  int frameCount;
  QE_float q_e;
  imuAxisDAta_t linear_accel;
  float temperatureData;
  uint8_t imu_time_40;
  uint8_t reserved[3];
  event_str_t event_arg;
  uint8_t pktTyp;//0Xaa, 60th byte
}HIDoutData_new_rotation_m2_m3;

typedef struct 
{
  uint32_t imu_time;
  uint32_t camera_time;
  int frameCount;
  mvImuData_t2 accelData;
  mvImuData_t2 gyroData;
  mvImuData_t2 magnData;
  float temperatureData;
  uint8_t imu_time_40;
  uint8_t reserved[3];
  event_str_t event_arg;
  uint8_t pktTyp;//0Xaa, 60th byte
}HIDoutData_new_rotation;

typedef struct
{
  uint32_t imu_time;
  uint32_t camera_time;
  int32_t frameCount;
  mvImuData_t2 accelData;
  mvImuData_t2 gyroData;
  mvImuData_t2 magnData;
  float temperatureData;

  uint8_t imu_time_40;
  uint8_t reserved[3];
  event_str_t event_arg;
  uint8_t pktTyp;//0Xaa, 60th byte
} HIDoutData_new;

typedef struct vSLAMLatency_t
{
  unsigned int whole_id;
  unsigned int localModule_id;
  unsigned int whole_US;
  unsigned int detectorModule_US;
  unsigned int localModule_US;
  unsigned int localModule_localizedUS;
  unsigned int localModule_wrap1US;
  unsigned int localModule_wrap2US;
  unsigned int localModule_postUS;
  unsigned int shave1_id;
  unsigned int shave2_id;
  unsigned int localizationshave1;
  unsigned int localizationshave2;
} vSLAMLatency;

typedef struct BNO_DYN_CALIB_REQUEST_t
{
  uint8_t p1;
  uint8_t p2;
  uint8_t not_record;
} BNO_DYN_CALIB_REQUEST_;

typedef struct BNO_DYN_CALIB_RESPONSE_t
{
  uint8_t acc_status;
  uint8_t gyr_status;
  uint8_t mag_status;
  uint8_t lacc_status;
  uint8_t gra_status;
  uint8_t gameR_status;
} BNO_DYN_CALIB_RESPONSE_;

typedef enum
{
  DETECTOR_TIMESTAMP = 0,
  START_TIMESTAMP,
  PROJECT_TIMESTAMP,
  MATCH1_TIMESTAMP,
  COMP_POSE1_AND_FILTER_TIMESTAMP,
  MATCH2_TIMESTAMP,
  COMP_POSE2_AND_INLIER_TIMESTAMP,
  MAP_TIMESTAMP,

} modules;

struct ProcessingTime
{

  float values[8];
  float operator[](int i) const
  {
    assert(i < 8);
    return values[i];
  }
  float &operator[](int i)
  {
    assert(i < 8);
    return values[i];
  }
  void disp();
};

struct HidControl
{
  hid_device *connected_device;
#ifdef MULTI_API_VERSION
  bool use_api_new;
#endif
  HidControl();
  ~HidControl();

  bool isConnected();
  void imu_fusion(int p1,bool p2, short p3, short p4, short p5);
  void enable_6dof_imu(bool flip = 0);
  void auto_exposure();
  void convert_and_set_aec_adjustment(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, float p6);
  void set_aec_adjustment(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, int p6);
  //void enable_auto_exposure();
  void set_gain_exposure(int gain, int integration);
  //void set_aec_adjustment(int param1, int param2);
  void enable_debug_info(bool flip = 0);
  void set_cnn_source(int cnn_source);
  void disable_debug_info(bool flip = 0);
  void enable_frame(bool flip = 0);
  void get_calibration(DeviceCalibration &device_calibration);
  void get_processing_time(ProcessingTime &pt,float * swISP_dbg);
  void request_get_processing_time(bool enable);
  void send_calibration(void *data, int sz);
  void enable_detector_only(bool flip = 0);
  void send_slam_config(void *data, int sz);
  void send_config(std::string conf_file);
  void imu_request(void);
  void imu_new_request(void);
  void get_6dof(HIDInfoStruct &hid_compact);
  void reset_slam();
  int bno_dyn_calib(BNO_DYN_CALIB_REQUEST_t request,BNO_DYN_CALIB_RESPONSE_t &response);
  void save_config(std::string filename);
  void read_save_config_data_fromCougar2(std::string filename);
  void internal_grab_imu_new(float values[12]);
  void internal_grab_imu(float values[9]);
  int internal_grab_imu_raw(unsigned char raw_data[60]);
  int internal_grab_imu_new_raw(unsigned char raw_data[60]);
  void grab_imu(float values[9]);
  void grab_imu_new(float values[12]);
  int hid_cycles_debug(int nb[2]);
#if (WEIZ_HID == 1u)
  void request_6dof(bool enable,bool shortTypeUsed);
  void get_6dof_new(HIDInfoStruct &hid_compact,bool usedShortType);
  void get_6dof_new_short(HIDInfoSimpleStruct &hid_compact_short);
#endif

  void enable_usb_boot_loader();

  void enable_imu_fusion();
  void disable_imu_fusion();
  void set_imu_time_offset(uint8_t ms);

  std::string get_software_version();
  float get_software_number();
  std::string get_platform_version();
  #ifdef MULTI_API_VERSION
  void set_api_version(bool use_new_api);
  #endif
  void set_old_edge_command(bool old_command);


  void set_uvc_mode2(bool flip = 0);
  void set_uvc_mode5(bool flip = 0);
  void set_apriltag_cfg(uint8_t * cfg_buf,uint32_t len);
};

HidControl &unique_hid_control();
