#include "hid_control.hpp"
#include "../../include/config.h"
#include "../../include/ccommon.h"
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <math.h>
#include <cassert>
#include <iomanip>
#include <iterator>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
using namespace std;

#define SET_BITX(var, bit_no) (var=var|(1<<bit_no))
#define UNSET_BITX(var, bit_no) (var=var&(~(1<<bit_no)))
#define ENCODE_Q8_8(int_val) (((int)(((float)(int_val))*(float)(1<<8)) + 0.5f))
#define ENCODE_Q8_10(int_val) (((int)(((float)(int_val))*(float)(1<<10)) + 0.5f))
#define DECODE_Q8_8(int_val) ((float)(((float)(int_val)) / (float)(1<<8)))
#define DECODE_Q8_10(int_val) ((float)(((float)(int_val))/(float)(1<<10)))
#define GET_14BITS_FROME_INT16(int_val) ((int)(int_val & 0x3fff))
#define GET_15_16BITS_FROM_INT16(int_val) ((int)((int_val & 0xc000)>>14))
#define TEMP_CONFIG_DATA_FLAG_LOCATION (62)

#define CONFIG_ELEMENT_SIZE 50
#define SINGLE_CONFIG_ELEMENT_SIZE 45
static const char config_element_name [][SINGLE_CONFIG_ELEMENT_SIZE]={
	"COMPARE_DESCRIPTOR_THRESHOLD",
	"MAPPING_ADD_P3D_THRESHOLD",
	"LOC_INLIER_THRESHOLD",
	"EPIPOLAR_DISTANCE",
	"PLL_CONFIG_MHZ",
	"IMU_CONFIG_FPS",
	"NON_MAX_SUPRESSION_RANGE",
	"COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES",
	"MATCH_RADIUS_LOC_1",
	"MATCH_RADIUS_LOC_2",
	"MAP_MATCHING_PREVIOUS_1",
	"MAP_MATCHING_PREVIOUS_2",
	"MAP_STEREO_MATCHING",
	"MINIMUM_INLIER_TO_BE_LOCALIZED",
	"MAP_CORRECTION_DISABLE",
	"DETECTOR_TARGET_FEATURES",
	"DETECTOR_THRESHOLD_LOWER_LIMIT",
	"DETECTOR_THRESHOLD_UPPER_LIMIT",
	"ENABLE_6_DOF_POST_FILTER",
	"ENABLE_POSEGRAPH",
	"ENABLE_CIRCULAR_MAP",
	"ENABLE_MAP_CORRECTION",
	"LOCALIZATION_MATCHING_MODE",
	"ENABLE_CROSS_MATCHING_MAP",
	"ENABLE_EPIPOLAR_MATCH3d2d_match2",
	"ENABLE_EPIPOLAR_STEREO_MATCH",
	"INLIER_RATIO",
	"MOVE_DIST",
	"MOVE_ANGLE_DEGREE",
	"DETECTOR_STEP_SIZE",
	"DETECTOR_TOLERANCE",
	"POSE_FILTER_RATIO_ROTATION",
	"POSE_FILTER_RATIO_TRANSLATION",
	"_NB_KF_IN_LOCALMAP",
	"_NUM_FEATURE_IN_LOCAL_BASE",
	"_CIRCULAR_BUFFER_SIZE",
	"_NB_OPT_POSE",
	"_NB_OPT_3D_RANGE_SIZE",
	"_NB_ITERATION_MAP_CORRECTION",
	"_NB_ITERATION_COMPUTE_POSE",
	"CAMERA_TARGET_FPS",
	"enable_power_saving",
	"enable_uart",
	"enable_vsc_feed",
	"enable_gamma_correction",
	"enable_simeon_filter",
	"enable_imu_extrapolate",
  "enable_lyapunov_lost",
  "ACC_RANGE",
	"TEMPORARY_CONFIG_DATA",
};
#define PASTER(x) #x
#define COMBINE(x) "ERROR: CONFIG_ELEMENT_SIZE SHOULD BE x"
static_assert(sizeof(config_element_name)/sizeof(*config_element_name) == CONFIG_ELEMENT_SIZE, COMBINE(""PASTER(SINGLE_CONFIG_ELEMENT_SIZE)"")); 
static int counter = 0;

bool judge_conf_elem_name(char input_str[SINGLE_CONFIG_ELEMENT_SIZE])
{
	bool ret= strcmp(input_str,config_element_name[counter]);
//  printf("%s , %s\n", input_str,config_element_name[counter]);
  counter++;
	return !ret;
}
// typedef enum {
//     DETECTOR_TIMESTAMP = 0,
//     START_TIMESTAMP,
//     PROJECT_TIMESTAMP,
//     MATCH1_TIMESTAMP,
//     COMP_POSE1_AND_FILTER_TIMESTAMP,
//     MATCH2_TIMESTAMP,
//     COMP_POSE2_AND_INLIER_TIMESTAMP,
//     MAP_TIMESTAMP,
// } modules;
/*************Bit Tools********************/
int getBitN(int in,int bitN)
{
  return ((in & (0x00000001<<bitN)) >>bitN); 
}
/******************************************/
int returnACCRangeInG(int acc_bitsFromHid)
{
  switch(acc_bitsFromHid)
  {
    case 0:return 2;break;
    case 1:return 4;break;
    case 2:return 8;break;
    case 3:return 16;break;
    default:return 2;break;
  }
  return 2;
}
void configACCRangeInHid(uint16_t inputRange,uint16_t *hid_var)
{
  switch(inputRange)
  {
    case 2:UNSET_BITX(*hid_var,14);UNSET_BITX(*hid_var,15);break;
    case 4:SET_BITX(*hid_var,14);UNSET_BITX(*hid_var,15);break;
    case 8:SET_BITX(*hid_var,15);UNSET_BITX(*hid_var,14);break;
    case 16:SET_BITX(*hid_var,14);SET_BITX(*hid_var,15);break;
    default:UNSET_BITX(*hid_var,14);UNSET_BITX(*hid_var,15);break;
  }
}
uint16_t setNon_Max_SupInHid(uint16_t non_max_sup_range)
{
  return (uint16_t)(non_max_sup_range & 0xcfff);
}
/*************Config Decode****************/
static const int HID_BUF_OFFSET = 3;
static bool g_use_new_api = false;
static bool g_check_version = false;
static bool use_old_edge_cmd = false;
void ProcessingTime::disp()
{
  std::cout << " DETECTOR_TIMESTAMP: " << values[DETECTOR_TIMESTAMP] << std::endl;
  std::cout << " START_TIMESTAMP: " << values[START_TIMESTAMP] << std::endl;
  std::cout << " PROJECT_TIMESTAMP: " << values[PROJECT_TIMESTAMP] << std::endl;
  std::cout << " MATCH1_TIMESTAMP: " << values[MATCH1_TIMESTAMP] << std::endl;
  std::cout << " COMP_POSE1_AND_FILTER_TIMESTAMP: " << values[COMP_POSE1_AND_FILTER_TIMESTAMP] << std::endl;
  std::cout << " MATCH2_TIMESTAMP: " << values[MATCH2_TIMESTAMP] << std::endl;
  std::cout << " COMP_POSE2_AND_INLIER_TIMESTAMP: " << values[COMP_POSE2_AND_INLIER_TIMESTAMP] << std::endl;
  std::cout << " MAP_TIMESTAMP: " << values[MAP_TIMESTAMP] << std::endl;
}

// 2 0xe3 0x22 0x01 0x01 0x01 0x?? 0x27 0x27

// disable_auto mode: 1 : auto exposure disabled, 0 auto exposure enabled
// gain : 0 -> 24 ?

// gain. P4 has a range 0 - 24, with 24 being the highest gain. 
//integration: Integration time. 66 is MSB, p5 is LSB . The maximum allowed value of P6_p5  is 10100. Normally, we keep p5 = 0, p6= 39

template <int Size>
void _send_mode_info(hid_device *connected_device, uint8_t info[Size])
{
  unsigned char buffer[63];
  std::memset(buffer, 0, 63);
  std::memcpy((uint8_t *)buffer, (uint8_t *)info, Size);
  unsigned char buf[256];
#ifdef MULTI_API_VERSION
 if(g_use_new_api)
 {
    hid_write(connected_device, (const unsigned char *)buffer, 63);
    buf[0] = 0x1;
    hid_weiz_read(connected_device, buf, sizeof(buf));
 }
 else
#endif
 {
    hid_read(connected_device, buf, sizeof(buf));
    hid_write(connected_device, (const unsigned char *)buffer, 63);
 }
}

HidControl::HidControl()
{
  static int count = 0;
  count++;
  assert(count == 1);
  struct hid_device_info *devs, *cur_dev;
  connected_device = NULL;
  #ifdef MULTI_API_VERSION
    use_api_new = false;
  #endif
  if (hid_init())
    throw *this;

  int cnt = 0;

  while (1)
  {
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;

    while (cur_dev)
    {

      // printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
      // printf("\n");
      // printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
      // printf("  Product:      %ls\n", cur_dev->product_string);
      // printf("  Release:      %hx\n", cur_dev->release_number);
      // printf("  Interface:    %d\n",  cur_dev->interface_number);
      // printf("\n");

      if (cur_dev->product_id == 62472)
      {

        // if(cnt!=0)
        //   std::cout << "\r\e[A";

        connected_device = hid_open(cur_dev->vendor_id, cur_dev->product_id, cur_dev->serial_number);

        if (connected_device == NULL)
        {
          printf("\033[31m HidControl: Device found but the connection failed. Note that you need to be super-user !\033[0m\n");
          throw *this;
        }
        //hid_set_nonblocking(connected_device, 1);
        hid_set_nonblocking(connected_device, 0);

        hid_free_enumeration(devs);
        goto end;
      }
      cur_dev = cur_dev->next;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (!cnt++)
      std::cout << "\033[31m Warning: Is the device plugged ?\033[0m\n"
                << " Retring ";

    std::cout << "." << std::flush;
  }

  hid_free_enumeration(devs);

end:

  if (connected_device == NULL)
    throw *this;
}

HidControl::~HidControl()
{
  if (connected_device)
    hid_close(connected_device);
  hid_exit();
}

bool HidControl::isConnected()
{

  // struct hid_device_info *devs, *cur_dev;
  // hid_device * connected_device = NULL;
  if (hid_init())
  {
      return true;
  }
  else false;
  // int cnt = 0;
  // // devs = hid_enumerate(0x0, 0x0);
  // cur_dev = devs;
  // // hid_set_nonblocking(connected_device, 0);
  // // hid_free_enumeration(devs);
  // if(cur_dev==NULL)
  //     return false;
  // else return true;
 
  // if (connected_device == NULL)
  //   return false;
  // else return true;

}
void HidControl::set_aec_adjustment(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, int p6)
{
  uint8_t mode_info[9] = {2, 0xE3, 0x22, p1, p2, p3, p4, p5, p6};

  std::cout << " Auto Exposure Control (sent via HID): ";
  for (int i = 0; i < 9; ++i)
    std::cout << (int)mode_info[i] << ",";
  std::cout << '\b' << std::endl;

  _send_mode_info<9>(connected_device, mode_info);
  /*
p3: the high brightness targe, range of 0-40, with 40 being the highest brightness. Default of 11
p4: the low brightness target, range of 0-40, with 40 being the highest brightness. Default of 11
p5: the maximum integration time, range 0 – 100, 0 means using the system default, 1-100 corresponds to integration time of 0.1 ms to 10.0 ms. Default 50.
p6: damping factor, range of 0-50, default of 10. The damping factor is used to control the speed of convergence. The higher the damping factor, the faster the convergence. Too high of damping factor may result in oscilation.
2 0xE3 0x22 0 0 11 11 50 10 // enable auto exposure, with low and hight target brightness of 11 and damping factor of 10, and 5 ms maximum integration time.
2 0xe3 0x22 1 0 0 8 0 39 // disable auto exposure, gain of 8.0, and integration of 9984
*/
}

void HidControl::convert_and_set_aec_adjustment(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, float p6)
{
  int p3i = double(p3) * 40. / 255.;
  int p4i = double(p4) * 40. / 255.;
  int p6i = int(p6 / 0.02);
  set_aec_adjustment(p1, p2, p3i, p4i, p5, p6i);
}

void _set_gain_exposure(hid_device *connected_device, int gain, int integration)
{
  unsigned char buffer[63];
  std::memset(buffer, 0, 63);
  uint8_t mode_info[9] = {2, 0xe3, 0x22, 1, 0, 0, gain, 0, integration};

  std::cout << " Manual Exposure Control (sent via HID): ";
  for (int i = 0; i < 9; ++i)
    std::cout << (int)mode_info[i] << ",";
  std::cout << '\b' << std::endl;

  _send_mode_info<9>(connected_device, mode_info);
}

// void HidControl::enable_auto_exposure()
// {
//   //std::cout << " Set manual exposure 11 10 " << std::endl;
//   _set_gain_exposure(connected_device,0,11,10);
// }

void HidControl::imu_fusion(int p1,bool p2, short p3, short p4, short p5)
{
  p2 = int (p2);
  std::cout << "HID IMU FUSION : " << 2 << " " << 0xE4 << " " << 0x23 << " " << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 << std::endl;
  uint8_t * msu8 = (uint8_t*) (&p3);
  uint8_t * predu8 = (uint8_t*) (&p4);
  uint8_t * timer8 = (uint8_t*) (&p5);
  uint8_t mode_info[11] = {2, 0xE4, 0x23, p1, p2, msu8[0], msu8[1], predu8[0], predu8[1], timer8[0], timer8[1]};
  _send_mode_info<11>(connected_device, mode_info);
  //only for log
  std::string LOG_IMU_MODE="Enable";//Default IMU MODE IS 1 
  std::string LOG_IMU_Sync_Gyro="Disable";
  if(p2) string LOG_IMU_Sync_Gyro="Enable";
  switch(p1)
  {
    case 0:LOG_IMU_MODE="Disable";break;
    case 1:LOG_IMU_MODE="Enable";break;
    case 2:LOG_IMU_MODE="BNO080 Rotation 200hz";break;
    case 3:LOG_IMU_MODE="BNO080 Gryo Rotation 1000hz";break;
    default:break;
  }
  std::cout<< "Config IMU FUSION: IMU-MODE: "<<LOG_IMU_MODE<<" IMU-Sync-Gyro: "<<LOG_IMU_Sync_Gyro<<" Delay: "<<p3<<" ms "<<"Prediction:"<<p4<<" ms "<<"timer:"<<p5<<" ms "<<std::endl;
}

void HidControl::set_gain_exposure(int gain, int integration)
{
  _set_gain_exposure(connected_device, gain, integration);
}

void HidControl::send_calibration(void *data, int sz)
{
  std::cout << " SEND CALIBRATION !!! " << std::endl;
  unsigned char write_buffer[63];
  uint8_t calib_id[3] = {2, 0xCA, 0x11};
  std::memcpy((uint8_t *)write_buffer, calib_id, 3);
  unsigned char read_buf[256];

  for (int i = 0; i < 3; ++i)
  {
#ifdef MULTI_API_VERSION
    if(use_api_new)//need to confirm
    {
      std::memcpy(write_buffer + 3, data + 60 * i, 60);
      int res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
      res = hid_weiz_read(connected_device, read_buf, sizeof(read_buf));
    }
    else
#endif
    {
      int res = hid_read(connected_device, read_buf, sizeof(read_buf));
      std::memcpy(write_buffer + 3, data + 60 * i, 60);
      res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
    }
  }
}

void HidControl::send_slam_config(void *data, int sz)
{
  std::cout << " size of slam config : " << sz << std::endl;
  unsigned char write_buffer[63];
  uint8_t config_id[3] = {2, 0x1B, 0x97};
  std::memcpy((uint8_t *)write_buffer, config_id, 3);
  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    std::memcpy(write_buffer + 3, data, 24);
    int res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
    res = hid_weiz_read(connected_device, read_buf, sizeof(read_buf));
  }
 else
#endif
  {
    int res = hid_read(connected_device, read_buf, sizeof(read_buf));
    std::memcpy(write_buffer + 3, data, 24);
    res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
  }
}

static int read_slam_config_from_file(slam_config_hid_t *outData,std::string selectFile,int * temp_flag)
{
  int ret=0;
  FILE *fp;
  fp = fopen((const char*)selectFile.data(), "r");
  if (fp == NULL)
  { std::cout << "slam config slam_calibration_data.txt file not found!!!" << std::endl;return 1; }
  char input_variable_name[SINGLE_CONFIG_ELEMENT_SIZE];
  float COMPARE_DESCRIPTOR_THRESHOLD;
	(void)(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &COMPARE_DESCRIPTOR_THRESHOLD);
  outData->COMPARE_DESCRIPTOR_THRESHOLD = COMPARE_DESCRIPTOR_THRESHOLD;
  uint16_t MAPPING_ADD_P3D_THRESHOLD;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MAPPING_ADD_P3D_THRESHOLD);
  outData->MAPPING_ADD_P3D_THRESHOLD = MAPPING_ADD_P3D_THRESHOLD;
  uint16_t LOC_INLIER_THRESHOLD;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &LOC_INLIER_THRESHOLD);
  outData->LOC_INLIER_THRESHOLD = LOC_INLIER_THRESHOLD;
  float EPIPOLAR_DISTANCE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &EPIPOLAR_DISTANCE);
  outData->EPIPOLAR_DISTANCE = (uint16_t)(EPIPOLAR_DISTANCE*(float)(1 << 14));
  uint16_t PLL_CONFIG_MHZ;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &PLL_CONFIG_MHZ);
  outData->PLL_CONFIG_MHZ = PLL_CONFIG_MHZ;
  uint16_t IMU_CONFIG_FPS;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &IMU_CONFIG_FPS);
  outData->IMU_CONFIG_FPS = IMU_CONFIG_FPS;
  uint16_t NON_MAX_SUPRESSION_RANGE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &NON_MAX_SUPRESSION_RANGE);
  outData->NON_MAX_SUPRESSION_RANGE_ACC_RANGE = setNon_Max_SupInHid(NON_MAX_SUPRESSION_RANGE);
  uint8_t COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES);
  outData->COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES = COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES;
  uint8_t MATCH_RADIUS_LOC_1;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MATCH_RADIUS_LOC_1);
  outData->MATCH_RADIUS_LOC_1 = MATCH_RADIUS_LOC_1;
  uint8_t MATCH_RADIUS_LOC_2;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MATCH_RADIUS_LOC_2);
  outData->MATCH_RADIUS_LOC_2 = MATCH_RADIUS_LOC_2;
  uint8_t MAP_MATCHING_PREVIOUS_1;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MAP_MATCHING_PREVIOUS_1);
  outData->MAP_MATCHING_PREVIOUS_1 = MAP_MATCHING_PREVIOUS_1;
  uint8_t MAP_MATCHING_PREVIOUS_2;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MAP_MATCHING_PREVIOUS_2);
  outData->MAP_MATCHING_PREVIOUS_2 = MAP_MATCHING_PREVIOUS_2;
  uint8_t MAP_STEREO_MATCHING;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MAP_STEREO_MATCHING);
  outData->MAP_STEREO_MATCHING = MAP_STEREO_MATCHING;
  uint8_t MINIMUM_INLIER_TO_BE_LOCALIZED;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MINIMUM_INLIER_TO_BE_LOCALIZED);
  outData->MINIMUM_INLIER_TO_BE_LOCALIZED = MINIMUM_INLIER_TO_BE_LOCALIZED;
  uint8_t MAP_CORRECTION_DISABLE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MAP_CORRECTION_DISABLE);
  //outData->MAP_CORRECTION_DISABLE = MAP_CORRECTION_DISABLE;
  uint16_t DETECTOR_TARGET_FEATURES;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &DETECTOR_TARGET_FEATURES);
  outData->DETECTOR_TARGET_FEATURES = DETECTOR_TARGET_FEATURES;
  uint8_t DETECTOR_THRESHOLD_LOWER_LIMIT;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &DETECTOR_THRESHOLD_LOWER_LIMIT);
  outData->DETECTOR_THRESHOLD_LOWER_LIMIT = DETECTOR_THRESHOLD_LOWER_LIMIT;
  uint8_t DETECTOR_THRESHOLD_UPPER_LIMIT;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &DETECTOR_THRESHOLD_UPPER_LIMIT);
  outData->DETECTOR_THRESHOLD_UPPER_LIMIT = DETECTOR_THRESHOLD_UPPER_LIMIT;
  uint8_t FEATURE_MASK = 0;
  uint8_t ENABLE_6_DOF_POST_FILTER;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_6_DOF_POST_FILTER);
  if(ENABLE_6_DOF_POST_FILTER) SET_BITX(FEATURE_MASK, 0);
  uint8_t ENABLE_POSEGRAPH;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_POSEGRAPH);
  if(ENABLE_POSEGRAPH) SET_BITX(FEATURE_MASK, 1);
  uint8_t ENABLE_CIRCULAR_MAP;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_CIRCULAR_MAP);
  if(ENABLE_CIRCULAR_MAP) SET_BITX(FEATURE_MASK, 2);
  uint8_t ENABLE_MAP_CORRECTION;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_MAP_CORRECTION);
  if(ENABLE_MAP_CORRECTION) SET_BITX(FEATURE_MASK, 3);
  uint8_t LOCALIZATION_MATCHING_MODE; 
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &LOCALIZATION_MATCHING_MODE);
  if(LOCALIZATION_MATCHING_MODE) SET_BITX(FEATURE_MASK, 4);
  uint8_t ENABLE_CROSS_MATCHING_MAP; 
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_CROSS_MATCHING_MAP);
  if(ENABLE_CROSS_MATCHING_MAP) SET_BITX(FEATURE_MASK, 5);
  uint8_t ENABLE_EPIPOLAR_MATCH3d2d_match2; 
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_EPIPOLAR_MATCH3d2d_match2);
  if(ENABLE_EPIPOLAR_MATCH3d2d_match2) SET_BITX(FEATURE_MASK, 6);
  uint8_t ENABLE_EPIPOLAR_STEREO_MATCH; 
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ENABLE_EPIPOLAR_STEREO_MATCH);
  if(ENABLE_EPIPOLAR_STEREO_MATCH) SET_BITX(FEATURE_MASK, 7);
  outData->FEATURE_MASK = FEATURE_MASK;
  float INLIER_RATIO;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &INLIER_RATIO);
  outData->INLIER_RATIO = ENCODE_Q8_8(INLIER_RATIO);
  uint8_t MOVE_DIST;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MOVE_DIST);
  outData->MOVE_DIST = MOVE_DIST;
  uint8_t MOVE_ANGLE_DEGREE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &MOVE_ANGLE_DEGREE);
  outData->MOVE_ANGLE_DEGREE = MOVE_ANGLE_DEGREE;
  uint8_t DETECTOR_STEP_SIZE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &DETECTOR_STEP_SIZE);
  outData->DETECTOR_STEP_SIZE = DETECTOR_STEP_SIZE;
  float DETECTOR_TOLERANCE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &DETECTOR_TOLERANCE);
  outData->DETECTOR_TOLERANCE = ENCODE_Q8_10(DETECTOR_TOLERANCE);
  float POSE_FILTER_RATIO_ROTATION;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &POSE_FILTER_RATIO_ROTATION);
  outData->POSE_FILTER_RATIO_ROTATION = ENCODE_Q8_8(POSE_FILTER_RATIO_ROTATION);
  float POSE_FILTER_RATIO_TRANSLATION;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%f", &POSE_FILTER_RATIO_TRANSLATION);
  outData->POSE_FILTER_RATIO_TRANSLATION = ENCODE_Q8_8(POSE_FILTER_RATIO_TRANSLATION);
  uint8_t _NB_KF_IN_LOCALMAP;
 	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NB_KF_IN_LOCALMAP);
  outData->_NB_KF_IN_LOCALMAP = _NB_KF_IN_LOCALMAP;
  uint16_t _NUM_FEATURE_IN_LOCAL_BASE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NUM_FEATURE_IN_LOCAL_BASE);
  outData->_NUM_FEATURE_IN_LOCAL_BASE = _NUM_FEATURE_IN_LOCAL_BASE;
  uint16_t _CIRCULAR_BUFFER_SIZE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_CIRCULAR_BUFFER_SIZE);
  outData->_CIRCULAR_BUFFER_SIZE = _CIRCULAR_BUFFER_SIZE;
  uint8_t _NB_OPT_POSE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NB_OPT_POSE);
  outData->_NB_OPT_POSE = _NB_OPT_POSE;
  uint8_t _NB_OPT_3D_RANGE_SIZE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NB_OPT_3D_RANGE_SIZE);
  outData->_NB_OPT_3D_RANGE_SIZE = _NB_OPT_3D_RANGE_SIZE;
  uint8_t _NB_ITERATION_MAP_CORRECTION;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NB_ITERATION_MAP_CORRECTION);
  outData->_NB_ITERATION_MAP_CORRECTION = _NB_ITERATION_MAP_CORRECTION;
  uint8_t _NB_ITERATION_COMPUTE_POSE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &_NB_ITERATION_COMPUTE_POSE);
  outData->_NB_ITERATION_COMPUTE_POSE = _NB_ITERATION_COMPUTE_POSE;
  uint8_t CAMERA_TARGET_FPS;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &CAMERA_TARGET_FPS);
  outData->CAMERA_TARGET_FPS = CAMERA_TARGET_FPS;
  uint8_t ENABLE_MASK = 0;
  uint8_t enable_power_saving = 0;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &enable_power_saving);
  if(enable_power_saving) SET_BITX(ENABLE_MASK, 0);
  uint8_t enable_uart = 0;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &enable_uart);
  if(enable_uart) SET_BITX(ENABLE_MASK, 1);
  uint8_t enable_vsc_feed = 0;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &enable_vsc_feed);
  if(enable_vsc_feed) SET_BITX(ENABLE_MASK, 2);
  uint8_t enable_gamma_correction = 0;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &enable_gamma_correction);
  if(enable_gamma_correction) SET_BITX(ENABLE_MASK, 3);
  uint8_t enable_simeon_filter = 0;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &enable_simeon_filter);
  if(enable_simeon_filter) SET_BITX(ENABLE_MASK, 4);
  
  uint8_t enable_imu_extraplate = 0;
  (void)fscanf(fp, "%s", input_variable_name);
  if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
  (void)fscanf(fp, "%u", &enable_imu_extraplate);
  if(enable_imu_extraplate) SET_BITX(ENABLE_MASK, 5);
  
  uint8_t enable_lyapunov_lost = 0;
  (void)fscanf(fp, "%s", input_variable_name);
  if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
  (void)fscanf(fp, "%u", &enable_lyapunov_lost);
  if(enable_lyapunov_lost) SET_BITX(ENABLE_MASK, 6);
  outData->ENABLE_MASK = ENABLE_MASK;

  uint16_t ACC_RANGE;
	(void)fscanf(fp, "%s", input_variable_name);
	if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
	(void)fscanf(fp, "%u", &ACC_RANGE);
  configACCRangeInHid(ACC_RANGE,&outData->NON_MAX_SUPRESSION_RANGE_ACC_RANGE);
  
  (void)fscanf(fp, "%s", input_variable_name);
  if(!judge_conf_elem_name(input_variable_name)){ret= 777;fclose(fp);return ret;}
  (void)fscanf(fp, "%u", &temp_flag);
    
  counter = 0;

  fclose(fp);
  return 0;
}

void HidControl::send_config(std::string conf_file)
{
  std::cout << " SEND CONFIGURATION !!! " << std::endl;
  unsigned char write_buffer[63] = {0};
  uint8_t config_id[3] = {2, 0x1B, 0x97};
  std::memcpy((uint8_t *)write_buffer, config_id, 3);
  unsigned char read_buf[256];
  int temp_config_flag ;
  int ret = read_slam_config_from_file((slam_config_hid_t *)&write_buffer[3],conf_file,&temp_config_flag);
  write_buffer[TEMP_CONFIG_DATA_FLAG_LOCATION] = temp_config_flag;
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    if(ret!=777){
    int res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
    read_buf[0] = 0x1;
    res = hid_weiz_read(connected_device, read_buf, sizeof(read_buf));
    }else{
      printf("ERROR: CONFIG FILE FORMAT IS WRONG!\n");
    }
  }
  else
#endif
  {
    if(ret!=777){
    int res = hid_read(connected_device, read_buf, sizeof(read_buf));
    res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
    }else{
      printf("ERROR: CONFIG FILE FORMAT IS WRONG!\n");
    }
  }
}

template <int Size>
void send_write_read(hid_device *connected_device, uint8_t info[Size], unsigned char buf[256])
{
  unsigned char buffer[63];
  int read = 0;
  std::memset(buffer, 0, 63);
  std::memcpy((uint8_t *)buffer, (uint8_t *)info, Size);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  int write = hid_write(connected_device, (const unsigned char *)buffer, 63);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (  g_check_version)
  {
    buf[0] = 0x1;
    read = hid_weiz_read(connected_device, buf, 256);
    if(buf[HID_BUF_OFFSET] == NULL)
    {
      read = hid_read(connected_device, buf, 256);
	  hid_set_nonblocking(connected_device, 1);
    }
  }
  else
  {
    #ifdef MULTI_API_VERSION
      if(g_use_new_api)//need to confirm
      {
        buf[0] = 0x1;
        read = hid_weiz_read(connected_device, buf, 256);
      }
      else
    #endif
      {
        read = hid_read(connected_device, buf, 256);
      }
  }
}
void HidControl::save_config(std::string filename){
slam_config_hid_t config_data;
uint8_t mode_info[3]={2,0x1b,0x98};
unsigned char buf[256];
memset(buf,0,256);
buf[0]=0x1;
send_write_read<3>(connected_device, mode_info, buf);
memcpy((char*)&config_data,(char*)&buf[HID_BUF_OFFSET],sizeof(config_data));
printf("COMPARE_DESCRIPTOR_THRESHOLD %.2f\n",config_data.COMPARE_DESCRIPTOR_THRESHOLD);
printf("MAPPING_ADD_P3D_THRESHOLD %d\n",config_data.MAPPING_ADD_P3D_THRESHOLD);
printf("LOC_INLIER_THRESHOLD %d\n",config_data.LOC_INLIER_THRESHOLD);
printf("EPIPOLAR_DISTANCE %.2f\n",float(config_data.EPIPOLAR_DISTANCE/((float)(1<<14))));
printf("PLL_CONFIG_MHZ %d\n",config_data.PLL_CONFIG_MHZ);
printf("IMU_CONFIG_FPS %d\n",config_data.IMU_CONFIG_FPS);
printf("NON_MAX_SUPPRESION_RANGE %d\n",GET_14BITS_FROME_INT16(config_data.NON_MAX_SUPRESSION_RANGE_ACC_RANGE));
printf("COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES %d\n",config_data.COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES);
printf("MATCH_RADIUS_LOC_1 %d\n",int(config_data.MATCH_RADIUS_LOC_1));
printf("MATCH_RADIUS_LOC_2 %d\n",int(config_data.MATCH_RADIUS_LOC_2));
printf("MAP_MATCHING_PREVIOUS_1 %d\n",config_data.MAP_MATCHING_PREVIOUS_1);
printf("MAP_MATCHING_PREVIOUS_2 %d\n",config_data.MAP_MATCHING_PREVIOUS_2);
printf("MAP_STEREO_MATCHING %d\n",config_data.MAP_STEREO_MATCHING);
printf("MINIMUM_INLIER_TO_BE_LOCALIZED %d\n",config_data.MINIMUM_INLIER_TO_BE_LOCALIZED);
printf("MAP_CORRECTION_DISABLE %d\n",0);
printf("DETECTOR_TARGET_FEATURES %d\n",config_data.DETECTOR_TARGET_FEATURES);
printf("DETECTOR_THRESHOLD_LOWER_LIMIT %hhu\n",config_data.DETECTOR_THRESHOLD_LOWER_LIMIT);
printf("DETECTOR_THRESHOLD_UPPER_LIMIT %hhu\n",config_data.DETECTOR_THRESHOLD_UPPER_LIMIT);
// fprintf(saveTXTfp,"FEATURE_MASK: %d\n",config_data.FEATURE_MASK);
//feature mask
printf("ENABLE_6_DOF_POST_FILTER %d\n",getBitN(config_data.FEATURE_MASK,0));
printf("ENABLE_POSEGRAPH %d\n",getBitN(config_data.FEATURE_MASK,1));
printf("ENABLE_CIRCULAR_MAP %d\n",getBitN(config_data.FEATURE_MASK,2));
printf("ENABLE_MAP_CORRECTION %d\n",getBitN(config_data.FEATURE_MASK,3));
printf("LOCALIZATION_MATCHING_MODE %d\n",getBitN(config_data.FEATURE_MASK,4));
printf("ENABLE_CROSS_MATCHING_MAP %d\n",getBitN(config_data.FEATURE_MASK,5));
printf("ENABLE_EPIPOLAR_MATCH3d2d_match2 %d\n",getBitN(config_data.FEATURE_MASK,6));
printf("ENABLE_EPIPOLAR_STEREO_MATCH %d\n",getBitN(config_data.FEATURE_MASK,7));
printf("INLIER_RATIO %0.1f\n",DECODE_Q8_8(config_data.INLIER_RATIO));
printf("MOVE_DIST %hhu\n",config_data.MOVE_DIST);
printf("MOVE_ANGLE_DEGREE %hhu\n",config_data.MOVE_ANGLE_DEGREE);
printf("DETECTOR_STEP_SIZE %hhu\n",config_data.DETECTOR_STEP_SIZE);
printf("DETECTOR_TOLERANCE %.2f\n",DECODE_Q8_10(config_data.DETECTOR_TOLERANCE));//decode DETECTOR_TOLERANCE
printf("POSE_FILTER_RATIO_ROTATION %.1f\n",DECODE_Q8_8(config_data.POSE_FILTER_RATIO_ROTATION));//decode POSE_FILTER_RATIO_ROTATION
printf("POSE_FILTER_RATIO_TRANSLATION %.2f\n",DECODE_Q8_8(config_data.POSE_FILTER_RATIO_TRANSLATION));// decode POSE_FILTER_RATIO_TRANSLATION
printf("_NB_KF_IN_LOCALMAP %hhu\n",config_data._NB_KF_IN_LOCALMAP);
printf("_NUM_FEATURE_IN_LOCAL_BASE %d\n",config_data._NUM_FEATURE_IN_LOCAL_BASE);
printf("_CIRCULAR_BUFFER_SIZE %d\n",config_data._CIRCULAR_BUFFER_SIZE);
printf("_NB_OPT_POSE %hhu\n",config_data._NB_OPT_POSE);
printf("_NB_OPT_3D_RANGE_SIZE %hhu\n",config_data._NB_OPT_3D_RANGE_SIZE);
printf("_NB_ITERATION_MAP_CORRECTION %hhu\n",config_data._NB_ITERATION_MAP_CORRECTION);
printf("_NB_ITERATION_COMPUTE_POSE %hhu\n",config_data._NB_ITERATION_COMPUTE_POSE);
printf("CAMERA_TARGET_FPS %hhu\n",config_data.CAMERA_TARGET_FPS);
// fprintf(saveTXTfp,"ENABLE_MASK: %hhu\n",config_data.ENABLE_MASK);
//decoding enable mask
printf("enable_power_saving %d\n",getBitN(config_data.ENABLE_MASK,0));
printf("enable_uart %d\n",getBitN(config_data.ENABLE_MASK,1));
printf("enable_vsc_feed %d\n",getBitN(config_data.ENABLE_MASK,2));
printf("enable_gamma_correction %d\n",getBitN(config_data.ENABLE_MASK,3));
printf("enable_simeon_filter %d\n",getBitN(config_data.ENABLE_MASK,4));
printf("enable_imu_extrapolate %d\n",getBitN(config_data.ENABLE_MASK,5));
printf("enable_lyapunov_lost %d\n",getBitN(config_data.ENABLE_MASK,6));
printf("ACC_RANGE %d\n",returnACCRangeInG(GET_15_16BITS_FROM_INT16(config_data.NON_MAX_SUPRESSION_RANGE_ACC_RANGE)));
printf("TEMPORARY_CONFIG_DATA 0\n");
FILE *saveTXTfp=NULL;
int print_count=0;
saveTXTfp=fopen((const char*)filename.data(),"w");//to open user's select file path
if(saveTXTfp==NULL){printf("Can not find the /data/config/ directory\n");}
else{
		fprintf(saveTXTfp,"%s %.2f\n",config_element_name[print_count++],config_data.COMPARE_DESCRIPTOR_THRESHOLD);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.MAPPING_ADD_P3D_THRESHOLD));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.LOC_INLIER_THRESHOLD));
		fprintf(saveTXTfp,"%s %.2f\n",config_element_name[print_count++],float(config_data.EPIPOLAR_DISTANCE/((float)(1<<14))));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.PLL_CONFIG_MHZ));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.IMU_CONFIG_FPS));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],GET_14BITS_FROME_INT16(int(config_data.NON_MAX_SUPRESSION_RANGE_ACC_RANGE)));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.MATCH_RADIUS_LOC_1));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data.MATCH_RADIUS_LOC_2));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],config_data.MAP_MATCHING_PREVIOUS_1);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],config_data.MAP_MATCHING_PREVIOUS_2);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],config_data.MAP_STEREO_MATCHING);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],config_data.MINIMUM_INLIER_TO_BE_LOCALIZED);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],0);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],config_data.DETECTOR_TARGET_FEATURES);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.DETECTOR_THRESHOLD_LOWER_LIMIT);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.DETECTOR_THRESHOLD_UPPER_LIMIT);
		//fprintf(saveTXTfp,"FEATURE_MASK: %d\n",config_data.FEATURE_MASK);
		//feature mask
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,0));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,1));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,2));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,3));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,4));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,5));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,6));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.FEATURE_MASK,7));
		fprintf(saveTXTfp,"%s %0.1f\n",config_element_name[print_count++],DECODE_Q8_8(config_data.INLIER_RATIO));
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.MOVE_DIST);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.MOVE_ANGLE_DEGREE);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.DETECTOR_STEP_SIZE);
		fprintf(saveTXTfp,"%s %.2f\n",config_element_name[print_count++],DECODE_Q8_10(config_data.DETECTOR_TOLERANCE));//decode DETECTOR_TOLERANCE
		fprintf(saveTXTfp,"%s %.1f\n",config_element_name[print_count++],DECODE_Q8_8(config_data.POSE_FILTER_RATIO_ROTATION));//decode POSE_FILTER_RATIO_ROTATION
		fprintf(saveTXTfp,"%s %.2f\n",config_element_name[print_count++],DECODE_Q8_8(config_data.POSE_FILTER_RATIO_TRANSLATION));// decode POSE_FILTER_RATIO_TRANSLATION
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data._NB_KF_IN_LOCALMAP);
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data._NUM_FEATURE_IN_LOCAL_BASE));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],int(config_data._CIRCULAR_BUFFER_SIZE));
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data._NB_OPT_POSE);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data._NB_OPT_3D_RANGE_SIZE);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data._NB_ITERATION_MAP_CORRECTION);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data._NB_ITERATION_COMPUTE_POSE);
		fprintf(saveTXTfp,"%s %hhu\n",config_element_name[print_count++],config_data.CAMERA_TARGET_FPS);
		//fprintf(saveTXTfp,"ENABLE_MASK: %hhu\n",config_data.ENABLE_MASK);
		//decoding enable mask
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,0));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,1));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,2));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,3));
		fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,4));
    fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,5));
    fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],getBitN(config_data.ENABLE_MASK,6));
    fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count++],returnACCRangeInG(GET_15_16BITS_FROM_INT16(config_data.NON_MAX_SUPRESSION_RANGE_ACC_RANGE)));
    fprintf(saveTXTfp,"%s %d\n",config_element_name[print_count],0);
    fclose(saveTXTfp);}
}
std::string HidControl::get_software_version()
{
  uint8_t mode_info[3] = {2, 0x1c, 0x99};
  unsigned char buf[256];
  g_check_version = true;
  memset(buf, 0, 256);
  buf[0] = 0x1;
  send_write_read<3>(connected_device, mode_info, buf);
  send_write_read<3>(connected_device, mode_info, buf);
  send_write_read<3>(connected_device, mode_info, buf);
  uint8_t *versionInfo = (uint8_t *)&buf[HID_BUF_OFFSET];
//  printf(" VERSION: %u, data: %s \n",versionInfo[0],(char*)&versionInfo[1]);
  std::string msg;
  msg.resize(versionInfo[0]);
  for (int i = 1; i < versionInfo[0]; ++i)
    msg[i - 1] = (char)versionInfo[i];
  g_check_version = false;
  return msg;
}

float HidControl::get_software_number()
{
  string msg = get_software_version();
  int pos1 = msg.find('P');
  if (pos1 >7)
  {
    pos1 = msg.find('|');
  }
  string verno = msg.substr(1, pos1);
  return (stof(verno));
}

std::string HidControl::get_platform_version()
{
  std::string msg = get_software_version();
  int pos1 = msg.find('|', msg.find('|') + 1) + 1;
  int pos2 = msg.find('|', pos1);
  return msg.substr(pos1, pos2 - pos1);
}

float rotation_norm(float const *v)
{
  int i;
  float sn = 0.f;
  for (i = 0; i < 9; ++i)
    sn += v[i] * v[i];
  return std::sqrt(sn);
}

void HidControl::get_calibration(DeviceCalibration &device_calibration)
{
  float norme1 = 0, norme2 = 0;
  float target = std::sqrt(3);
  int max = 10;
  while (std::abs(norme1 - target) > 0.2 || std::abs(norme2 - target) > 0.2)
  {
    uint8_t mode_info3[3] = {2,0xCA,0x12};
    unsigned char buf3[256];
    memset(buf3, 0, 256);
#ifdef MULTI_API_VERSION
   if(g_use_new_api)
   {
      buf3[0] = 0x1;
      buf3[1] = 0x1;
      buf3[2] = 0x1;
      buf3[3] = 0x1;
   }
#endif
    send_write_read<3>(connected_device, mode_info3, buf3);

    uint8_t mode_info0[3] = {2,0xCA,0x12};
    unsigned char buf0[256];
    memset(buf0,0,256);
    send_write_read<3>(connected_device,mode_info0,buf0);

    uint8_t mode_info1[3] = {2,0xCA,0x12};
    unsigned char buf1[256];
    memset(buf1,0,256);
    send_write_read<3>(connected_device,mode_info1,buf1);

    uint8_t mode_info2[3] = {2,0xCA,0x12};
    unsigned char buf2[256];
    memset(buf2,0,256);
    send_write_read<3>(connected_device,mode_info2,buf2);

    memcpy((char*)(char*)device_calibration.R1, buf0+HID_BUF_OFFSET, 60);
    memcpy((char*)(char*)device_calibration.R1+60, buf1+HID_BUF_OFFSET, 60);
    memcpy((char*)(char*)device_calibration.R1+60+60, buf2+HID_BUF_OFFSET, 60);

    norme1 = rotation_norm(device_calibration.R1);
    norme2 = rotation_norm(device_calibration.R2);
    max --;
    if(max<0)
    {
      std::cout << " Impossible to load a good calibration ... " << std::endl;
      return ;
    }
  }
}

void HidControl::enable_usb_boot_loader()
{
  std::cout << " enable_usb_boot_loader " << std::endl;

  if ( get_software_number() >= 0.04)
  {
      std::cout << " send pre_mode command: 2, 0xde, 0x12" << std::endl;
      uint8_t pre_mode_info[8] = {2, 0xde, 0x12, 0, 0, 0, 0, 0};
      _send_mode_info<8>(connected_device, pre_mode_info);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << " send switch_mode command: 2, 0x12, 0x34" << std::endl;
  uint8_t mode_info[8] = {2, 0x12, 0x34, 0, 0, 0, 0, 0};
  _send_mode_info<8>(connected_device, mode_info);
  std::cout << " switched to usb bootloader mode, exit the program! " << std::endl;
}

void HidControl::enable_6dof_imu(bool flip)
{
  printf("enable_6dof_imu flip : %d\n", flip);
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 0, 0, 0, 0, flip}; //2 0x19 0x95 1 0 0 0 0
  _send_mode_info<9>(connected_device, mode_info);
}



void HidControl::enable_debug_info(bool flip)
{
  printf("enable_debug_info flip : %d\n", flip);
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 3, 0, 0, 0, flip};
  _send_mode_info<9>(connected_device, mode_info);
}

void HidControl::set_cnn_source(int cnn_source)
{
  printf("set_cnn_source : %d\n", cnn_source);
  uint8_t mode_info[4] = {2, 0x20, 0x20, cnn_source};
  _send_mode_info<4>(connected_device, mode_info);
}

void HidControl::enable_detector_only(bool flip)
{
  printf("enable_detector_only flip : %d\n", flip);
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 3, 0, 0, 1, flip};
  _send_mode_info<9>(connected_device, mode_info);
}

void HidControl::disable_debug_info(bool flip)
{
  printf("disable_debug_info flip : %d\n", flip);
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 1, 0, 0, 0, flip};
  _send_mode_info<9>(connected_device, mode_info);
}

void HidControl::enable_frame(bool flip)
{
  printf("enable_frame flip : %d\n", flip);
  uint8_t mode_info[9] = {2, 0x19, 0x95, 0, 1, 0, 0, 0, flip};
  _send_mode_info<9>(connected_device, mode_info);
}

void HidControl::reset_slam()
{
  uint8_t mode_info[4] = {2, 0x1A, 0x96, 1};
  _send_mode_info<4>(connected_device, mode_info);
}

int HidControl::bno_dyn_calib(BNO_DYN_CALIB_REQUEST_t request,BNO_DYN_CALIB_RESPONSE_t &response)
{
  unsigned char read_buf[256];
  unsigned char write_buffer[63];
  uint8_t imu_flag[6] = {2, 0xbf, 0x9b, (uint8_t)request.p1, (uint8_t)request.p2,(uint8_t)request.not_record};
  std::memcpy((uint8_t*)write_buffer, imu_flag, 6); 
  int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63); 
  res = hid_read(connected_device, read_buf, sizeof(read_buf));
  response.acc_status   = read_buf[HID_BUF_OFFSET];
  response.gyr_status   = read_buf[HID_BUF_OFFSET+1];
  response.mag_status   = read_buf[HID_BUF_OFFSET+2];
  response.lacc_status  = read_buf[HID_BUF_OFFSET+3];
  response.gra_status   = read_buf[HID_BUF_OFFSET+4];
  response.gameR_status = read_buf[HID_BUF_OFFSET+5];
  return 0;
}

void HidControl::request_get_processing_time(bool enable){
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
  unsigned char write_buffer[63];
  uint8_t imu_flag[4] = {2, 0xDE, 0xBB, 0x0};
  imu_flag[3] = (enable == true) ? 0x1 : 0x0;
  std::memcpy((uint8_t *)write_buffer, imu_flag, 4);
  unsigned char read_buf[256];
  hid_write(connected_device, (const unsigned char *)write_buffer, 63);
}
#endif
}
void HidControl::get_processing_time(ProcessingTime &pt,float * swISP_dbg)
{
  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    int res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);
  }
  else
#endif
  {
    unsigned char write_buffer[63];
    uint8_t imu_flag[3] = {2,0xDE,0xBB};
    std::memcpy((uint8_t*)write_buffer, imu_flag, 3); 
    int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63); 
    res = hid_read(connected_device, read_buf, sizeof(read_buf));
  }

  float *myHidData = (float *)&read_buf[HID_BUF_OFFSET + sizeof(uint32_t)];
  //printf("[%s] myHidData is %f\r\n", __func__, myHidData[0]);
  int frame = *(uint32_t *)&read_buf[HID_BUF_OFFSET];
  std::memcpy(pt.values, myHidData, sizeof(float) * 8);
  std::memcpy(swISP_dbg, read_buf+HID_BUF_OFFSET+sizeof(uint32_t)+sizeof(float) * 9, sizeof(float) * 2);
  
}


void stststic_ave(unsigned int in, unsigned int count, unsigned int *acc,char * tag)
{
  *acc = *acc + in;
  printf("[%s] %d\n", tag, (*acc)/count);
}

void stststic_max(unsigned int in, unsigned int count, unsigned int *accmax, char * tag)
{
  *accmax = in > (*accmax) ?  in : (*accmax);
  printf("[%s] %d\n", tag, *accmax);
}

void stststic_min( int in, unsigned int count, unsigned int *accmin, char * tag)
{
  *accmin = in < (*accmin) ?  in : (*accmin);
  printf("[%s] %d\n", tag, *accmin); 
}

int HidControl::hid_cycles_debug(int nb[2])
{
  static unsigned int record_id1 = 0;
  static unsigned int record_id2 = 0;
  static unsigned int record_id3 = 0;
  static unsigned int record_id4 = 0;

  static unsigned int count = 1;

  static unsigned int accwhole = 0;
  static unsigned int acclocalization = 0;
  static unsigned int accdetector = 0;
  static unsigned int wholemax = 0;
  static unsigned int detectormax = 0;
  static unsigned int localizemax = 0;
  static unsigned int wholemin = 0;
  static unsigned int detectormin = 0;
  static unsigned int localizemin = 0;

  static unsigned int acclocalized = 0;
  static unsigned int accwrap1 = 0;
  static unsigned int accwrap2 = 0;
  static unsigned int accpost = 0;
  static unsigned int localizedmax = 0;
  static unsigned int wrap1max = 0;
  static unsigned int wrap2max = 0;
  static unsigned int postmax = 0;
  static unsigned int localizedmin = 0;
  static unsigned int wrap1min = 0;
  static unsigned int wrap2min = 0;
  static unsigned int postmin = 0;

  static unsigned int  localizationshave1ave = 0;
  static unsigned int  localizationshave2ave = 0;
  static unsigned int  localizationshave1max = 0;
  static unsigned int  localizationshave2max = 0;
  static unsigned int  localizationshave1min = 0;
  static unsigned int  localizationshave2min = 0;

  static unsigned int  left_map_ave = 0;
  static unsigned int  right_map_ave = 0;
  static unsigned int  left_map_max = 0;
  static unsigned int  right_map_max = 0;
  static unsigned int  left_map_min = 0;
  static unsigned int  right_map_min = 0;
  static unsigned int  whole_ave = 0;
  static unsigned int  whole_max = 0;
  static unsigned int  whole_min = 0;

  unsigned char write_buffer[63];
  uint8_t imu_flag[3] = {2,0x1b,0xbb};

  std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
  unsigned char read_buf[256];

  int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63);
    
  // res = hid_read(connected_device, read_buf, sizeof(read_buf));
  res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);

  
  if ((read_buf[1] == 0x1b) && (read_buf[2] == 0xbb))
  {
    vSLAMLatency vslam;
    memcpy(&vslam, &read_buf[3], sizeof(vslam));
    if ((vslam.whole_id > record_id1) && ((vslam.whole_id - record_id1) < 1000) && 
        (vslam.localModule_id > record_id2) && ((vslam.localModule_id-record_id2) < 1000) && 
        (vslam.localModule_id > 0) && (vslam.whole_id > 0) && 
        (vslam.shave1_id > record_id3) && ((vslam.shave1_id - record_id3) < 1000) && 
        (vslam.shave2_id > record_id4) && ((vslam.shave2_id - record_id4) < 1000));
    {
      record_id1 = vslam.whole_id;
      record_id2 = vslam.localModule_id;
      record_id3 = vslam.shave1_id;
      record_id4 = vslam.shave2_id;

      static int first_time = 0;
      if ((1000 < vslam.whole_id) && (vslam.whole_id< 8000) && 
          (1000 < vslam.localModule_id) && (vslam.localModule_id< 8000))
      {
        if (!first_time)
        {
          detectormax = vslam.detectorModule_US;
          wholemax = vslam.whole_US;
          localizemax = vslam.localModule_US;
          wholemin = vslam.whole_US;
          detectormin = vslam.detectorModule_US;
          localizemin = vslam.localModule_US;

          localizedmax = vslam.localModule_localizedUS;
          wrap1max = vslam.localModule_wrap1US;
          wrap2max = vslam.localModule_wrap2US;
          postmax = vslam.localModule_postUS;
          localizedmin = vslam.localModule_localizedUS;
          wrap1min = vslam.localModule_wrap1US;
          wrap2min = vslam.localModule_wrap2US;
          postmin = vslam.localModule_postUS;

          localizationshave1max = vslam.localizationshave1;
          localizationshave2max = vslam.localizationshave2;
          localizationshave1min = vslam.localizationshave1;
          localizationshave2min = vslam.localizationshave2;

          left_map_max = nb[0];
          right_map_max = nb[1];
          left_map_min = nb[0];
          right_map_min = nb[1];
          whole_max = left_map_max + right_map_max;
          whole_min = left_map_min + right_map_min;

          first_time = 1;
        }
        printf("count = %d, whole_id = %d, shave1_id = %d, shave2_id = %d\n", count, vslam.whole_id, vslam.shave1_id, vslam.shave2_id);
        printf("cycles: %d %d %d %d  %d %d %d %d %d\nshaves: %d %d %d %d\nfeatures_num:%d (%d %d)\n\n", vslam.whole_id, 
                                                                                                        vslam.localModule_id, 
                                                                                                        vslam.whole_US, 
                                                                                                        vslam.detectorModule_US, 
                                                                                                        vslam.localModule_US, 
                                                                                                        vslam.localModule_localizedUS,
                                                                                                        vslam.localModule_wrap1US, 
                                                                                                        vslam.localModule_wrap2US, 
                                                                                                        vslam.localModule_postUS,
                                                                                                        vslam.shave1_id,
                                                                                                        vslam.shave2_id,
                                                                                                        vslam.localizationshave1,
                                                                                                        vslam.localizationshave2,
                                                                                                        (nb[0] + nb[1]),
                                                                                                        nb[0],
                                                                                                        nb[1]);

        stststic_ave(vslam.whole_US, count, &accwhole, "whole_ave");
        stststic_ave(vslam.detectorModule_US, count, &accdetector, "detect_ave");
        stststic_ave(vslam.localModule_US, count, &acclocalization, "local_ave");
        printf("\n");
        stststic_max(vslam.whole_US, count, &wholemax, "whole_max");
        stststic_max(vslam.detectorModule_US, count, &detectormax, "detect_max");
        stststic_max(vslam.localModule_US, count, &localizemax, "local_max");
        printf("\n");
        stststic_min(vslam.whole_US, count, &wholemin, "whole_min");
        stststic_min(vslam.detectorModule_US, count, &detectormin, "detect_min");
        stststic_min(vslam.localModule_US, count, &localizemin, "local_min");
        printf("\n");

        stststic_ave(vslam.localModule_localizedUS, count, &acclocalized, "localized_ave");
        stststic_ave(vslam.localModule_wrap1US, count, &accwrap1, "wrap1_ave");
        stststic_ave(vslam.localModule_wrap2US, count, &accwrap2, "wrap2_ave");
        stststic_ave(vslam.localModule_postUS, count, &accpost, "post_ave");
        printf("\n");
        stststic_max(vslam.localModule_localizedUS, count, &localizedmax, "localized_max");
        stststic_max(vslam.localModule_wrap1US, count, &wrap1max, "wrap1_max");
        stststic_max(vslam.localModule_wrap2US, count, &wrap2max, "wrap2_max");
        stststic_max(vslam.localModule_postUS, count, &postmax, "post_max");
        printf("\n");
        stststic_min(vslam.localModule_localizedUS, count, &localizedmin, "localized_min");
        stststic_min(vslam.localModule_wrap1US, count, &wrap1min, "wrap1_min");
        stststic_min(vslam.localModule_wrap2US, count, &wrap2min, "wrap2_min");
        stststic_min(vslam.localModule_postUS, count, &postmin, "post_min");
        printf("\n");

        stststic_ave(vslam.localizationshave1, count, &localizationshave1ave, "localizationshave1_ave");
        stststic_ave(vslam.localizationshave2, count, &localizationshave2ave, "localizationshave2_ave");
        printf("\n");
        stststic_max(vslam.localizationshave1, count, &localizationshave1max, "localizationshave1_max");
        stststic_max(vslam.localizationshave2, count, &localizationshave2max, "localizationshave2_max");
        printf("\n");
        stststic_min(vslam.localizationshave1, count, &localizationshave1min, "localizationshave1_min");
        stststic_min(vslam.localizationshave2, count, &localizationshave2min, "localizationshave2_min");
        printf("\n");

        stststic_ave(nb[0] + nb[1], count, &whole_ave, "whole_ave");
        stststic_ave(nb[0], count, &left_map_ave, "left_ave");
        stststic_ave(nb[1], count, &right_map_ave, "right_ave");
        printf("\n");
        stststic_max(nb[0] + nb[1], count, &whole_max, "whole_max");
        stststic_max(nb[0], count, &left_map_max, "left_max");
        stststic_max(nb[1], count, &right_map_max, "right_max");
        printf("\n");
        stststic_min(nb[0] + nb[1], count, &whole_min, "whole_min");
        stststic_min(nb[0], count, &left_map_min, "left_min");
        stststic_min(nb[1], count, &right_map_min, "right_min");

        count++;
        printf("\n");
      }
    }
  }
  return 0;
}

int HidControl::internal_grab_imu_raw(unsigned char raw_data[60])
{
  unsigned char write_buffer[63];
  uint8_t imu_flag[3] = {2, 0xBE, 0xEF};
  //uint8_t imu_flag[3] = {2,0xA2,0x33};

  std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    int res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);
  }
  else
#endif
  {
    int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63);
    res = hid_read(connected_device, read_buf, sizeof(read_buf));
  }
  if(read_buf[1]!=(unsigned char)0xBE || read_buf[2]!=(unsigned char)0xEF){return 1;}
  memcpy((unsigned char*)raw_data,&read_buf[HID_BUF_OFFSET],60);
  return 0;
}
 int HidControl::internal_grab_imu_new_raw(unsigned char raw_data[60])
 {

  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    int res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);
  }
  else
#endif
  {
    unsigned char write_buffer[63];
    uint8_t imu_flag[3] = {2, 0xBD, 0xDF};
    std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
    int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63);
    res = hid_read(connected_device, read_buf, sizeof(read_buf));
  }
  if(read_buf[1]!=(unsigned char)0xBD || read_buf[2]!=(unsigned char)0xDF){return 1;}
  memcpy((unsigned char*)raw_data,(unsigned char*)&read_buf[HID_BUF_OFFSET],60);
  return 0;
 }

 void HidControl::internal_grab_imu(float values[9])
{
  unsigned char write_buffer[63];
  uint8_t imu_flag[3] = {2, 0xBE, 0xEF};
  //uint8_t imu_flag[3] = {2,0xA2,0x33};

  std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    int res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);
  }
  else
#endif
  {
    int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63);
    res = hid_read(connected_device, read_buf, sizeof(read_buf));
  }

  HIDoutData *myHidData = (HIDoutData *)&read_buf[HID_BUF_OFFSET];

  values[0] = (float)(double(myHidData->tymeStamp) / double(1e9));
  values[1] = (float)myHidData->frameCount;

  // myHidData->accelData is the gyro !
  values[5] = -myHidData->accelData.axisData.y;
  values[6] = myHidData->accelData.axisData.x;
  values[7] = myHidData->accelData.axisData.z;
  // myHidData->gyroData is the accel !
  values[2] = -myHidData->gyroData.axisData.y;
  values[3] = myHidData->gyroData.axisData.x;
  values[4] = myHidData->gyroData.axisData.z;
    
  values[8] = 0;


  /*
if(_45mm || _80mm)
{
  // myHidData->accelData is the gyro !
  values[5] = -myHidData->accelData.axisData.y;
  values[6] = myHidData->accelData.axisData.x;
  values[7] = myHidData->accelData.axisData.z;
  // myHidData->gyroData is the accel !
  values[2] = -myHidData->gyroData.axisData.y;
  values[3] = myHidData->gyroData.axisData.x;
  values[4] = myHidData->gyroData.axisData.z;

  // std::cout << " IMU " 
  //   << values[2] << " " << values[3] << " " << values[4] << " "
  //   << values[5] << " " << values[6] << " " << values[7] << std::endl;
}
*/
  if (values[2] > 1000 || values[3] > 1000 || values[4] > 1000 ||
      values[5] > 1000 || values[6] > 1000 || values[7] > 1000)
  {
    std::cout
        << " bad imu data ... "
        << values[0] << " " << values[1]
        << values[2] << " " << values[3] << " " << values[4] << " "
        << values[5] << " " << values[6] << " " << values[7] << std::endl;
    memset(values, 0, sizeof(float) * 8);
  }
  else
  {
    // std::cout
    // << values[0] << " " << values[1]
    // << values[2] << " " << values[3] << " " << values[4] << " "
    // << values[5] << " " << values[6] << " " << values[7] << std::endl;
  }
}


void HidControl::internal_grab_imu_new(float values[12])
{

  unsigned char read_buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    int res = hid_read_timeout(connected_device, read_buf, sizeof(read_buf), 200);
  }
  else
#endif
  {
    unsigned char write_buffer[63];
    uint8_t imu_flag[3] = {2, 0xBD, 0xDF};
    std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
    int res = hid_write(connected_device, (const unsigned char*)write_buffer, 63);
    res = hid_read(connected_device, read_buf, sizeof(read_buf));
  }

  HIDoutData_new *myHidData = (HIDoutData_new *)&read_buf[HID_BUF_OFFSET];

  values[0] = (float)((double)myHidData->imu_time / double(1e3));
  values[1] = (float)((double)myHidData->camera_time / double(1e3));
  values[2] = (float)myHidData->frameCount;

  // myHidData->accelData is the gyro !
  values[6] = -myHidData->accelData.axisData.y;
  values[7] = myHidData->accelData.axisData.x;
  values[8] = myHidData->accelData.axisData.z;
  // myHidData->gyroData is the accel !
  values[3] = -myHidData->gyroData.axisData.y;
  values[4] = myHidData->gyroData.axisData.x;
  values[5] = myHidData->gyroData.axisData.z;

  values[9] = -myHidData->magnData.axisData.y;
  values[10] = myHidData->magnData.axisData.x;
  values[11] = myHidData->magnData.axisData.z;

  /*
if(_45mm || _80mm)
{
  // myHidData->accelData is the gyro !
  values[5] = -myHidData->accelData.axisData.y;
  values[6] = myHidData->accelData.axisData.x;
  values[7] = myHidData->accelData.axisData.z;
  // myHidData->gyroData is the accel !
  values[2] = -myHidData->gyroData.axisData.y;
  values[3] = myHidData->gyroData.axisData.x;
  values[4] = myHidData->gyroData.axisData.z;

  // std::cout << " IMU " 
  //   << values[2] << " " << values[3] << " " << values[4] << " "
  //   << values[5] << " " << values[6] << " " << values[7] << std::endl;
}

*/

  if (std::abs(values[3]) > 1000 || std::abs(values[4]) > 1000 || std::abs(values[5]) > 1000 ||
      std::abs(values[6]) > 1000 || std::abs(values[7]) > 1000 || std::abs(values[8]) > 1000 ||
      std::abs(values[9]) > 1000 || std::abs(values[10]) > 1000 || std::abs(values[11]) > 1000)
  {
    std::cout
        << " bad imu data ... "
        << values[0] << " " << values[1]
        << values[2] << " " << values[3] << " " << values[4] << " "
        << values[5] << " " << values[6] << " " << values[7] << " "
        << values[8] << " " << values[9] << " " << values[10] << " "
        << values[11] << std::endl;
    memset(values, 0, sizeof(float) * 12);
  }
  else
  {
    // std::cout
    // << values[0] << " " << values[1]
    // << values[2] << " " << values[3] << " " << values[4] << " "
    // << values[5] << " " << values[6] << " " << values[7] << std::endl;
  }

}
double now();
void HidControl::grab_imu(float values[9])
{
  internal_grab_imu(values);
}

void HidControl::grab_imu_new(float values[12])
{
  internal_grab_imu_new(values);
}

void HidControl::imu_request(void)
{
  unsigned char write_buffer[63];
  uint8_t imu_flag[4] = {2, 0xBE, 0xEF, 0x1};
  std::memcpy((uint8_t *)write_buffer, imu_flag, 4);
  unsigned char read_buf[256];
  hid_write(connected_device, (const unsigned char *)write_buffer, 63);
}

void HidControl::imu_new_request(void)
{
  unsigned char write_buffer[63];
  uint8_t imu_flag[4] = {2, 0xBD, 0xDF, 0x1};
  std::memcpy((uint8_t *)write_buffer, imu_flag, 4);
  unsigned char read_buf[256];
  hid_write(connected_device, (const unsigned char *)write_buffer, 63);
}

void HidControl::get_6dof(HIDInfoStruct &hid_compact)
{
  unsigned char write_buffer[63];
  //uint8_t imu_flag[3] = {2,0xBE,0xEF};
  uint8_t imu_flag[3] = {2, 0xA2, 0x33}; //2 0xA2 0x33
                                         //            IMUInfoCompact imu_compact;

  std::memcpy((uint8_t *)write_buffer, imu_flag, 3);
  int res = hid_write(connected_device, (const unsigned char *)write_buffer, 63);
  unsigned char buf[256];
#ifdef MULTI_API_VERSION
  if(use_api_new)
  {
    res = hid_weiz_read(connected_device, buf, sizeof(buf));
  }
  else
#endif
  {
    res = hid_read(connected_device, buf, sizeof(buf));
  }
  memcpy((char *)&hid_compact, &buf[HID_BUF_OFFSET], sizeof(HIDInfoStruct));
}

void HidControl::enable_imu_fusion()
{
  uint8_t mode_info[6] = {2, 0xE4, 0x23, 1, 0, 0};
  _send_mode_info<6>(connected_device, mode_info);
}

void HidControl::disable_imu_fusion()
{
  uint8_t mode_info[6] = {2, 0xE4, 0x23, 0, 0, 0};
  _send_mode_info<6>(connected_device, mode_info);
}

void HidControl::set_imu_time_offset(uint8_t ms)
{
  uint8_t mode_info[6] = {2, 0xE4, 0x23, 1, 1, ms};
  _send_mode_info<6>(connected_device, mode_info);
}

HidControl &unique_hid_control()
{
  static HidControl singleton;
  return singleton;
}

#ifdef MULTI_API_VERSION
void HidControl::set_api_version(bool use_new_api)
{
  use_api_new = use_new_api;
  g_use_new_api = use_new_api;
}
#endif
void HidControl::set_old_edge_command(bool old_command)
{
  use_old_edge_cmd = old_command;
}

void HidControl::set_uvc_mode2(bool flip)
{
  printf("Set UVC Mode 2 flip : %d\n", flip);
  printf("MixedModeStruct is used!!!!!!!!!\n");
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 2, 0, 0, 0, flip};
  _send_mode_info<9>(connected_device, mode_info);
}

void HidControl::set_uvc_mode5(bool flip)
{
  printf("Set UVC Mode 5 flip : %d\n", flip);
  printf("MixedModeStruct is used!!!!!!!!!\n");
  uint8_t mode_info[9] = {2, 0x19, 0x95, 1, 5, 0, 0, 0, flip};
  _send_mode_info<9>(connected_device, mode_info);
}
void HidControl::set_apriltag_cfg(uint8_t * cfg_buf,uint32_t len)
{
  uint8_t hid_buf[63];
  hid_buf[0] = 0x02;
  hid_buf[1] = 0xAA;
  hid_buf[2] = 0x7A;
  memcpy(&hid_buf[3],cfg_buf,len);
  hid_write(connected_device, (const unsigned char *)hid_buf, 63);
}