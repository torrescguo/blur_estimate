#pragma once

#include "config.h"
#include <utility>
#include <vector>
#include <iostream>
#include "container.h"
#include "ccommon.h"

using uchar = unsigned char;

//#define SAVE_BINARY_DATA
#define LOAD_BINARY_DATA

#define MATCHING_IN_C
#define DESCRIPTOR_IN_C
#define MAPPING_IN_C

#define MAX_QUEUE_SIZE  (100u)

void load_dbg(DebugStruct* dbg, char* buffer);
void save_dbg(DebugStruct* debug_information, char* debug_information_buffer);
void check_debug_struct(DebugStruct*, DebugStruct*, int verbose_level, std::ostream& out=std::cout);
void check_debug_file(std::string folder_slam, std::string filename, std::string title);
void check_debug_buffer(char* , char*, int verbose_level);
void disp_dbg_main_info(DebugStruct* dbg);
//void check_minimal_debug(char *, char *);