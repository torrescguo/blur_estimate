#ifndef __BMP_H__
#define __BMP_H__
#ifdef __DISABLE_LOG_DEBUG__
#define LOG_BMP(X,...) (void)0
#else
#define LOG_BMP(X,...) printf("[Debug]: " #X " \n",__VA_ARGS__)
#endif
#define BITMAP_FILE_LABEL_LENGTH    (sizeof(unsigned short))
#define BITMAP_FILE_HEADER_LENGTH   (sizeof(BMP_FILE))
#define BITMAP_INFO_HEADER_LENGTH   (sizeof(BMP_INFO))
#define BITMAP_HEADER_LENGTH        (BITMAP_FILE_HEADER_LENGTH+BITMAP_INFO_HEADER_LENGTH+BITMAP_FILE_LABEL_LENGTH)
#define BITCOUNT_TO_BYTE_SHIFT      (3)
#define DEFAULT_PIXELS_X_METER      (37795)
#define DEFAULT_PIXELS_Y_METER      (37795)
typedef struct BMP_FILE_t
{
	unsigned int    bmpFile_size;
	unsigned short  bmpFile_reversed[2];
	unsigned int    bmpFile_offbytes;
}BMP_FILE;//12 bytes
typedef struct BMP_INFO_t
{
	unsigned int    bmpInfo_size;
    unsigned int    bmpInfo_width;
    unsigned int    bmpInfo_height;
    unsigned short  bmpInfo_planes;
    unsigned short  bmpInfo_bitcount;
    unsigned int    bmpInfo_compression;
    unsigned int    bmpInfo_image_size;
    unsigned int    bmpInfo_xpels_per_meter;
    unsigned int    bmpInfo_ypels_per_meter;
    unsigned int    bmpInfo_used_clr;
    unsigned int    bmpInfo_important_clr;
}BMP_INFO;//40 bytes
typedef struct BMP_HEADER_t
{
    BMP_FILE bmp_file_struct;
    BMP_INFO bmp_info_struct;
}BMP_HEADER;//54 bytes
typedef struct BMP_DATA_t
{
    unsigned short  bmpFile_type;
    BMP_HEADER bmp_header_struct;
    unsigned char * pixel_raw_array;
}BMP_DATA;
// unsigned char bmpFileHeader[BITMAP_FILE_HEADER_LENGTH];
// unsigned char bmpInfoHeader[BITMAP_INFO_HEADER_LENGTH];
#ifdef __cplusplus
extern "C"{
#endif

    void init_bmp_data_mem(unsigned int raw_data_bytes,unsigned char *load_ptr);
    void free_bmp_data_mem();
    void log_bmp_header_info();
    void read_bmp_file(const char * readBmpPath,BMP_DATA * loadBmpData);
    void save_bmp_file(const char * saveBmpPath,BMP_DATA * saveBmpData);
    void save_bgr_bmp(const char * saveBmpPath,unsigned char * bgr,unsigned int pixelW,unsigned int pixelH);

#ifdef __cplusplus
}
#endif

#endif //__BMP_H__