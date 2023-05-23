#include "../include/bmp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#define LOG_STR2(S1,S2)         (LOG_BMP("%s %s\n",S1,S2))
#define LOG_STR_INT(S1,S2)      (LOG_BMP("%s %d\n",S1,S2))
BMP_DATA global_bmp_data;
unsigned char rawDataPtr[1024*1024*12];
void init_bmp_data_mem(unsigned int raw_data_bytes,unsigned char *load_ptr)
{
    load_ptr = (unsigned char *)malloc(raw_data_bytes);
}
void free_bmp_data_mem()
{
    if(!rawDataPtr)
    {
        free(rawDataPtr);
    }
}
void log_bmp_header_info()
{
    LOG_STR_INT("bmpFile_type : ",((global_bmp_data.bmpFile_type)));
    LOG_STR_INT("bmpFile_type low  bit: ",((global_bmp_data.bmpFile_type) & (unsigned short)0x00ff));
    LOG_STR_INT("bmpFile_type high bit: ",(((global_bmp_data.bmpFile_type) & (unsigned short)0xff00)>>8));
    LOG_STR_INT("bmpFile_offbits:",global_bmp_data.bmp_header_struct.bmp_file_struct.bmpFile_offbytes);
    LOG_STR_INT("bmpFile_reversed[0]: ",global_bmp_data.bmp_header_struct.bmp_file_struct.bmpFile_reversed[0]);
    LOG_STR_INT("bmpFile_reversed[1]: ",global_bmp_data.bmp_header_struct.bmp_file_struct.bmpFile_reversed[1]);
    LOG_STR_INT("bmpFile_size: ",global_bmp_data.bmp_header_struct.bmp_file_struct.bmpFile_size);
    LOG_STR_INT("bmpInfo_bitcount: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_bitcount);
    LOG_STR_INT("bmpInfo_compression: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_compression);
    LOG_STR_INT("bmpInfo_height: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_height);
    LOG_STR_INT("bmpInfo_width: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_width);
    LOG_STR_INT("bmpInfo_important_clr: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_important_clr);
    LOG_STR_INT("bmpInfo_planes: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_planes);
    LOG_STR_INT("bmpInfo_size: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_size);
    LOG_STR_INT("bmpInfo_used_clr: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_used_clr);
    LOG_STR_INT("bmpInfo_xpels_per_meter: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_xpels_per_meter);
    LOG_STR_INT("bmpInfo_ypels_per_meter: ",global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_ypels_per_meter);
}
void read_bmp_file(const char * readBmpPath,BMP_DATA * loadBmpData)
{
    FILE * fp =  fopen(readBmpPath,"rb");
    int ret=fread(&global_bmp_data.bmpFile_type,sizeof(unsigned char),sizeof(unsigned short),fp);
    ret=fread(&global_bmp_data.bmp_header_struct,sizeof(unsigned char),sizeof(BMP_HEADER),fp);
    uint32_t data_bytes  = global_bmp_data.bmp_header_struct.bmp_file_struct.bmpFile_size - BITMAP_HEADER_LENGTH;
    log_bmp_header_info();
    // init_bmp_data_mem(data_bytes,loadBmpData->pixel_raw_array);
    uint32_t pixel_height_nb = global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_height;
    uint32_t pixel_bytes_width = global_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_image_size/pixel_height_nb;
    // for(uint32_t i = 0 ; i < pixel_height_nb ; i++)
    // {
    //     unsigned char * load_addr = loadBmpData->pixel_raw_array + (pixel_height_nb-1-i)* pixel_bytes_width;
    //     fread(load_addr,sizeof(unsigned char),pixel_bytes_width,fp);
    // }
    ret=fread(loadBmpData->pixel_raw_array,sizeof(unsigned char),data_bytes,fp);
    fclose(fp);
    loadBmpData->bmp_header_struct=global_bmp_data.bmp_header_struct;
    loadBmpData->bmpFile_type=global_bmp_data.bmpFile_type;
    // loadBmpData->pixel_raw_array=rawDataPtr;
}
void save_bmp_file(const char * saveBmpPath,BMP_DATA * saveBmpData)
{
    //force to set the const bits
    saveBmpData->bmpFile_type=(unsigned short)(19778);
    saveBmpData->bmp_header_struct.bmp_file_struct.bmpFile_offbytes=BITMAP_HEADER_LENGTH;
    saveBmpData->bmp_header_struct.bmp_file_struct.bmpFile_reversed[0]=0;
    saveBmpData->bmp_header_struct.bmp_file_struct.bmpFile_reversed[1]=0;
    saveBmpData->bmp_header_struct.bmp_file_struct.bmpFile_size = saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_image_size+BITMAP_HEADER_LENGTH;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_size= BITMAP_INFO_HEADER_LENGTH;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_important_clr=0;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_compression=0;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_planes=1;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_used_clr=0;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_xpels_per_meter=DEFAULT_PIXELS_X_METER;
    saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_ypels_per_meter=DEFAULT_PIXELS_Y_METER;
    //save to target path
    FILE * fp = fopen(saveBmpPath,"wb");
    fwrite(&saveBmpData->bmpFile_type,sizeof(unsigned short),1,fp);
    fwrite(&saveBmpData->bmp_header_struct,sizeof(unsigned char),sizeof(BMP_HEADER),fp);
    unsigned int bytes_perPixel =(unsigned int) (saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_bitcount >>3);
    unsigned int pixel_width    = saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_width;
    unsigned int pixel_height   = saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_height;
    for(int i=pixel_height-1;i>=0;i--)
    {
        unsigned char* save_array_addr =  saveBmpData->pixel_raw_array + i * pixel_width * bytes_perPixel;
        fwrite(save_array_addr,sizeof(unsigned char),pixel_width * bytes_perPixel,fp);
    }
    // fwrite(saveBmpData->pixel_raw_array,sizeof(unsigned char),saveBmpData->bmp_header_struct.bmp_info_struct.bmpInfo_image_size,fp);
    fclose(fp);
}
void save_bgr_bmp(const char * saveBmpPath,unsigned char * bgr,unsigned int pixelW,unsigned int pixelH)
{
    BMP_DATA save_bmp_data;
    save_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_height=pixelH;
    save_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_width=pixelW;
    save_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_bitcount=24;
    save_bmp_data.bmp_header_struct.bmp_info_struct.bmpInfo_image_size=pixelW*pixelH*3;
    save_bmp_data.pixel_raw_array=bgr;
    save_bmp_file(saveBmpPath,&save_bmp_data);
}