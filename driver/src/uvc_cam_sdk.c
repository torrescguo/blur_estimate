#include "../include/uvc_cam_sdk.h"
#include <libudev.h>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <unistd.h>
#define LOG_UVC printf
camera_t* camera_open(const char *device_path, uint32_t width, uint32_t height);
void camera_init(camera_t* p_camera,unsigned int v4l_format);
void camera_start(camera_t* p_camera);
int camera_capture(camera_t* p_camera);
int camera_frame(camera_t* p_camera, struct timeval timeout);
void camera_stop(camera_t* p_camera);
void camera_release(camera_t* p_camera);
void camera_close(camera_t* p_camera);
camera_t* camera_open(const char *device_path, uint32_t width, uint32_t height)
{
	int fd = open(device_path, O_RDWR | O_NONBLOCK);
	if (fd < 0)
	{
		return NULL;
	}
	
	camera_t* p_camera = (camera_t *)malloc(sizeof(camera_t));
	if (NULL == p_camera)
	{
		close(fd);
		return NULL;
	}
	
	memset(p_camera, 0, sizeof(camera_t));
	p_camera->fd = fd;
	p_camera->width = width;
	p_camera->height = height;
	p_camera->buffer_count = 0;
	p_camera->buffers = NULL;
	p_camera->head.start = NULL;
	p_camera->head.length = 0;
	
	return p_camera;	
}
void camera_init(camera_t* p_camera,unsigned int v4l_format)
{
	struct v4l2_capability cap;
	
	memset(&cap, 0, sizeof(cap));
	if (ioctl(p_camera->fd, VIDIOC_QUERYCAP, &cap) >= 0)
	{
		LOG_UVC("version:%d, cap:%d\n", cap.version, cap.capabilities);
		if (! (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		{
			LOG_UVC("[ERROR] camera_init: no campture\n%s","");
			return ;
		}
		
		if (! (cap.capabilities & V4L2_CAP_STREAMING))
		{
			LOG_UVC("[ERROR] camera_init: no streaming\n%s","");
			return ;
		}		
	}
	
	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof(cropcap));
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(p_camera->fd, VIDIOC_CROPCAP, &cropcap) >= 0)
	{
		struct v4l2_crop crop;
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;
		
		ioctl(p_camera->fd, VIDIOC_S_CROP, &crop);
	}
	
	struct v4l2_format format;
	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = p_camera->width;
	format.fmt.pix.height = p_camera->height;
	format.fmt.pix.pixelformat = v4l_format;
	format.fmt.pix.field = V4L2_FIELD_NONE;
	format.fmt.pix.width = p_camera->width;
	ioctl(p_camera->fd, VIDIOC_S_FMT, &format);
	
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(req));
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	ioctl(p_camera->fd, VIDIOC_REQBUFS, &req);
	p_camera->buffer_count = req.count;
	p_camera->buffers = calloc(req.count, sizeof(buffer_t));
	
	size_t i = 0;
	size_t buf_max = 0;
	struct v4l2_buffer buffer;
	for (i = 0; i < p_camera->buffer_count; ++i)
	{
		memset(&buffer, 0, sizeof(buffer));
		
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		
		ioctl(p_camera->fd, VIDIOC_QUERYBUF, &buffer);
		if (buffer.length > buf_max)
			buf_max = buffer.length;
		
		LOG_UVC("count:%ld, length:%d\n", i, buffer.length);
		p_camera->buffers[i].length = buffer.length;
		p_camera->buffers[i].start = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, p_camera->fd, buffer.m.offset);
		if (NULL == p_camera->buffers[i].start)
		{
			perror("mmap error");
			return ;
		}
	}
	
	p_camera->head.start = (uint8_t *)malloc(buf_max);
	
	return ;
}
void camera_start(camera_t* p_camera)
{
	size_t i = 0;
	struct v4l2_buffer buffer;
	for (i = 0; i < p_camera->buffer_count;  ++i)
	{
		memset(&buffer, 0, sizeof(buffer));
		
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		
		ioctl(p_camera->fd, VIDIOC_QBUF, &buffer);
	}
	
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(p_camera->fd, VIDIOC_STREAMON, &type);
	
	return ;
}
int camera_capture(camera_t* p_camera)
{
	struct v4l2_buffer buffer;
	
	memset(&buffer, 0, sizeof(buffer));
		
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	
	ioctl(p_camera->fd, VIDIOC_DQBUF, &buffer);	
	memcpy(p_camera->head.start, p_camera->buffers[buffer.index].start, buffer.bytesused);
	p_camera->head.length = buffer.bytesused;
	
	ioctl(p_camera->fd, VIDIOC_QBUF, &buffer);
	
	return 1;
}
int camera_frame(camera_t* p_camera, struct timeval timeout)
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(p_camera->fd, &fds);
	
	int nready = select(p_camera->fd + 1, &fds, NULL, NULL, &timeout);
	if (nready < 0)
	{
		perror("[ERROR] camera_frame");
		return nready;
	}
	
	if (nready == 0)
	{
		return nready;
	}
	
	return camera_capture(p_camera);
}
void camera_stop(camera_t* p_camera)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(p_camera->fd, VIDIOC_STREAMOFF, &type);
	
	return ;
}
void camera_release(camera_t* p_camera)
{
	size_t i = 0;
	for (i = 0; i < p_camera->buffer_count;  ++i)
	{
		munmap(p_camera->buffers[i].start, p_camera->buffers[i].length);
	}
	
	free(p_camera->buffers);
	p_camera->buffer_count = 0;
	p_camera->buffers = NULL;
	
	free(p_camera->head.start);
	p_camera->head.length = 0;
	p_camera->head.start = NULL;
	
	return ;
}
void camera_close(camera_t* p_camera)
{
	close(p_camera->fd);
	free(p_camera);
}
camera_t* p_camera_global = NULL;
struct timeval timeout;
int uvc_camera_sdk_init(const char * device_path,uint32_t pixel_width,uint32_t pixel_height,int format)
{
	// try to access device_path
	LOG_UVC("try to access device_path\n");
	if (access(device_path, F_OK) != 0)
	{
		perror("[ERROR] can not find the usb camera device!");
		return -1;
	}
	LOG_UVC("access device_path successfully!\n%s","");
	p_camera_global = camera_open(device_path, pixel_width, pixel_height);
	if (NULL == p_camera_global)
	{
		perror("[ERROR] can not open the usb camera device!");
		return -1;
	}
	switch(format)
	{
		case 0:camera_init(p_camera_global,V4L2_PIX_FMT_YUYV);break;
		// case uvc_camera_sdk_stream_mpeg:camera_init(p_camera_global,V4L2_PIX_FMT_MPEG);break;
		case 1:camera_init(p_camera_global,V4L2_PIX_FMT_YUYV);break;
		default:camera_init(p_camera_global,V4L2_PIX_FMT_YUYV);break;
	}
	LOG_UVC("uvc_camera_sdk_init successfully!\n%s","");
	return 0;
}
void uvc_camera_sdk_stream_start(uint64_t timeout_microSeconds)
{
   timeout.tv_sec  = timeout_microSeconds/1000000;
   timeout.tv_usec = timeout_microSeconds%1000000;
   LOG_UVC("try to capture 10 frames at beginning timeout.tv_sec=%ld, timeout.tv_usec=%ld\n",timeout.tv_sec,timeout.tv_usec);
   camera_start(p_camera_global);
   for(int i = 0; i < 10; ++i)
   {
	camera_frame(p_camera_global, timeout);
   }
   LOG_UVC("capture 10 frames ok!\n%s","");
}
camera_t * uvc_camera_sdk_stream_captured_once()
{
	camera_frame(p_camera_global, timeout);
	return p_camera_global;
}
void uvc_camera_sdk_stream_stop()
{
	camera_stop(p_camera_global);
	camera_release(p_camera_global);
	camera_close(p_camera_global);
}
uint8_t meanOfBuffer(uint8_t * pixels_p, uint32_t num)
{
	uint8_t mean_value=0;
	uint32_t sum_total=0;
	for(uint32_t i=0;i<num;++i)
	{
		sum_total += pixels_p[i];
	}
	mean_value = num>0 ? sum_total/num : 0;
	return mean_value;
}
void udev_search_node(char * path_f408_firstNode,int order)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    /* Create the udev object */
    udev = udev_new();
    if (!udev) {
        printf("Can't create udev\n");
        exit(1);
    }

    /* Create a list of the devices in the 'hidraw' subsystem. */
    enumerate = udev_enumerate_new(udev);
//     udev_enumerate_add_match_subsystem(enumerate, "");
    //  udev_enumerate_add_match_sysattr(enumerate,"idProduct","f408"); //"manufacturer","SIUPO"
      udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);
    /* For each item enumerated, print out its information.
       udev_list_entry_foreach is a macro which expands to
       a loop. The loop will be executed for each member in
       devices, setting dev_list_entry to a list entry
       which contains the device's path in /sys. */
    udev_list_entry_foreach(dev_list_entry, devices) {
        static int times=0;
        const char *path;
        /* Get the filename of the /sys entry for the device
           and create a udev_device object (dev) representing it */
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        /* usb_device_get_devnode() returns the path to the device node
           itself in /dev. */
        printf("Device Node Path: %s\n", udev_device_get_devnode(dev));
        const char * record_node=udev_device_get_devnode(dev);
        /* The device pointed to by dev contains information about
           the hidraw device. In order to get information about the
           USB device, get the parent device with the
           subsystem/devtype pair of "usb"/"usb_device". This will
           be several levels up the tree, but the function will find
           it.*/
        dev = udev_device_get_parent_with_subsystem_devtype(
                  dev,
                  "usb",
                  "usb_device");
        if (!dev) {
            printf("Unable to find parent usb device.");
            exit(1);
        }

        /* From here, we can call get_sysattr_value() for each file
           in the device's /sys entry. The strings passed into these
           functions (idProduct, idVendor, serial, etc.) correspond
           directly to the files in the directory which represents
           the USB device. Note that USB strings are Unicode, UCS2
           encoded, but the strings returned from
           udev_device_get_sysattr_value() are UTF-8 encoded. */
        printf("  VID/PID: %s %s\n",
               udev_device_get_sysattr_value(dev,"idVendor"),
               udev_device_get_sysattr_value(dev, "idProduct"));
        printf("  %s\n  %s\n",
               udev_device_get_sysattr_value(dev,"manufacturer"),
               udev_device_get_sysattr_value(dev,"product"));
        printf("  serial: %s\n",
               udev_device_get_sysattr_value(dev, "serial"));
       if(   ((char*)(udev_device_get_sysattr_value(dev, "idProduct")))[0] == 'f'  
       	&& ((char*)(udev_device_get_sysattr_value(dev, "idProduct")))[1] == '4' 
	&& ((char*)(udev_device_get_sysattr_value(dev, "idProduct")))[2] == '0'
	&& ((char*)(udev_device_get_sysattr_value(dev, "idProduct")))[3] == '8'    
       )
       {
	printf("found\n");
	sprintf(path_f408_firstNode,"%s",record_node);
	printf("path dev=%s\n",path_f408_firstNode);
	if(times == order)
	{
		udev_device_unref(dev);
		break;
	}
	times++;
       }
        udev_device_unref(dev);
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);

    udev_unref(udev);
}