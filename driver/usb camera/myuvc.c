#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <linux/mm.h>
#include <asm/atomic.h>
#include <asm/unaligned.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include "uvcvideo.h"

/* 参考 drivers/media/video/uvc */

#define MYUVC_URBS 5
#define MAX_BULK_BUFS 6

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

/* Video Class-Specific Request Codes */
#define UVC_RC_UNDEFINED			0x00
#define UVC_SET_CUR					0x01
#define UVC_GET_CUR					0x81
#define UVC_GET_MIN					0x82
#define UVC_GET_MAX					0x83
#define UVC_GET_RES					0x84
#define UVC_GET_LEN					0x85
#define UVC_GET_INFO				0x86
#define UVC_GET_DEF					0x87

struct myuvc_streaming_control {
	__u16 bmHint;
	__u8  bFormatIndex;
	__u8  bFrameIndex;
	__u32 dwFrameInterval;
	__u16 wKeyFrameRate;
	__u16 wPFrameRate;
	__u16 wCompQuality;
	__u16 wCompWindowSize;
	__u16 wDelay;
	__u32 dwMaxVideoFrameSize;
	__u32 dwMaxPayloadTransferSize;
	__u32 dwClockFrequency;
	__u8  bmFramingInfo;
	__u8  bPreferedVersion;
	__u8  bMinVersion;
	__u8  bMaxVersion;
};

struct myuvc_frame_buf {
	/* common v4l buffer stuff -- must be first */
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct myuvc {
	struct device *dev;
	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct usb_device *udev;

	struct vb2_queue *vb2_queue;
	struct list_head queued_bufs;  

	int            buf_num;
	unsigned long  buf_size;
	u8             *buf_list[MAX_BULK_BUFS];
	dma_addr_t     dma_addr[MAX_BULK_BUFS];
	int            urbs_initialized;
	int            urbs_submitted;
	struct urb     *urb_list[MAX_BULK_BUFS];
};

struct frame_desc {
    int width;
    int height;
};

/* 参考uvc_video_queue定义一些结构体 */
struct myuvc_buffer {    
    struct v4l2_buffer buf;
    int state;
    int vma_use_count; /* 表示是否已经被mmap */
    wait_queue_head_t wait;  /* APP要读某个缓冲区,如果无数据,在此休眠 */
	struct list_head stream;
	struct list_head irq;
};

struct myuvc_queue {
    void *mem;
    int count;
    int buf_size;    
    struct myuvc_buffer buffer[32];

	struct urb *urb[32];
	char *urb_buffer[32];
	dma_addr_t urb_dma[32];
	unsigned int urb_size;

	struct list_head mainqueue;   /* 供APP消费用 */
	struct list_head irqqueue;    /* 供底层驱动生产用 */
};

static struct myuvc_queue myuvc_queue;
static struct list_head g_queued_bufs;

static struct usb_device *myuvc_udev;
static int myuvc_bEndpointAddress = 0x82;
static int myuvc_streaming_intf;
static struct v4l2_format myuvc_format;

static struct frame_desc frames[] = {{640, 480}, {160, 120}, {320, 240}, {352, 288}, {800, 600}, {1280, 720}, {1280, 1024}};
static int frame_idx = 1;
static int bBitsPerPixel = 0; /* lsusb -v -d 0x1e4e:  "bBitsPerPixel" */
static int uvc_version = 0x0100; /* lsusb -v -d 0x1b3b: bcdUVC */

static int myuvc_streaming_bAlternateSetting = 11;
static int wMaxPacketSize = 3072;
static int dwMaxVideoFrameSize = 38400;
static int last_fid = -1;
static struct myuvc_streaming_control myuvc_params;

/* 列举支持哪种格式, 参考: uvc_fmts 数组 */
static int myuvc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
    /* 人工查看描述符可知我们用的摄像头只支持1种格式 */
	if (f->index > 0)
		return -EINVAL;

	strlcpy(f->description, "MJPEG", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_MJPEG;  
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    
	return 0;
}
					
/* 列举支持哪种分辨率 */
static int myuvc_enum_framesizes(struct file *file, void *fh,
                     struct v4l2_frmsizeenum *fsize)
{
    if (fsize->index > 0)
        return -EINVAL;

    fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    fsize->discrete.width = 640;
    fsize->discrete.height = 480;
    return 0;
}

/* 返回当前所使用的格式 */
static int myuvc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
    memcpy(f, &myuvc_format, sizeof(myuvc_format));
	return (0);
}

/* 测试驱动程序是否支持某种格式, 强制设置该格式 
 * 参考: uvc_v4l2_try_format
 *       myvivi_vidioc_try_fmt_vid_cap
 */
static int myuvc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
    if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        return -EINVAL;
    }

    if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
        return -EINVAL;
    
    /* 调整format的width, height, 
     * 计算bytesperline, sizeimage
     */

    /* 人工查看描述符, 确定支持哪几种分辨率 */
    f->fmt.pix.width  = frames[frame_idx].width;
    f->fmt.pix.height = frames[frame_idx].height;
    
	f->fmt.pix.bytesperline = (f->fmt.pix.width * bBitsPerPixel) >> 3;
	f->fmt.pix.sizeimage = dwMaxVideoFrameSize;

	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.priv = 0;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    return 0;
}

/* 参考 myvivi_vidioc_s_fmt_vid_cap */
static int myuvc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	int ret = myuvc_try_fmt_vid_cap(file, NULL, f);
	if (ret < 0)
		return ret;

    memcpy(&myuvc_format, f, sizeof(myuvc_format));
    
    return 0;
}

static int myuvc_queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers,
		unsigned int *nplanes, unsigned int sizes[], struct device *alloc_devs[])
{
	struct myuvc *s = vb2_get_drv_priv(vq);
	int bufsize  = PAGE_ALIGN(myuvc_format.fmt.pix.sizeimage);

	dev_dbg(s->dev, "nbuffers=%d\n", *nbuffers);

	/* Need at least 8 buffers */
	if (vq->num_buffers + *nbuffers < 8)
		*nbuffers = 8 - vq->num_buffers;
	*nplanes = 1;
	sizes[0] = PAGE_ALIGN(bufsize);

	dev_dbg(s->dev, "nbuffers=%d sizes[0]=%d\n", *nbuffers, sizes[0]);
	return 0;
}

static void myuvc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	//struct myuvc *s = vb2_get_drv_priv(vb->vb2_queue);
	struct myuvc_frame_buf *buf =
			container_of(vbuf, struct myuvc_frame_buf, vb);
	//unsigned long flags;

	//spin_lock_irqsave(&s->queued_bufs_lock, flags);
	list_add_tail(&buf->list, &g_queued_bufs);
	//spin_unlock_irqrestore(&s->queued_bufs_lock, flags);
}

static int myuvc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{    
	struct myuvc *s = video_drvdata(file);
	
	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, s->vdev.name, sizeof(cap->card));
	usb_make_path(s->udev, cap->bus_info, sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int uvc_control_msg(struct usb_device *udev, u8 request, u16 value,
                           void *data, u16 size)
{
    u8 requesttype;
    unsigned int pipe;
    
    if (request & USB_DIR_IN) {
        requesttype = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN;
        pipe = usb_rcvctrlpipe(udev, 0);
    } else {
        requesttype = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT;
        pipe = usb_sndctrlpipe(udev, 0);
    }
    
    return usb_control_msg(udev, pipe, request & 0x7F, requesttype,
                          value, myuvc_streaming_intf, data, size, 5000);
}

static void myuvc_print_streaming_params(struct myuvc_streaming_control *ctrl)
{
    printk("video params:\n");
    printk("bmHint                   = %d\n", ctrl->bmHint);
    printk("bFormatIndex             = %d\n", ctrl->bFormatIndex);
    printk("bFrameIndex              = %d\n", ctrl->bFrameIndex);
    printk("dwFrameInterval          = %d\n", ctrl->dwFrameInterval);
    printk("wKeyFrameRate            = %d\n", ctrl->wKeyFrameRate);
    printk("wPFrameRate              = %d\n", ctrl->wPFrameRate);
    printk("wCompQuality             = %d\n", ctrl->wCompQuality);
    printk("wCompWindowSize          = %d\n", ctrl->wCompWindowSize);
    printk("wDelay                   = %d\n", ctrl->wDelay);
    printk("dwMaxVideoFrameSize      = %d\n", ctrl->dwMaxVideoFrameSize);
    printk("dwMaxPayloadTransferSize = %d\n", ctrl->dwMaxPayloadTransferSize);
    printk("dwClockFrequency         = %d\n", ctrl->dwClockFrequency);
    printk("bmFramingInfo            = %d\n", ctrl->bmFramingInfo);
    printk("bPreferedVersion         = %d\n", ctrl->bPreferedVersion);
    printk("bMinVersion              = %d\n", ctrl->bMinVersion);
    printk("bMinVersion              = %d\n", ctrl->bMinVersion);
}


/* 参考: uvc_get_video_ctrl 
 (ret = uvc_get_video_ctrl(video, probe, 1, GET_CUR)) 
 static int uvc_get_video_ctrl(struct uvc_video_device *video,
     struct uvc_streaming_control *ctrl, int probe, __u8 query)
 */
static int myuvc_get_streaming_params(struct myuvc_streaming_control *ctrl)
{
	__u8 *data;
	__u16 size;
	int ret;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;

	size = uvc_version >= 0x0110 ? 34 : 26;
	data = kmalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	pipe = (UVC_GET_CUR & 0x80) ? usb_rcvctrlpipe(myuvc_udev, 0)
			      : usb_sndctrlpipe(myuvc_udev, 0);
	type |= (UVC_GET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

	ret = usb_control_msg(myuvc_udev, pipe, UVC_GET_CUR, type, UVC_VS_PROBE_CONTROL << 8,
			0 << 8 | myuvc_streaming_intf, data, size, 5000);

	if (ret < 0)
        goto done;

	ctrl->bmHint = le16_to_cpup((__le16 *)&data[0]);
	ctrl->bFormatIndex = data[2];
	ctrl->bFrameIndex = data[3];
	ctrl->dwFrameInterval = le32_to_cpup((__le32 *)&data[4]);
	ctrl->wKeyFrameRate = le16_to_cpup((__le16 *)&data[8]);
	ctrl->wPFrameRate = le16_to_cpup((__le16 *)&data[10]);
	ctrl->wCompQuality = le16_to_cpup((__le16 *)&data[12]);
	ctrl->wCompWindowSize = le16_to_cpup((__le16 *)&data[14]);
	ctrl->wDelay = le16_to_cpup((__le16 *)&data[16]);
	ctrl->dwMaxVideoFrameSize = get_unaligned_le32(&data[18]);
	ctrl->dwMaxPayloadTransferSize = get_unaligned_le32(&data[22]);

	if (size == 34) {
		ctrl->dwClockFrequency = get_unaligned_le32(&data[26]);
		ctrl->bmFramingInfo = data[30];
		ctrl->bPreferedVersion = data[31];
		ctrl->bMinVersion = data[32];
		ctrl->bMaxVersion = data[33];
	} else {
		//ctrl->dwClockFrequency = video->dev->clock_frequency;
		ctrl->bmFramingInfo = 0;
		ctrl->bPreferedVersion = 0;
		ctrl->bMinVersion = 0;
		ctrl->bMaxVersion = 0;
	}

done:
    kfree(data);
    
    return (ret < 0) ? ret : 0;
}

/* 参考: uvc_v4l2_try_format ∕uvc_probe_video 
 *       uvc_set_video_ctrl(video, probe, 1)
 */
static int myuvc_try_streaming_params(struct myuvc_streaming_control *ctrl)
{
    __u8 *data;
    __u16 size;
    int ret;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
    
	memset(ctrl, 0, sizeof *ctrl);
    
	ctrl->bmHint = 1;	/* dwFrameInterval */
	ctrl->bFormatIndex = 1;
	ctrl->bFrameIndex  = frame_idx + 1;
	ctrl->dwFrameInterval = 333333;


    size = uvc_version >= 0x0110 ? 34 : 26;
    data = kzalloc(size, GFP_KERNEL);
    if (data == NULL)
        return -ENOMEM;

    *(__le16 *)&data[0] = cpu_to_le16(ctrl->bmHint);
    data[2] = ctrl->bFormatIndex;
    data[3] = ctrl->bFrameIndex;
    *(__le32 *)&data[4] = cpu_to_le32(ctrl->dwFrameInterval);
    *(__le16 *)&data[8] = cpu_to_le16(ctrl->wKeyFrameRate);
    *(__le16 *)&data[10] = cpu_to_le16(ctrl->wPFrameRate);
    *(__le16 *)&data[12] = cpu_to_le16(ctrl->wCompQuality);
    *(__le16 *)&data[14] = cpu_to_le16(ctrl->wCompWindowSize);
    *(__le16 *)&data[16] = cpu_to_le16(ctrl->wDelay);
    put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
    put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

    if (size == 34) {
        put_unaligned_le32(ctrl->dwClockFrequency, &data[26]);
        data[30] = ctrl->bmFramingInfo;
        data[31] = ctrl->bPreferedVersion;
        data[32] = ctrl->bMinVersion;
        data[33] = ctrl->bMaxVersion;
    }

	pipe = (UVC_SET_CUR & 0x80) ? usb_rcvctrlpipe(myuvc_udev, 0)
					  : usb_sndctrlpipe(myuvc_udev, 0);
	type |= (UVC_SET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;
	
	ret = usb_control_msg(myuvc_udev, pipe, UVC_SET_CUR, type, UVC_VS_PROBE_CONTROL << 8,
				0 << 8 | myuvc_streaming_intf, data, size, 5000);
	
    kfree(data);
    
    return (ret < 0) ? ret : 0;
    
}


/* 参考: uvc_v4l2_try_format ∕uvc_probe_video 
 *       uvc_set_video_ctrl(video, probe, 1)
 */
static int myuvc_set_streaming_params(struct myuvc_streaming_control *ctrl)
{
    __u8 *data;
    __u16 size;
    int ret;
	__u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
    
    size = uvc_version >= 0x0110 ? 34 : 26;
    data = kzalloc(size, GFP_KERNEL);
    if (data == NULL)
        return -ENOMEM;

    *(__le16 *)&data[0] = cpu_to_le16(ctrl->bmHint);
    data[2] = ctrl->bFormatIndex;
    data[3] = ctrl->bFrameIndex;
    *(__le32 *)&data[4] = cpu_to_le32(ctrl->dwFrameInterval);
    *(__le16 *)&data[8] = cpu_to_le16(ctrl->wKeyFrameRate);
    *(__le16 *)&data[10] = cpu_to_le16(ctrl->wPFrameRate);
    *(__le16 *)&data[12] = cpu_to_le16(ctrl->wCompQuality);
    *(__le16 *)&data[14] = cpu_to_le16(ctrl->wCompWindowSize);
    *(__le16 *)&data[16] = cpu_to_le16(ctrl->wDelay);
    put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
    put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

    if (size == 34) {
        put_unaligned_le32(ctrl->dwClockFrequency, &data[26]);
        data[30] = ctrl->bmFramingInfo;
        data[31] = ctrl->bPreferedVersion;
        data[32] = ctrl->bMinVersion;
        data[33] = ctrl->bMaxVersion;
    }

    pipe = (UVC_SET_CUR & 0x80) ? usb_rcvctrlpipe(myuvc_udev, 0)
                  : usb_sndctrlpipe(myuvc_udev, 0);
    type |= (UVC_SET_CUR & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

    ret = usb_control_msg(myuvc_udev, pipe, UVC_SET_CUR, type, UVC_VS_COMMIT_CONTROL << 8,
            0 << 8 | myuvc_streaming_intf, data, size, 5000);
	
    kfree(data);
    
    return (ret < 0) ? ret : 0;
    
}

static void myuvc_uninit_urbs(void)
{
    int i;
    for (i = 0; i < MYUVC_URBS; ++i) {
        if (myuvc_queue.urb_buffer[i])
        {
            usb_free_coherent(myuvc_udev, myuvc_queue.urb_size, myuvc_queue.urb_buffer[i], myuvc_queue.urb_dma[i]);
            myuvc_queue.urb_buffer[i] = NULL;
        }

        if (myuvc_queue.urb[i])
        {
            usb_free_urb(myuvc_queue.urb[i]);
            myuvc_queue.urb[i] = NULL;
        }
    }
}

/* 参考: uvc_video_complete / uvc_video_decode_isoc */
static void myuvc_video_complete(struct urb *urb)
{
	u8 *src;
    u8 *dest;
	int ret, i;
    int len;
    int maxlen;
    int nbytes;
	__u8 fid;
    struct myuvc_buffer *buf;
    
	switch (urb->status) {
	case 0:
		break;

	default:
		printk("Non-zero status (%d) in video completion handler.\n", urb->status);
		return;
	}

    /* 从irqqueue队列中取出第1个缓冲区 */
	if (!list_empty(&myuvc_queue.irqqueue))
	{
		buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
	}
	else
	{
		buf = NULL;
	}
	for (i = 0; i < urb->number_of_packets; ++i) {
		if (urb->iso_frame_desc[i].status < 0) {
			printk("USB isochronous frame lost (%d).\n", urb->iso_frame_desc[i].status);
			continue;
		}

        src  = urb->transfer_buffer + urb->iso_frame_desc[i].offset;

        len = urb->iso_frame_desc[i].actual_length;
        /* 判断数据是否有效 */
        /* URB数据含义:
         * data[0] : 头部长度
         * data[1] : 错误状态
         */
        if (len < 2 || src[0] < 2 || src[0] > len)
            continue;
        
        /* Skip payloads marked with the error bit ("error frames"). */
        if (src[1] & UVC_STREAM_ERR) {
            printk("Dropping payload (error bit set).\n");
            continue;
        }

		fid = src[1] & UVC_STREAM_FID;

		if (buf == NULL) {
			last_fid = fid;
			continue;
		}

		/* 根据FID判断当前帧的数据是否结束 */
		if (buf->state != VIDEOBUF_ACTIVE) {
			//struct timespec ts;

			if (fid == last_fid) {
				/* 既然你刚开始接收数据，那么FID应该是新的值，不应该等于原来的last_fid */
				//return -ENODATA;
				return;
			}

			/* 表示开始接受第一个数据*/
			buf->state = VIDEOBUF_ACTIVE;
		}

		/* fid != last_fid 表示开始新的一帧 */
		if (fid != last_fid && buf->buf.bytesused != 0) 
		{
			buf->state = VIDEOBUF_DONE;

			/* 从队列中删除，唤醒进程 */
			list_del(&buf->irq);
        	wake_up(&buf->wait);
	
			/*取出下一个buffer*/
			if (!list_empty(&myuvc_queue.irqqueue))
			{
				buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
			}
			else
			{
				buf = NULL;
			}
			continue;
		}
		last_fid = fid;
		
		dest = myuvc_queue.mem + buf->buf.m.offset + buf->buf.bytesused;
		
        /* 除去头部后的数据长度 */
        len -= src[0];

        /* 缓冲区最多还能存多少数据 */
        maxlen = buf->buf.length - buf->buf.bytesused;
        nbytes = min(len, maxlen);

        /* 复制数据 */
        memcpy(dest, src + src[0], nbytes);
        buf->buf.bytesused += nbytes;

        /* 判断一帧数据是否已经全部接收到 */
        if (len > maxlen) {
            buf->state = VIDEOBUF_DONE;
        }
        
        /* Mark the buffer as done if the EOF marker is set. */
        if (src[1] & UVC_STREAM_EOF && buf->buf.bytesused != 0) {
            printk("Frame complete (EOF found).\n");
            if (len == 0)
                printk("EOF in empty payload.\n");
            buf->state = VIDEOBUF_DONE;
        }

		if (buf->state == VIDEOBUF_DONE ||
        buf->state == VIDEOBUF_ERROR)
	    {
	        list_del(&buf->irq);
	        wake_up(&buf->wait);

			if (!list_empty(&myuvc_queue.irqqueue))
			{
				buf = list_first_entry(&myuvc_queue.irqqueue, struct myuvc_buffer, irq);
			}
			else
			{
				buf = NULL;
			}
	    }

	}

    /* 当接收完一帧数据, 
     * 从irqqueue中删除这个缓冲区
     * 唤醒等待数据的进程 
     */
    /* 再次提交URB */
	if ((ret = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		printk("Failed to resubmit video URB (%d).\n", ret);
	}
}

/* 参考: uvc_init_video_isoc */
static int myuvc_alloc_init_urbs(void)
{
	u16 psize;
	u32 size;
	int npackets;
	int i;
	int j;

	struct urb *urb;

	psize = wMaxPacketSize; /* 实时传输端点一次能传输的最大字节数 */
	size  = myuvc_params.dwMaxVideoFrameSize;  /* 一帧数据的最大长度 */
	npackets = DIV_ROUND_UP(size, psize);
	if (npackets > 32)
		npackets = 32;

	size = myuvc_queue.urb_size = psize * npackets;
	
	for (i = 0; i < MYUVC_URBS; ++i) {
		/* 1. 分配usb_buffers */
		
		myuvc_queue.urb_buffer[i] = usb_alloc_coherent(
			myuvc_udev, size,
			GFP_KERNEL | __GFP_NOWARN, &myuvc_queue.urb_dma[i]);

		/* 2. 分配urb */
		myuvc_queue.urb[i] = usb_alloc_urb(npackets, GFP_KERNEL);

		if (!myuvc_queue.urb_buffer[i] || !myuvc_queue.urb[i])
		{
			myuvc_uninit_urbs();
			return -ENOMEM;
		}

	}

	/* 3. 设置urb */
	for (i = 0; i < MYUVC_URBS; ++i) {
		urb = myuvc_queue.urb[i];
		
		urb->dev = myuvc_udev;
		urb->context = NULL;
		urb->pipe = usb_rcvisocpipe(myuvc_udev,myuvc_bEndpointAddress);
		urb->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
		urb->interval = 1;
		urb->transfer_buffer = myuvc_queue.urb_buffer[i];
		urb->transfer_dma = myuvc_queue.urb_dma[i];
		urb->complete = myuvc_video_complete;
		urb->number_of_packets = npackets;
		urb->transfer_buffer_length = size;
		
		for (j = 0; j < npackets; ++j) {
			urb->iso_frame_desc[j].offset = j * psize;
			urb->iso_frame_desc[j].length = psize;
		}
	
	}
	
	return 0;
}


/* 启动传输 
 * 参考:		uvc_start_streaming 
 *			uvc_video_enable(video, 1):
 *          uvc_commit_video
 *          uvc_init_video
 */
static int myuvc_streamon(struct vb2_queue *vq, unsigned int count)
{
    int ret;
	int i;
    ret = myuvc_try_streaming_params(&myuvc_params);
    ret = myuvc_get_streaming_params(&myuvc_params);
    ret = myuvc_set_streaming_params(&myuvc_params);
    printk("myuvc_set_streaming_params ret = %d\n", ret);
	myuvc_print_streaming_params(&myuvc_params);
    usb_set_interface(myuvc_udev, myuvc_streaming_intf, myuvc_streaming_bAlternateSetting);
	
    ret = myuvc_alloc_init_urbs();
    if (ret) {
		printk("myuvc_alloc_init_urbs err : ret = %d\n", ret);
    }
   
	for (i = 0; i < MYUVC_URBS; ++i) {
		if ((ret = usb_submit_urb(myuvc_queue.urb[i], GFP_KERNEL)) < 0) {
			printk("Failed to submit URB %u (%d).\n", i, ret);
			myuvc_uninit_urbs();
			return ret;
		}
	}
	return 0;
}

static void myuvc_streamoff(struct vb2_queue *vq)
{
	struct urb *urb;
	unsigned int i;

    /* 1. kill URB */
	for (i = 0; i < MYUVC_URBS; ++i) {
		if ((urb = myuvc_queue.urb[i]) == NULL)
			continue;
		usb_kill_urb(urb);
	}

    /* 2. free URB */
    myuvc_uninit_urbs();

    /* 3. 设置VideoStreaming Interface为setting 0 */
    usb_set_interface(myuvc_udev, myuvc_streaming_intf, 0);
  
}

static int myuvc_free_buffers(void)
{
    if (myuvc_queue.mem)
    {
        vfree(myuvc_queue.mem);
        memset(&myuvc_queue, 0, sizeof(myuvc_queue));
        myuvc_queue.mem = NULL;
    }
    return 0;
}

static const struct v4l2_ioctl_ops myuvc_ioctl_ops = {
        // 表示它是一个摄像头设备
        .vidioc_querycap          = myuvc_querycap,

        /* 用于列举、获得、测试、设置摄像头的数据的格式 */
        .vidioc_enum_fmt_vid_cap  = myuvc_enum_fmt_vid_cap,
        .vidioc_g_fmt_vid_cap     = myuvc_g_fmt_vid_cap,
        .vidioc_try_fmt_vid_cap   = myuvc_try_fmt_vid_cap,
        .vidioc_s_fmt_vid_cap     = myuvc_s_fmt_vid_cap,
        .vidioc_enum_framesizes   = myuvc_enum_framesizes,
        
        /* 缓冲区操作: 申请/查询/放入队列/取出队列 */
	    .vidioc_reqbufs           = vb2_ioctl_reqbufs, //申请分配缓冲区
		.vidioc_querybuf          = vb2_ioctl_querybuf,
		.vidioc_qbuf              = vb2_ioctl_qbuf,    //设置buf状态为active，调用硬件驱动将缓冲区加入到驱动程序queued_list队列
		.vidioc_dqbuf             = vb2_ioctl_dqbuf,

        /* 查询/获得/设置属性 */
        //.vidioc_queryctrl       = myuvc_vidioc_queryctrl,
        //.vidioc_g_ctrl          = myuvc_vidioc_g_ctrl,
        //.vidioc_s_ctrl          = myuvc_vidioc_s_ctrl,
        
        // 启动/停止
        .vidioc_streamon          = vb2_ioctl_streamon,
        .vidioc_streamoff         = vb2_ioctl_streamoff,   
};

static int myuvc_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	printk("vdev->queue = %p", vdev->queue);
	return 0;
}

static void myuvc_vm_open(struct vm_area_struct *vma)
{
	struct myuvc_buffer *buffer = vma->vm_private_data;
	buffer->vma_use_count++;
}

static void myuvc_vm_close(struct vm_area_struct *vma)
{
	struct myuvc_buffer *buffer = vma->vm_private_data;
	buffer->vma_use_count--;
}

static struct vm_operations_struct myuvc_vm_ops = {
	.open		= myuvc_vm_open,
	.close		= myuvc_vm_close,
};


/* 把缓存映射到APP的空间,以后APP就可以直接操作这块缓存 
 * 参考: uvc_v4l2_mmap
 */
static int myuvc_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct myuvc_buffer *buffer;
    struct page *page;
    unsigned long addr, start, size;
    unsigned int i;
    int ret = 0;

    start = vma->vm_start;
    size = vma->vm_end - vma->vm_start;

    /* 应用程序调用mmap函数时, 会传入offset参数
     * 根据这个offset找出指定的缓冲区
     */
    for (i = 0; i < myuvc_queue.count; ++i) {
        buffer = &myuvc_queue.buffer[i];
        if ((buffer->buf.m.offset >> PAGE_SHIFT) == vma->vm_pgoff)
            break;
    }

    if (i == myuvc_queue.count || size != myuvc_queue.buf_size) {
        ret = -EINVAL;
        goto done;
    }

    /*
     * VM_IO marks the area as being an mmaped region for I/O to a
     * device. It also prevents the region from being core dumped.
     */
    vma->vm_flags |= VM_IO;

    /* 根据虚拟地址找到缓冲区对应的page构体 */
	printk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	printk("myuvc_queue.mem = %p\n", myuvc_queue.mem);
    addr = (unsigned long)myuvc_queue.mem + buffer->buf.m.offset;
    while (size > 0) {
        page = vmalloc_to_page((void *)addr);

        /* 把page和APP传入的虚拟地址挂构 */
        if ((ret = vm_insert_page(vma, start, page)) < 0)
            goto done;

        start += PAGE_SIZE;
        addr += PAGE_SIZE;
        size -= PAGE_SIZE;
    }

    vma->vm_ops = &myuvc_vm_ops;
    vma->vm_private_data = buffer;
    myuvc_vm_open(vma);

done:
    return ret;
}

/* APP调用POLL/select来确定缓存是否就绪(有数据) 
 * 参考 : uvc_v4l2_poll
 */
static unsigned int myuvc_poll(struct file *file, struct poll_table_struct *wait)
{
	struct myuvc_buffer *buf;
	unsigned int mask = 0;
    
    /* 从mainqueuq中取出第1个缓冲区 */

    /*判断它的状态, 如果未就绪, 休眠 */

    if (list_empty(&myuvc_queue.mainqueue)) {
        mask |= POLLERR;
        goto done;
    }
    
    buf = list_first_entry(&myuvc_queue.mainqueue, struct myuvc_buffer, stream);

    poll_wait(file, &buf->wait, wait);
    if (buf->state == VIDEOBUF_DONE ||
        buf->state == VIDEOBUF_ERROR)
        mask |= POLLIN | POLLRDNORM;
    
done:
    return mask;
}

/* 关闭 */
static int myuvc_close(struct file *file)
{
    
	return 0;
}

static const struct v4l2_file_operations myuvc_fops = {
	.owner		= THIS_MODULE,
    .open       = v4l2_fh_open,
    .release    = vb2_fop_release,
    .mmap       = vb2_fop_mmap,
    .ioctl      = video_ioctl2, 	/* V4L2 ioctl handler */
    .poll       = vb2_fop_poll,
};

static void  myuvc_video_device_release(struct video_device *vdev)
{
	struct myuvc *s = container_of(vdev, struct myuvc, vdev);
    v4l2_device_put(&s->v4l2_dev); // 减少 v4l2_device 的引用计数
}

static void myuvc_v4l2_release(struct v4l2_device *v)
{
	struct myuvc *s = container_of(v, struct myuvc, v4l2_dev);

	v4l2_device_unregister(&s->v4l2_dev);
	kfree(s);
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
}

static const struct vb2_ops myuvc_vb2_ops = {
	.queue_setup			= myuvc_queue_setup,
	.buf_queue				= myuvc_buf_queue,
	.start_streaming		= myuvc_streamon,
	.stop_streaming 		= myuvc_streamoff,
	.wait_prepare			= vb2_ops_wait_prepare,
	.wait_finish			= vb2_ops_wait_finish,

};

static int myuvc_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct myuvc *s;
	struct video_device *vdev;
	int ret;

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	/* 需要为结构体指针显示分配内存，否则段错误 */
	vdev->queue = kzalloc(sizeof(struct vb2_queue), GFP_KERNEL);
	if (!vdev->queue) {
	    ret = -ENOMEM;
	}
	
	//mutex_init(vdev->queue->lock);
  	vdev->release    = myuvc_video_device_release;
	vdev->fops 		 = &myuvc_fops;
	vdev->ioctl_ops  = &myuvc_ioctl_ops;

	s = kzalloc(sizeof(struct myuvc), GFP_KERNEL);
	s->vdev = *vdev;

	vdev->queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vdev->queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	vdev->queue->drv_priv = NULL;
	vdev->queue->buf_struct_size = sizeof(struct myuvc_frame_buf);	
	vdev->queue->ops = &myuvc_vb2_ops;
	vdev->queue->mem_ops = &vb2_vmalloc_memops;
	vdev->queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(vdev->queue);

	s->vdev.queue = vdev->queue;
	
	
	printk("s->vedv.queue = %p", s->vdev.queue);
	
	myuvc_streaming_intf = intf->cur_altsetting->desc.bInterfaceNumber;

	myuvc_udev = interface_to_usbdev(intf);

	s->udev = myuvc_udev;
	s->dev = &intf->dev;
	INIT_LIST_HEAD(&g_queued_bufs);
	//s->vdev.release = myuvc_video_device_release; 
	
	usb_set_intfdata(intf, s);
	video_set_drvdata(&s->vdev, s);

	/* Register the v4l2_device structure(辅助作用) */
	s->v4l2_dev.release = myuvc_v4l2_release;
	strcpy(s->v4l2_dev.name, "myuvc_v4l2");
	ret = v4l2_device_register(NULL, &s->v4l2_dev);
	if (ret) {
		printk("Failed to register v4l2-device (%d)\n", ret);
		return -1;
	}

	s->vdev.v4l2_dev = &s->v4l2_dev;

	ret = video_register_device(&s->vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		printk("Failed to register as video device (%d)\n", ret);
		return -1;
	}
	dev_info(s->dev, "Device connected\n");

    return 0;
}

static void myuvc_disconnect(struct usb_interface *intf)
{
#if 0
	struct uvc_device *dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	struct uvc_streaming *stream;
	video_unregister_device(&stream->vdev);
	printk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
#endif

#if 1
	struct v4l2_device *v = usb_get_intfdata(intf);
	struct myuvc *s = container_of(v, struct myuvc, v4l2_dev);
	
	//vb2_queue_release(&s->vb2_queue)


	/* 断开v4l2设备连接 */
	v4l2_device_disconnect(&s->v4l2_dev);
	video_unregister_device(&s->vdev);

	/* 释放v4l2设备 */
	v4l2_device_unregister(&s->v4l2_dev);

	usb_set_intfdata(intf, NULL);
	kfree(s);

	/* 清除接口数据 */


	/* 释放URB资源 */
	myuvc_uninit_urbs();

	/* 释放缓冲区 */
	myuvc_free_buffers();
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);

#endif
}

static struct usb_device_id myuvc_ids[] = {
	/* Generic USB Video Class */
	//{ USB_INTERFACE_INFO(USB_CLASS_VIDEO, 1, UVC_PC_PROTOCOL_UNDEFINED) },  /* VideoControl Interface */
    { USB_INTERFACE_INFO(USB_CLASS_VIDEO, 2, UVC_PC_PROTOCOL_UNDEFINED) },  /* VideoStreaming Interface */
	{}  //UVC_PC_PROTOCOL_UNDEFINED
};

/* 1. 分配usb_driver */
/* 2. 设置 */
static struct uvc_driver myuvc_driver = {
	.driver = {
		.name       = "myuvc",
	    .probe      = myuvc_probe,
	    .disconnect = myuvc_disconnect,
	    .id_table   = myuvc_ids,
	},
};

static int __init myuvc_init(void)
{
    /* 3. 注册 */
    usb_register(&myuvc_driver.driver);
    return 0;
}

static void __exit myuvc_exit(void)
{
    usb_deregister(&myuvc_driver.driver);
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
}

module_init(myuvc_init);
module_exit(myuvc_exit);

MODULE_LICENSE("GPL");
