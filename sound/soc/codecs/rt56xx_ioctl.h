#include <sound/hwdep.h>
#include <linux/ioctl.h>

struct rt56xx_cmd
{
	size_t number;
	int __user *buf;
};

enum 
{
	RT_READ_CODEC_REG_IOCTL = _IOR('R', 0x01, struct rt56xx_cmd),
	RT_WRITE_CODEC_REG_IOCTL = _IOW('R', 0x01, struct rt56xx_cmd),
	RT_READ_ALL_CODEC_REG_IOCTL = _IOR('R', 0x02, struct rt56xx_cmd),
	RT_READ_CODEC_INDEX_IOCTL = _IOR('R', 0x03, struct rt56xx_cmd),
	RT_WRITE_CODEC_INDEX_IOCTL = _IOW('R', 0x03, struct rt56xx_cmd),
	RT_READ_CODEC_DSP_IOCTL = _IOR('R', 0x04, struct rt56xx_cmd),
	RT_WRITE_CODEC_DSP_IOCTL = _IOW('R', 0x04, struct rt56xx_cmd),
	RT_SET_CODEC_HWEQ_IOCTL = _IOW('R', 0x05, struct rt56xx_cmd),
	RT_GET_CODEC_HWEQ_IOCTL = _IOR('R', 0x05, struct rt56xx_cmd),
	RT_SET_CODEC_SPK_VOL_IOCTL = _IOW('R', 0x06, struct rt56xx_cmd),
	RT_GET_CODEC_SPK_VOL_IOCTL = _IOR('R', 0x06, struct rt56xx_cmd),
	RT_SET_CODEC_MIC_GAIN_IOCTL = _IOW('R', 0x07, struct rt56xx_cmd),
	RT_GET_CODEC_MIC_GAIN_IOCTL = _IOR('R', 0x07, struct rt56xx_cmd),
	RT_SET_CODEC_3D_SPK_IOCTL = _IOW('R', 0x08, struct rt56xx_cmd),
	RT_GET_CODEC_3D_SPK_IOCTL = _IOR('R', 0x08, struct rt56xx_cmd),
	RT_SET_CODEC_MP3PLUS_IOCTL = _IOW('R', 0x09, struct rt56xx_cmd),
	RT_GET_CODEC_MP3PLUS_IOCTL = _IOR('R', 0x09, struct rt56xx_cmd),
	RT_SET_CODEC_3D_HEADPHONE_IOCTL = _IOW('R', 0x0A, struct rt56xx_cmd),
	RT_GET_CODEC_3D_HEADPHONE_IOCTL = _IOR('R', 0x0A, struct rt56xx_cmd),
	RT_SET_CODEC_BASS_BACK_IOCTL = _IOW('R', 0x0B, struct rt56xx_cmd),
	RT_GET_CODEC_BASS_BACK_IOCTL = _IOR('R', 0x0B, struct rt56xx_cmd),
	RT_SET_CODEC_DIPOLE_SPK_IOCTL = _IOW('R', 0x0C, struct rt56xx_cmd),
	RT_GET_CODEC_DIPOLE_SPK_IOCTL = _IOR('R', 0x0C, struct rt56xx_cmd),
	RT_SET_CODEC_DRC_AGC_ENABLE_IOCTL = _IOW('R', 0x0D, struct rt56xx_cmd),
	RT_GET_CODEC_DRC_AGC_ENABLE_IOCTL = _IOR('R', 0x0D, struct rt56xx_cmd),
	RT_SET_CODEC_DSP_MODE_IOCTL = _IOW('R', 0x0E, struct rt56xx_cmd),
	RT_GET_CODEC_DSP_MODE_IOCTL = _IOR('R', 0x0E, struct rt56xx_cmd),
	RT_SET_CODEC_WNR_ENABLE_IOCTL = _IOW('R', 0x0F, struct rt56xx_cmd),
	RT_GET_CODEC_WNR_ENABLE_IOCTL = _IOR('R', 0x0F, struct rt56xx_cmd),
	RT_SET_CODEC_DRC_AGC_PAR_IOCTL = _IOW('R', 0x10, struct rt56xx_cmd),
	RT_GET_CODEC_DRC_AGC_PAR_IOCTL = _IOR('R', 0x10, struct rt56xx_cmd),
	RT_SET_CODEC_DIGI_BOOST_GAIN_IOCTL = _IOW('R', 0x11, struct rt56xx_cmd),
	RT_GET_CODEC_DIGI_BOOST_GAIN_IOCTL = _IOR('R', 0x11, struct rt56xx_cmd),
	RT_SET_CODEC_NOISE_GATE_IOCTL = _IOW('R', 0x12, struct rt56xx_cmd),
	RT_GET_CODEC_NOISE_GATE_IOCTL = _IOR('R', 0x12, struct rt56xx_cmd),
	RT_SET_CODEC_DRC_AGC_COMP_IOCTL = _IOW('R', 0x13, struct rt56xx_cmd),
	RT_GET_CODEC_DRC_AGC_COMP_IOCTL = _IOR('R', 0x13, struct rt56xx_cmd),
	RT_GET_CODEC_ID = _IOR('R', 0x30, struct rt56xx_cmd),
};
