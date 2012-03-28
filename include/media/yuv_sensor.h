#ifndef ___YUV_SENSOR_H__
#define ___YUV_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/*-------------------------------------------Important---------------------------------------
 * for changing the SENSOR_NAME, you must need to change the owner of the device. For example
 * Please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * -------------------------------------------Important--------------------------------------
*/

#define SENSOR_NAME     "mt9d115"
#define DEV(x)          "/dev/"x
#define SENSOR_PATH     DEV(SENSOR_NAME)
#define LOG_NAME(x)     "ImagerODM-"x
#define LOG_TAG         LOG_NAME(SENSOR_NAME)

#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */
#define MAX_FACEDETECT_WINDOWS  5

#define SENSOR_IOCTL_SET_MODE           _IOW('o', 1, struct sensor_mode)
#define SENSOR_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define SENSOR_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define SENSOR_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define SENSOR_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define SENSOR_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define SENSOR_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)
#define SENSOR_IOCTL_SET_CAMERA         _IOW('o', 8, __u8)

enum {
    ASUS_CUSTOM_IOCTL_NUMBASE = 40,
    ASUS_CUSTOM_IOCTL_AF_SET,
    ASUS_CUSTOM_IOCTL_GET_ID,
    ASUS_CUSTOM_IOCTL_SET_EV,
    ASUS_CUSTOM_IOCTL_GET_EV,
    ASUS_CUSTOM_IOCTL_AF_GET,
    ASUS_CUSTOM_IOCTL_GET_ET,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_MODE,
    ASUS_CUSTOM_IOCTL_SET_TOUCH_AF,
    ASUS_CUSTOM_IOCTL_SET_FLASH_STATUS,
    ASUS_CUSTOM_IOCTL_GET_FLASH_STATUS,
    ASUS_CUSTOM_IOCTL_GET_ISO,
    ASUS_CUSTOM_IOCTL_SET_SCENEMODE,
    ASUS_CUSTOM_IOCTL_SET_ISO,
    ASUS_CUSTOM_IOCTL_SET_FRAME_RATE,
    ASUS_CUSTOM_IOCTL_SET_FLICKERING,
    ASUS_CUSTOM_IOCTL_SET_SHADING,
    ASUS_CUSTOM_IOCTL_SET_FACEDETECT,
    ASUS_CUSTOM_IOCTL_GET_FACEDETECT,
    ASUS_CUSTOM_IOCTL_SET_CONTINUOUS_AF,
    ASUS_CUSTOM_IOCTL_SET_AE_WINDOW,
    ASUS_CUSTOM_IOCTL_SET_WDR,
    ASUS_CUSTOM_IOCTL_SET_AE_LOCK,
    ASUS_CUSTOM_IOCTL_SET_AWB_LOCK,
    ASUS_CUSTOM_IOCTL_GET_AE_LOCK,
    ASUS_CUSTOM_IOCTL_GET_AWB_LOCK,
    ASUS_CUSTOM_IOCTL_INITIAL,
    ASUS_CUSTOM_IOCTL_SET_AF_CONTROL
};

enum {
    ASUS_CUSTOM_IOCTL_FW_UPDATE_INIT = 100,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_PROGRAM,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_DATA,
};

#define  AF_CMD_START 0
#define  AF_CMD_ABORT 1
#define  AF_CMD_SET_POSITION  2
#define  AF_CMD_SET_WINDOW_POSITION 3
#define  AF_CMD_SET_WINDOW_SIZE 4
#define  AF_CMD_SET_AFMODE  5
#define  AF_CMD_SET_CAF 6
#define  AF_CMD_GET_AF_STATUS 7

typedef struct
{
    int cmd;
    int data;
} custom_af_cmd_package;

typedef struct
{
    int win_w;
    int win_h;
    int win_x;
    int win_y;
    int zoom;
} custom_touch_af_cmd_package;

typedef struct
{
    unsigned int exposure;
    unsigned int vts;
} custom_et_value_package;

typedef struct
{
    int x;
    int y;
    int width;
    int height;
} AsusUserFaceDetectWindow;

typedef struct
{
    int face_num;
    AsusUserFaceDetectWindow *face_window;
} custom_face_detection_cmd_package;

typedef struct
{
    int cmd;
    unsigned int flash_rom_start_address;
    unsigned int program_size;
    char* binfile_path;
} custom_fw_update_rom_package;

typedef struct
{
    unsigned int start_address;
    unsigned int data_byte;
    unsigned int data_length;
    unsigned int *data;
} custom_fw_update_data_package;

typedef enum {
    E_M6MO_Status_Parameter,            /**< Parameter mode */
    E_M6MO_Status_Monitor,              /**< Monitor mode */
    E_M6MO_Status_Capture,              /**< Capture mode */
} E_M6MO_Status;

#define SENSOR_CUSTOM_IOCTL_SET_AF_MODE       _IOW('o', ASUS_CUSTOM_IOCTL_AF_SET, custom_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_GET_AF_MODE       _IOWR('o', ASUS_CUSTOM_IOCTL_AF_GET, custom_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_GET_ID            _IOW('o', ASUS_CUSTOM_IOCTL_GET_ID, __u16)
#define SENSOR_CUSTOM_IOCTL_SET_EV            _IOW('o', ASUS_CUSTOM_IOCTL_SET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_GET_EV            _IOR('o', ASUS_CUSTOM_IOCTL_GET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_GET_ET            _IOR('o', ASUS_CUSTOM_IOCTL_GET_ET, __s16)
#define SENSOR_CUSTOM_IOCTL_FW_UPDATE_MODE    _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_MODE, __u16)
#define SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF      _IOWR('o', ASUS_CUSTOM_IOCTL_SET_TOUCH_AF, custom_touch_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_SET_FLASH_STATUS  _IOWR('o', ASUS_CUSTOM_IOCTL_SET_FLASH_STATUS, __u16)
#define SENSOR_CUSTOM_IOCTL_GET_FLASH_STATUS  _IOWR('o', ASUS_CUSTOM_IOCTL_GET_FLASH_STATUS, __u16)
#define SENSOR_CUSTOM_IOCTL_GET_ISO           _IOR('o', ASUS_CUSTOM_IOCTL_GET_ISO, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_SCENEMODE     _IOW('o', ASUS_CUSTOM_IOCTL_SET_SCENEMODE, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_ISO           _IOW('o', ASUS_CUSTOM_IOCTL_SET_ISO, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_FRAME_RATE    _IOW('o', ASUS_CUSTOM_IOCTL_SET_FRAME_RATE, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_FLICKERING    _IOW('o', ASUS_CUSTOM_IOCTL_SET_FLICKERING, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_SHADING       _IOW('o', ASUS_CUSTOM_IOCTL_SET_SHADING, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_FACEDETECT    _IOWR('o', ASUS_CUSTOM_IOCTL_SET_FACEDETECT, custom_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_GET_FACEDETECT    _IOWR('o', ASUS_CUSTOM_IOCTL_GET_FACEDETECT, custom_face_detection_cmd_package)
#define SENSOR_CUSTOM_IOCTL_SET_CONTINUOUS_AF _IOW('o', ASUS_CUSTOM_IOCTL_SET_CONTINUOUS_AF, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_AE_WINDOW     _IOW('o', ASUS_CUSTOM_IOCTL_SET_AE_WINDOW, __u16)
#define SENSOR_CUSTOM_IOCTL_SET_WDR           _IOW('o', ASUS_CUSTOM_IOCTL_SET_WDR, __u16)
#define SENSOR_CUSTOM_IOCTL_FW_UPDATE_INIT    _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_INIT, custom_fw_update_rom_package)
#define SENSOR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_PROGRAM, custom_fw_update_rom_package)
#define SENSOR_CUSTOM_IOCTL_FW_UPDATE_DATA    _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_DATA, custom_fw_update_data_package)
#define SENSOR_CUSTOM_IOCTL_SET_AE_LOCK       _IOW('o', ASUS_CUSTOM_IOCTL_SET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_SET_AWB_LOCK      _IOW('o', ASUS_CUSTOM_IOCTL_SET_AWB_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AE_LOCK       _IOWR('o', ASUS_CUSTOM_IOCTL_GET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AWB_LOCK      _IOWR('o', ASUS_CUSTOM_IOCTL_GET_AWB_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_INITIAL           _IOWR('o', ASUS_CUSTOM_IOCTL_INITIAL, __u32)
#define SENSOR_CUSTOM_IOCTL_SET_AF_CONTROL _IOW('o', ASUS_CUSTOM_IOCTL_SET_AF_CONTROL, __s16)

enum {
    YUV_ColorEffect = 0,
    YUV_Whitebalance,
    YUV_SceneMode,
    YUV_FlashType
};

enum {
    YUV_ColorEffect_Invalid = 0,
    YUV_ColorEffect_Aqua,
    YUV_ColorEffect_Blackboard,
    YUV_ColorEffect_Mono,
    YUV_ColorEffect_Negative,
    YUV_ColorEffect_None,
    YUV_ColorEffect_Posterize,
    YUV_ColorEffect_Sepia,
    YUV_ColorEffect_Solarize,
    YUV_ColorEffect_Whiteboard,
    YUV_ColorEffect_Vivid,
    YUV_ColorEffect_WaterColor
};

enum {
    YUV_Whitebalance_Invalid = 0,
    YUV_Whitebalance_Auto,
    YUV_Whitebalance_Incandescent,
    YUV_Whitebalance_Fluorescent,
    YUV_Whitebalance_WarmFluorescent,
    YUV_Whitebalance_Daylight,
    YUV_Whitebalance_CloudyDaylight,
    YUV_Whitebalance_Shade,
    YUV_Whitebalance_Twilight,
    YUV_Whitebalance_Custom
};

enum {
    YUV_SceneMode_Invalid = 0,
    YUV_SceneMode_Auto,
    YUV_SceneMode_Action,
    YUV_SceneMode_Portrait,
    YUV_SceneMode_Landscape,
    YUV_SceneMode_Beach,
    YUV_SceneMode_Candlelight,
    YUV_SceneMode_Fireworks,
    YUV_SceneMode_Night,
    YUV_SceneMode_NightPortrait,
    YUV_SceneMode_Party,
    YUV_SceneMode_Snow,
    YUV_SceneMode_Sports,
    YUV_SceneMode_SteadyPhoto,
    YUV_SceneMode_Sunset,
    YUV_SceneMode_Theatre,
    YUV_SceneMode_Barcode,
    YUV_SceneMode_BackLight
};

enum {
    YUV_FlashControlOn = 0,
    YUV_FlashControlOff,
    YUV_FlashControlAuto,
    YUV_FlashControlRedEyeReduction,
    YUV_FlashControlFillin,
    YUV_FlashControlTorch
};

enum {
    YUV_ISO_AUTO = 0,
    YUV_ISO_50 = 1,
    YUV_ISO_100 = 2,
    YUV_ISO_200 = 3,
    YUV_ISO_400 = 4,
    YUV_ISO_800 = 5,
    YUV_ISO_1600 = 6,
};

enum {
    YUV_FRAME_RATE_AUTO = 0,
    YUV_FRAME_RATE_30 = 1,
    YUV_FRAME_RATE_15 = 2,
    YUV_FRAME_RATE_7 = 3,
};

enum {
    YUV_ANTIBANGING_OFF = 0,
    YUV_ANTIBANGING_AUTO = 1,
    YUV_ANTIBANGING_50HZ = 2,
    YUV_ANTIBANGING_60HZ = 3,
};

enum {
    INT_STATUS_MODE = 0x01,
    INT_STATUS_AF = 0x02,
    INT_STATUS_ZOOM = 0x04,
    INT_STATUS_CAPTURE = 0x08,
    INT_STATUS_FRAMESYNC = 0x10,
    INT_STATUS_FD = 0x20,
    INT_STATELENS_INIT = 0x40,
    INT_STATUS_SOUND = 0x80,
};

enum {
    TOUCH_STATUS_OFF= 0,
    TOUCH_STATUS_ON,
    TOUCH_STATUS_DONE,
};

enum {
    CALIBRATION_ISP_POWERON_FAIL = 0,
    CALIBRATION_ISP_INIT_FAIL,
    CALIBRATION_ISP_MONITOR_FAIL,
    CALIBRATION_LIGHT_SOURCE_FAIL,
    CALIBRATION_LIGHT_SOURCE_OK,
    CALIBRATION_ISP_CAPTURE_FAIL,
    CALIBRATION_ISP_CHECKSUM_FAIL,
    CALIBRATION_ISP_PGAIN_FAIL,
    CALIBRATION_ISP_GOLDEN_FAIL,
    CALIBRATION_ISP_FAIL,
    CALIBRATION_ISP_OK,
};

struct sensor_mode {
    int xres;
    int yres;
};

#ifdef __KERNEL__
struct yuv_sensor_platform_data {
    int (*power_on)(void);
    int (*power_off)(void);
};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_H__ */

