#ifndef _KD_IMGSENSOR_DATA_H
#define _KD_IMGSENSOR_DATA_H

/* #include "../camera/kd_camera_hw.h" */
#include "kd_camera_feature.h"

#define SENSOR_CLOCK_POLARITY_HIGH     0
#define SENSOR_CLOCK_POLARITY_LOW      1
/* #define CONFIG_COMPAT */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif



/*************************************************
*
**************************************************/
/* In KERNEL mode,SHOULD be sync with mediatype.h */
/* CHECK before remove or modify */
/* #undef BOOL */
/* #define BOOL signed int */
#ifndef _MEDIA_TYPES_H
typedef unsigned char MUINT8;
typedef unsigned short MUINT16;
typedef unsigned int MUINT32;
typedef signed char MINT8;
typedef signed short MINT16;
typedef signed int MINT32;
#endif

/*******************************************************************************
*
********************************************************************************/
/*  */
/* msdk_isp_exp.h */
/*  */
#define BASEGAIN 0x40
#define BASEGAIN_SHIFT 6

typedef enum {
	ISP_DRIVING_2MA = 0,
	ISP_DRIVING_4MA,
	ISP_DRIVING_6MA,
	ISP_DRIVING_8MA
} ISP_DRIVING_CURRENT_ENUM;


/*******************************************************************************
*
********************************************************************************/
//
//msdk_isp_feature_exp.h
//

//#ifndef __MSDK_ISP_Feature_H
//#define __MSDK_ISP_Feature_H
//
//#include "msdk_comm_define_exp.h"
//
//#define DIGITAL_ZOOM_STEP_TIMES   100     /* times that will be multiplied on ZoomStep that can minimize the variance
//                                         when transfer from zoom factor to zoom step. */
//#define DIGITAL_ZOOM_TOTAL_STEP   40      /* total digital zoom step */
//
//typedef enum
//{ // mapping to VideoProcAmp in Windows Mobile Null Camera
//  ISP_FEATURE_BRIGHTNESS=MSDK_CAMERA_FEATURE_START,
//  ISP_FEATURE_CONTRAST,
//  ISP_FEATURE_HUE,
//  ISP_FEATURE_SATURATION,
//  ISP_FEATURE_SHARPNESS,
//  ISP_FEATURE_GAMMA,
//  ISP_FEATURE_COLORENABLE,
//  ISP_FEATURE_WHITEBALANCE,
//  ISP_FEATURE_BACKLIGHT_COMPENSATION,
//  ISP_FEATURE_GAIN,
//
//  // mapping to CameraControl in Windows Mobile Null Camera
//  ISP_FEATURE_PAN,
//  ISP_FEATURE_TILT,
//  ISP_FEATURE_ROLL,
//  ISP_FEATURE_DIGITAL_ZOOM,
//  ISP_FEATURE_EXPOSURE,   /* EV Compensation ?? */
//  ISP_FEATURE_IRIS,
//  ISP_FEATURE_FOCUS,
//  ISP_FEATURE_FLASH,
//
//  // Media Tek. Specific Features, should mapped with the property of custom camera,
//  // Ref src\common\inc\camera_custom_cs.h
//  ISP_FEATURE_IMAGE_EFFECT,
//  ISP_FEATURE_BANDING_FREQ,
//  ISP_FEATURE_DSC_MODE,
//  ISP_FEATURE_ISO,
//  ISP_FEATURE_AE_METERING_MODE,
//  ISP_FEATURE_AF_KEY,
//  ISP_FEATURE_AF_RANGE_CONTROL,
//  ISP_FEATURE_AF_OPERATION,
//  ISP_FEATURE_BINNING_MODE,
//  ISP_FEATURE_VIDEO_SCENE_MODE,
//
//  // The following feature are not implemented now.
//  ISP_FEATURE_MANUAL_FOCUS_DIR,
//  ISP_FEATURE_YUV_AF_STATUS,
//  ISP_FEATURE_FAST_ZOOM,
//  ISP_FEATURE_CONTINUE_SHOT,  /* flash will be turn off if continue_shot is enabled */
//  ISP_FEATURE_MAX
//} MSDK_ISP_FEATURE_ENUM;
//
//typedef enum
//{
//  ISP_EXTEND_FEATURE_BEGIN=MSDK_ISP_EXTEND_FEATURE_START,
//  ISP_EXTEND_FEATURE_GET_CAMERA_TUNNING_PARA,
//  ISP_EXTEND_FEATURE_SET_FD_RESULT_FOR_AF,
//  ISP_EXTEND_FEATURE_RESTORE_AE_AWB,
//  ISP_EXTEND_FEATURE_SET_SELECTED_SENSOR,
//  ISP_EXTEND_FEATURE_END
//} MSDK_ISP_EXTEND_FEATURE_ENUM;
//
//enum
//{
//    CAM_SRC_ISP = 0,
//    CAM_SRC_MEM,
//    CAM_NO_OF_SRC
//};
//
///* ev compensation enum */
//typedef enum
//{
//    CAM_EV_NEG_4_3 = 0,
//    CAM_EV_NEG_3_3,
//    CAM_EV_NEG_2_3,
//    CAM_EV_NEG_1_3,
//    CAM_EV_ZERO,
//    CAM_EV_POS_1_3,
//    CAM_EV_POS_2_3,
//    CAM_EV_POS_3_3,
//    CAM_EV_POS_4_3,
//    CAM_EV_NIGHT_SHOT,
//    CAM_NO_OF_EV
//} CAM_EV_COMPENSATION_ENUM;
//
///* White balance enum */
//typedef enum
//{
//    CAM_WB_AUTO = 0,
//    CAM_WB_CLOUD,
//    CAM_WB_DAYLIGHT,
//    CAM_WB_INCANDESCENCE,
//    CAM_WB_FLUORESCENT,
//    CAM_WB_TUNGSTEN,
//    CAM_WB_MANUAL,
//    CAM_NO_OF_WB
//} CAM_WB_MODE_ENUM;
//
///* camera image rotate/mirror enum */
//typedef enum
//{
//    CAM_IMAGE_NORMAL = 0,
//    CAM_IMAGE_ROTATE_90,
//    CAM_IMAGE_ROTATE_180,
//    CAM_IMAGE_ROTATE_270,
//    CAM_IMAGE_MIRROR,
//    CAM_IMAGE_MIRROR_ROTATE_90,
//    CAM_IMAGE_MIRROR_ROTATE_180,
//    CAM_IMAGE_MIRROR_ROTATE_270,
//    CAM_NO_OF_IMAGE_ROTATE
//} CAM_IMAGE_ROTATE_MIRROR_ENUM;
//
//
///* banding freq enum */
//typedef enum
//{
//    CAM_BANDING_50HZ = 0,
//    CAM_BANDING_60HZ,
//    CAM_NO_OF_BANDING
//} CAM_BANDING_FREQ_ENUM;
//
///* Special Effect for Preview, Capture and MPEG4 Encode */
//typedef enum
//{
//    CAM_EFFECT_ENC_NORMAL = 0,
//    CAM_EFFECT_ENC_GRAYSCALE,
//    CAM_EFFECT_ENC_SEPIA,
//    CAM_EFFECT_ENC_SEPIAGREEN,
//    CAM_EFFECT_ENC_SEPIABLUE,
//    CAM_EFFECT_ENC_COLORINV,
//    CAM_EFFECT_ENC_GRAYINV,
//    CAM_EFFECT_ENC_BLACKBOARD,
//    CAM_EFFECT_ENC_WHITEBOARD,
//    CAM_EFFECT_ENC_COPPERCARVING,
//    CAM_EFFECT_ENC_EMBOSSMENT,
//    CAM_EFFECT_ENC_BLUECARVING,
//    CAM_EFFECT_ENC_JEAN,
//    CAM_EFFECT_ENC_SKETCH,
//    CAM_EFFECT_ENC_OIL,
//    CAM_NO_OF_EFFECT_ENC
//} CAM_EFFECT_ENUM;
//
///* Special Effect for Playback and Post-processing */
//enum
//{
//    IMG_EFFECT_NORMAL = 0,
//    IMG_EFFECT_GRAYSCALE,
//    IMG_EFFECT_SEPIA,
//    IMG_EFFECT_SEPIAGREEN,
//    IMG_EFFECT_SEPIABLUE,
//    IMG_EFFECT_COLORINV,
//    IMG_EFFECT_GRAYINV,
//    IMG_EFFECT_WATERCOLOR,
//    IMG_EFFECT_LIGHTBLUR,
//    IMG_EFFECT_BLUR,
//    IMG_EFFECT_STRONGBLUR,
//    IMG_EFFECT_UNSHARP,
//    IMG_EFFECT_SHARPEN,
//    IMG_EFFECT_MORESHARPEN,
//    IMG_EFFECT_MEDIAN,
//    IMG_EFFECT_DILATION,
//    IMG_EFFECT_EROSION,
//    NO_OF_IMG_EFFECT
//};
//
///* Adjustment Effect for Preview, Capture and MPEG4 Encode */
//enum
//{
//    CAM_ADJ_ENC_NONE = 0,
//    CAM_ADJ_ENC_HUE,
//    CAM_ADJ_ENC_SATURATION,
//    CAM_ADJ_ENC_GAMMA,
//    CAM_ADJ_ENC_ADJR,
//    CAM_ADJ_ENC_ADJG,
//    CAM_ADJ_ENC_ADJB,
//    CAM_ADJ_ENC_CONTRAST,
//    CAM_ADJ_ENC_BRIGHTNESS,
//    CAM_NO_OF_ADJ_ENC
//};
//
///* flash mode enum */
//typedef enum
//{
//    CAM_FLASH_OFF = 0,
//    CAM_FLASH_ON,
//    CAM_FLASH_AUTO,
//    CAM_FLASH_REDEYE,
//    CAM_NO_OF_FLASH
//} CAM_FLASH_MODE_ENUM;
//
////flash type enum
//typedef enum
//{
//    FLASHLIGHT_NONE = 0,
//  FLASHLIGHT_LED_ONOFF,           // LED always on/off
//  FLASHLIGHT_LED_CONSTANT,        // CONSTANT type LED
//  FLASHLIGHT_LED_PEAK,            // peak strobe type LED
//  FLASHLIGHT_XENON_SCR,           // SCR strobe type Xenon
//  FLASHLIGHT_XENON_IGBT           // IGBT strobe type Xenon
//} FLASHLIGHT_TYPE_ENUM;
//
///* preview frame rate enum */
//enum
//{
//    CAMERA_PERVIEW_FRAME_RATE = 0,
//    VIDEO_PERVIEW_FRAME_RATE = 1
//};
//
///* camera operation enum */
//enum
//{
//    CAM_DSC_MODE = 0,
//    CAM_AE_METERING,
//    CAM_AF_KEY,
//    CAM_MANUAL_FOCUS_DIR,
//    CAM_FLASH_MODE,
//    CAM_AF_OPERATION_MODE,
//    CAM_AF_RANGE,
//    CAM_NO_OF_CAMERA_OPERATION
//};
//
///* ae metering enum */
//typedef enum
//{
//    CAM_AE_METER_AUTO = 0,
//    CAM_AE_METER_SPOT,
//    CAM_AE_METER_CENTRAL,
//    CAM_AE_METER_AVERAGE,
//    CAM_NO_OF_AE_METERING
//} CAM_AE_METERING_MODE_ENUM;
//
///* af key enum */
//typedef enum
//{
//    CAM_AF_RELEASE = 0,
//    CAM_AF_ONE_TIME,
//    CAM_AF_CONTINUE,
//    CAM_NO_OF_AF_KEY
//} CMA_AF_KEY_ENUM;
//
///* af operation mode enum */
//typedef enum
//{
//  CAM_AF_OFF,
//  CAM_AF_SINGLE_ZONE,
//  CAM_AF_MULTI5_ZONE,
//  CAM_AF_CONTINUOUS,
//    CAM_AF_MANUAL_MODE,
//  CAM_NO_OF_AF_OPERATION_MODE
//} CAM_AF_OPERATION_MODE_ENUM;
//
///* af range enum */
//typedef enum
//{
//    CAM_AF_RANGE_AUTO = 0,
//    CAM_AF_RANGE_MACRO,
//    CAM_AF_RANGE_LANDSCAPE,
//    CAM_NO_OF_AF_RANGE
//} CAM_AF_RANGE_ENUM;
//
///* af preview rotate enum, TBD */
//enum
//{
//    CAM_AF_PREVIEW_ROTATE_0 = 0,
//    CAM_AF_PREVIEW_ROTATE_90,
//    CAM_AF_PREVIEW_ROTATE_180,
//    CAM_AF_PREVIEW_ROTATE_270,
//    CAM_NO_OF_AF_PREVIEW_ROTATE
//} ;
//
///* menual focus direction enum */
//enum
//{
//    CAM_FOCUS_STOP = 0,
//    CAM_FOCUS_FORWARD,
//    CAM_FOCUS_BACKWARD,
//    CAM_FOCUS_FORWARD_CONTINUE,
//    CAM_FOCUS_BACKWARD_CONTINUE,
//    CAM_NO_OF_FOCUS_DIR
//};
//
///* dsc mode enum */
//typedef enum
//{
//    CAM_AUTO_DSC_MODE = 0,
//    CAM_PORTRAIT_MODE,
//    CAM_LANDSCAPE_MODE,
//    CAM_SPORT_MODE,
//    CAM_FLOWER_MODE,
//    CAM_NIGHTSCENE_MODE,
//  CAM_DOCUMENT_MODE,
//  CAM_ISO_ANTI_HAND_SHAKE_MODE,
//  CAM_ISO100_MODE,
//  CAM_ISO200_MODE,
//  CAM_ISO400_MODE,
//  CAM_ISO800_MODE,
//  CAM_ISO1600_MODE,
//  CAM_VIDEO_AUTO_MODE,
//  CAM_VIDEO_NIGHT_MODE,
//    CAM_NO_OF_SCENE_MODE_MAX
//} CAM_SCENE_MODE_ENUM;
//
//enum
//{
//  CAM_ADJUST_LOW_LEVEL=0,
//  CAM_ADJUST_MED_LEVEL,
//  CAM_ADJUST_HIGH_LEVEL,
//  CAM_NO_OF_ADJUST
//};
//
///* multishot enum */
//typedef enum
//{
//  MULTISHOT_FALSE=0,
//  MULTISHOT_FIRST,
//  MULTISHOT_NOT_FIRST,
//  MULTISHOT_END
//} MULTISHOT_ENUM;

enum
{
    IMAGE_NORMAL=0,
    IMAGE_H_MIRROR,
    IMAGE_V_MIRROR,
    IMAGE_HV_MIRROR 
};

//typedef enum
//{
//  AF_ZONE_STATUS_IDLE=0,
//  AF_ZONE_STATUS_INACTIVE,
//  AF_ZONE_STATUS_ACTIVE   ,
//  AF_ZONE_STATUS_FOUND_SUCCESS,
//  AF_ZONE_STATUS_FOUND_NOFOCUS,
//  AF_ZONE_STATUS_SWITCH_MACRO
//} MSDK_ISP_AF_STATUS_ENUM;
//
//typedef enum
//{
//  AE_BRACKET_0 =0,
//  AE_BRACKET_1,
//  AE_BRACKET_2,
//  AE_BRACKET_NON
//} MSDK_ISP_AE_BRACKET_ENUM;

// defined the enum for enumerating the ISO/Binning information about each ISO mode.
typedef enum {
	MSDK_SCENARIO_ID_CAMERA_PREVIEW = 0,
	MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG,
	MSDK_SCENARIO_ID_VIDEO_PREVIEW,
	MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO,
	MSDK_SCENARIO_ID_CAMERA_ZSD,
	MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW,
	MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE,
	MSDK_SCENARIO_ID_CAMERA_3D_VIDEO,
	MSDK_SCENARIO_ID_TV_OUT,
	MSDK_SCENARIO_ID_SLIM_VIDEO,
	MSDK_SCENARIO_ID_CUSTOM1,
	MSDK_SCENARIO_ID_CUSTOM2,
	MSDK_SCENARIO_ID_CUSTOM3,
	MSDK_SCENARIO_ID_CUSTOM4,
	MSDK_SCENARIO_ID_CUSTOM5,
	MSDK_SCENARIO_ID_MAX,
}   MSDK_SCENARIO_ID_ENUM;

typedef enum
{
  ISO_100_MODE =0,
  ISO_200_MODE,
  ISO_400_MODE,
  ISO_800_MODE,
  ISO_1600_MODE,
  ISO_MAX_MODE					  
} ACDK_ISP_ISO_ENUM;

typedef struct
{
  MUINT32              MaxWidth;
  MUINT32              MaxHeight;
  MINT32            ISOSupported;
  MINT32                BinningEnable;
} ACDK_ISP_BINNING_INFO_STRUCT, *PACDK_ISP_BINNING_INFO_STRUCT;
//#endif /* __MSDK_ISP_Feature_H */

/*******************************************************************************
*
********************************************************************************/
//
//camera_custom_cs.h
//

//#ifndef _CUSTOM_CAMERA_CS_H_
//#define _CUSTOM_CAMERA_CS_H_
//#include <cs.h>
//#include <csmedia.h>
//#include "msdk_isp_feature_exp.h"
//#include "msdk_post_process_feature_exp.h"
//#include "video_types_exp.h"
//#include "jpeg_enum.h"
//
//#define CameraAFEventName ((LPCWSTR) (L"AFEvent"))
//#define CameraJPEGDoneEventName     ((LPCWSTR) (L"JPEGDoneEvent"))
//#define CameraFDEventName           ((LPCWSTR) (L"FDEvent"))
//#define CameraPanoramaEventName     ((LPCWSTR) (L"PanoEvent"))
//
//// {D38CDCC5-BD03-4fa4-9014-95ED5DA4A362}
//#define STATIC_PROPSETID_CUSTOM_CAMERA    0xd38cdcc5, 0xbd03, 0x4fa4, 0x90, 0x14, 0x95, 0xed, 0x5d, 0xa4, 0xa3, 0x62
//DEFINE_GUIDSTRUCT("D38CDCC5-BD03-4fa4-9014-95ED5DA4A362", PROPSETID_CUSTOM_CAMERA);
//#define PROPSETID_CUSTOM_CAMERA DEFINE_GUIDNAMED(PROPSETID_CUSTOM_CAMERA)
//
//// Defined for adding to time stamp of video encode bit stream to notify video encode DMO and
//// MP4 packer that the video bit tram is the last frame when pause/stop or the first frame when resume/start
//#define VIDOE_I_FRAME                   0x0001
//#define VIDEO_PAUSE_FRAME               0x0002
//#define VIDEO_RESUME_FRAME              0x0004
//
///* the correlated feature's enum of camera is defined in msdk_isp_feature_exp.h */
//typedef enum propertyid_customcamera
//{
//    CUSTOMCAMERA_EFFECT=0,              // RW O, CAM_EFFECT_ENUM
//    CUSTOMCAMERA_BANDING,               // RW O, CAM_BANDING_FREQ_ENUM
//    CUSTOMCAMERA_SCENE_MODE,            // RW O, CAM_SCENE_MODE_ENUM
//    CUSTOMCAMERA_ISO,                   // RW O, Not support now
//    CUSTOMCAMERA_AE_METERING_MOE,       // RW O, CAM_AE_METERING_MODE_ENUM
//    CUSTOMCAMERA_AF_KEY,                // RW O, CMA_AF_KEY_ENUM
//    CUSTOMCAMERA_AF_RANGE,              // RW O, CAM_AF_RANGE_ENUM
//    CUSTOMCAMERA_AF_MODE,               // RW O, CAM_AF_OPERATION_MODE_ENUM
//    CUSTOMCAMERA_BINNING,               // RW O, Not support now
//    CUSTOMCAMERA_VIDEO_SCENE_MODE,      // RW O, video scene mode
//
//    /* The following feature do not map to ISP features and will be processed by camera driver*/
//    CUSTOMCAMERA_GROUP_FEATURES,        // RW O, CAMERA_GROUP_FEATURES_STRUCT
//    CUSTOMCAMERA_AF_RESULT,             // RW O, CAMERA_AF_RESULT_STRUCT
//    CUSTOMCAMERA_IMAGE_ROTATE_MIRROR,   // RW O, CAM_IMAGE_ROTATE_MIRROR_ENUM
//    CUSTOMCAMERA_JPEG_QUALITY,          // RW O, JPEG_QUALITY_ENUM
//    CUSTOMCAMERA_VIDEO_QUALITY,         // RW O, VIDEO_QUALITY_ENUM
//    CUSTOMCAMERA_BURST_CAPTURE_NUMBER,  // RW O, picture number of burst capture
//    CUSTOMCAMERA_STILL_CAPTURE_MODE,    // RW O, select of camera capture mode
//    CUSTOMCAMERA_VIDEO_TIME_STAMP,      // R O, time stamp of current video encode bit stream.
//    CUSTOMCAMERA_ISO_BINNING_INFO,      // R O, the information of High ISO binning.
//    CUSTOMCAMERA_FD_RESULT,             // R O, the result of face detection
//    CUSTOMCAMERA_FD_CONTROL,            // RW O, start/stop face detection
//    CUSTOMCAMERA_PANORAMA_INFO,         // R O, the information of panorama capture.
//    CUSTOMCAMERA_PANORAMA_CONTROL,      // W O, start stitch/stop panorama capture process.
//    CUSTOMCAMERA_PANORAMA_RESULT,       // R O, the result of panorama capture.
//    CUSTOMCAMERA_PANORAMA_DIRECTION,    // W O, the direction of panorama capture.
//    CUSTOMCAMERA_SET_ADD_FRAME_PALETTE, // W O, enable/disalbe and configure camera overlay frame feature
//    CUSTOMCAMERA_ADD_FRAME,             // W O, enable/disalbe and configure camera overlay frame feature
//    CUSTOMCAMERA_DUAL_CAMERA_SENSOR_SEL,// RW O, get/set currenct selected sensor (MAIN/SUB)
//    CUSTOMCAMERA_ABILITY_MAX
//} CSPROPERTY_CUSTOM_CAMERA;
//
//typedef enum
//{
//    ROT_ANGLE_0,
//    ROT_ANGLE_90,
//    ROT_ANGLE_180,
//    ROT_ANGLE_270,
//    ROT_ANGLE_0_FLIP,
//    ROT_ANGLE_90_FLIP,
//    ROT_ANGLE_180_FLIP,
//    ROT_ANGLE_270_FLIP,
//    ROT_ANGLE_MAX
//} IMAGE_ROTATE_FLIP_ENUM;
//
//typedef enum
//{
//    JPEG_QUALITY_EXCELLENT=JPEG_ENCODE_QUALITY_EXCELLENT,
//    JPEG_QUALITY_GOOD=JPEG_ENCODE_QUALITY_GOOD,
//    JPEG_QUALITY_FAIR=JPEG_ENCODE_QUALITY_FAIR,
//    JPEG_QUALITY_LOW=JPEG_ENCODE_QUALITY_LOW,
//    JPEG_QUALITY_POOR=JPEG_ENCODE_QUALITY_POOR
//} JPEG_QUALITY_ENUM;
//
//#define VIDEO_QUALITY_ENUM VIDEO_QUALITY
//
//typedef struct
//{
//    MINT32    BinningMode;
//    MUINT8   ContrastLevel;
//    MUINT8   SaturationLevel;
//    MUINT8   SharpnessLevel;
//    MUINT8   BrightnessLevel;
//    CAM_WB_MODE_ENUM            WBMode;
//    CAM_BANDING_FREQ_ENUM       BandingFreq;
//    CAM_EFFECT_ENUM             ImageEffect;
//    CAM_AF_OPERATION_MODE_ENUM  AFOperationMode;
//    CAM_AF_RANGE_ENUM           AFRange;
//    CAM_SCENE_MODE_ENUM         DscMode;
//    CAM_FLASH_MODE_ENUM         FlashMode;
//    CAM_AE_METERING_MODE_ENUM   AEMeteringMode;
//    MUINT16  ZoomStep;
//
//    CAM_IMAGE_ROTATE_MIRROR_ENUM    ImageDisplayRotateMirror;
//    VIDEO_QUALITY_ENUM              VideoQuality;
//    JPEG_QUALITY_ENUM               JPEGQuality;
//    MSDK_FD_CONTROL_ENUM            FDControl;
//    PANO_DIRECTION_ENUM             PanoDirection;
//} CAMERA_GROUP_FEATURES_STRUCT, *PCAMERA_GROUP_FEATURES_STRUCT;
//
//typedef struct
//{
//    HANDLE  hEvent;
//    MSDK_ISP_AF_STATUS_ENUM AFZoneStatus[5];
//    RECT rc[5];
//} CAMERA_AF_RESULT_STRUCT, *PCAMERA_AF_RESULT_STRUCT;
//
//typedef enum
//{
//    MSDK_STILL_SINGLE_CAPTURE=0,
//    MSDK_STILL_BURST_CAPTURE,
//    MSDK_STILL_BSS_CAPTURE,
//    MSDK_STILL_EV_BRACKET_CAPTURE,
//    MSDK_STILL_PANORAMA_CAPTURE,
//    MSDK_STILL_ADD_FRAME_CAPTURE,
//    MSDK_STILL_CAPTURE_MODE_MAX
//} CAMERA_STILL_CAPTURE_MODE_ENUM;
//
//typedef struct
//{
//    CAMERA_STILL_CAPTURE_MODE_ENUM  CameraStillCaptureMode;
//    MSDK_ISP_AE_BRACKET_ENUM EVBracketLevel;
//    MUINT8 BurstShotNumber;
//} CAMERA_STILL_CAPTURE_MODE_STRUCT, *PCAMERA_STILL_CAPTURE_MODE_STRUCT;

typedef struct
{
    ACDK_ISP_BINNING_INFO_STRUCT    ISOBinningInfo[ISO_MAX_MODE];
} CAMERA_ISO_BINNING_INFO_STRUCT, *PCAMERA_ISO_BINNING_INFO_STRUCT;

///* FD Result Parameters */
//#define CAMERA_FD_MAX_NO        (9)
//typedef struct
//{
//    MUINT8   FaceNo;                 /* total detected face no */
//    MUINT8   Priority[CAMERA_FD_MAX_NO]; /* face priority */
//    RECT    rc[CAMERA_FD_MAX_NO];       /* face position */
//    MUINT32  FDTargetWidth;              /* FD position target width */
//    MUINT32  FDTargetHeight;             /* FD position target height */
//    MINT32    DetectRateLog;          /* Detection rate log enable flah */
//} CAMERA_FD_RESULT_STRUCT, *PCAMERA_FD_RESULT_STRUCT, camera_fd_result_struct;
//
//typedef struct
//{
//    // for Camera APP
//    MUINT16 MaxWidth;        // maximum capture widht in panoram capture mode
//    MUINT16 MaxHeight;       // maximum capture height in panoram capture mode
//    MUINT16 MaxPanoNo;       // maximum number of stitched pictures.
//} CAMERA_PANORAMA_INFO_STRUCT, *PCAMERA_PANORAMA_INFO_STRUCT;
//typedef struct
//{
//    MUINT32 BuffVirAddr;
//    MUINT32 BuffSize;
//} CAMERA_PANORAMA_RESULT_STRUCT, *PCAMERA_PANORAMA_RESULT_STRUCT;
//
//typedef enum
//{
//    OVERLAY_COLOR_DEPTH_1BPP=0,
//    OVERLAY_COLOR_DEPTH_2BPP,
//    OVERLAY_COLOR_DEPTH_4BPP,
//    OVERLAY_COLOR_DEPTH_8BPP
//} OVERLAY_FRAME_COLOR_DEPTH_ENUM;
//
//typedef struct
//{
//    MUINT16 NumOverlayFrameColorPalette;     // should be 256 for 8bit mode.
//    MUINT32 *pOverlayFrameColorPalette;
//} CAMERA_SET_ADD_FRAME_PALETTE_STRUCT, *PCAMERA_SET_ADD_FRAME_PALETTE_STRUCT;
//
//typedef struct
//{
//    MINT32    EnableOverlayFrame;         // Enable/Disable Overlay frame function.
//    OVERLAY_FRAME_COLOR_DEPTH_ENUM  OverlayFrameColorDepth;
//    MUINT8   OverlayFrameSrcKey;         // source key of the overlayed frame for transparency
//    MUINT16  OverlayFrameWidth;          // width of the overlayed frame
//    MUINT16  OverlayFrameHeight;         // height of the overlayed frame
//    MUINT32  OverlayFrameBuffAddr;       // buffer address of the overlayed frame
//} CAMERA_ADD_FRAME_STRUCT, *PCAMERA_ADD_FRAME_STRUCT;

//
//typedef struct
//{
//    CSPROPERTY Property;
//    LONG       Value;
//    ULONG      Flags;
//    ULONG      Capabilities;
//    union
//    {
//        CAMERA_GROUP_FEATURES_STRUCT    CamGroupFeature;
//        CAMERA_AF_RESULT_STRUCT     CamAFData;
//        CAMERA_STILL_CAPTURE_MODE_STRUCT CamStillCaptureMode;
//        CAMERA_ISO_BINNING_INFO_STRUCT   CamISOBinningInfo;
//        CAMERA_FD_RESULT_STRUCT             CamFDResult;        // The structure is defined in msdk_post_process_exp.h
//        CAMERA_PANORAMA_INFO_STRUCT         CamPanoInfo;
//        CAMERA_PANORAMA_RESULT_STRUCT       CamPanoResult;
//        CAMERA_SET_ADD_FRAME_PALETTE_STRUCT CamSetAddFramePalette;
//        CAMERA_ADD_FRAME_STRUCT             CamAddFrame;
//    };
//} CAMERA_CUSTOM_FEATURES_STRUCT, *PCAMERA_CUSTOM_FEATURES_STRUCT;
//#endif //_CUSTOM_CAMERA_CS_H_

/*******************************************************************************
*
********************************************************************************/
//
//msdk_common_define_exp.h
//

//#ifndef __MSDK_COMM_DEFINE_EXP_H__
//#define __MSDK_COMM_DEFINE_EXP_H__
//
//// MSDK based Media Player need to listen the event to stop media player for
//// 1. H/W concurrency problem
//// 2. H/W module power down -> power on state cleared issue
//#define STOP_MSDK_MEDIA_PLAYER_EVENT    L"StopMSDKMediaPlayerEvent"
//
//#define MSDK_DEVICE_NAME                L"MDK1:"
//
//#define CAMERA_MSG_THREAD_EXIT_CODE     0xAA55A55A


typedef enum
{
    ACDK_SCENARIO_ID_CAMERA_PREVIEW=0,
    ACDK_SCENARIO_ID_VIDEO_PREVIEW,
    ACDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4,
    ACDK_SCENARIO_ID_CAMERA_3D_VIDEO,
    ACDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG,
    ACDK_SCENARIO_ID_CAMERA_CAPTURE_MEM,
    ACDK_SCENARIO_ID_CAMERA_BURST_CAPTURE_JPEG,
    ACDK_SCENARIO_ID_VIDEO_DECODE_MPEG4,
    ACDK_SCENARIO_ID_VIDEO_DECODE_H263,
    ACDK_SCENARIO_ID_VIDEO_DECODE_H264,
    ACDK_SCENARIO_ID_VIDEO_DECODE_WMV78,
    ACDK_SCENARIO_ID_VIDEO_DECODE_WMV9,
    ACDK_SCENARIO_ID_VIDEO_DECODE_MPEG2,
    ACDK_SCENARIO_ID_IMAGE_YUV2RGB,
    ACDK_SCENARIO_ID_IMAGE_RESIZE,
    ACDK_SCENARIO_ID_IMAGE_ROTATE,
    ACDK_SCENARIO_ID_IMAGE_POST_PROCESS,
    ACDK_SCENARIO_ID_JPEG_RESIZE,
    ACDK_SCENARIO_ID_JPEG_DECODE,
    ACDK_SCENARIO_ID_JPEG_PARSE,
    ACDK_SCENARIO_ID_JPEG_ENCODE,
    ACDK_SCENARIO_ID_JPEG_ENCODE_THUMBNAIL,
    ACDK_SCENARIO_ID_DRIVER_IO_CONTROL,
    ACDK_SCENARIO_ID_DO_NOT_CARE,
    ACDK_SCENARIO_ID_IMAGE_DSPL_BUFFER_ALLOC,
    ACDK_SCENARIO_ID_TV_OUT,
    ACDK_SCENARIO_ID_MAX,
    ACDK_SCENARIO_ID_VIDOE_ENCODE_WITHOUT_PREVIEW,      // for LTK test case
    ACDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG_BACK_PREVIEW,  // for LTK test case
    ACDK_SCENARIO_ID_VIDEO_DECODE_RV8,
    ACDK_SCENARIO_ID_VIDEO_DECODE_RV9,
    ACDK_SCENARIO_ID_CAMERA_ZSD,
    ACDK_SCENARIO_ID_CAMERA_3D_PREVIEW,
    ACDK_SCENARIO_ID_CAMERA_3D_CAPTURE,
}   ACDK_SCENARIO_ID_ENUM;

//typedef enum
//{
//    MSDK_BUFFER_TYPE_PREVIEW=0,
//    MSKD_BUFFER_TYPE_JPEG_ENC_STREAM,
//    MSDK_BUFFER_TYPE_JPEG_DEC_STREAM,
//    MSDK_BUFFER_TYPE_VIDEO_ENC_STREAM,
//    MSDK_BUFFER_TYPE_VIDEO_DEC_STREAM,
//    MSDK_BUFFER_TYPE_MAX
//}   MSDK_BUFFER_TYPE_ENUM;
//
//typedef enum
//{
//    MSDK_MSG_QUEUE_CAMERA=0,
//    MSDK_MSG_QUEUE_VIDEO_PLAY,
//    MSDK_MSG_QUEUE_IMAGE_PLAY,
//    MSDK_MSG_QUEUE_IMAGE_PROCESS,
//    MSDK_MSG_QUEUE_MAX
//}   MSDK_MSG_QUEUE_ENUM;
//
//typedef enum
//{   /* enum to identify the hardware that the AP would like to use */
//    /* When AP would like to enter or exit one operation group, it has to send one of the value as
//       parameters to MSDK driver such that the hardware driver can control its operation state */
//    MSDK_ENTER_EXIT_PARA_CAMERA=0,
//    MSDK_ENTER_EXIT_PARA_VIDEO,
//    MSDK_ENTER_EXIT_PARA_IMAGE,
//    MSDK_ENTER_EXIT_PARA_JPEG,
//    MSDK_ENTER_EXIT_PARA_MAX
//} MSDK_ENTER_EXIT_CONTORL_PARA_ENUM;
//
//#define MSDK_CONTROL_CODE_INIT              0x00000001
//#define MSDK_CONTROL_CODE_DEINIT            0x00000002
//#define MSDK_CONTROL_CODE_CONFIG            0x00000004
//#define MSDK_CONTROL_CODE_START             0x00000008
//#define MSDK_CONTROL_CODE_STOP              0x00000010
//#define MSDK_CONTROL_CODE_PAUSE             0x00000020
//#define MSDK_CONTROL_CODE_RESUME            0x00000040
//#define MSDK_CONTROL_CODE_MEM_INIT          0x00000080
//#define MSDK_CONTROL_CODE_ENTER             0x00000100
//#define MSDK_CONTROL_CODE_EXIT              0x00000200
//#define MSDK_CONTROL_CODE_STATISTICS        0x00000400
//// used by MSDK driver to configure camera 3A parameters
//#define MSDK_CONTROL_CODE_CONFIG_CAMERA3A   0x00000800
//// used by MSDK driver to trigger pano driver
//#define MSDK_CONTROL_CODE_BGSTITCH              0x00001000
//#define MSDK_CONTROL_CODE_STITCH                0x00002000
//#define MSDK_CONTROL_CODE_FEATURE           0x80000000
//
//// This value should be located at 32768(0x8000) ~ 65535(0xFFFF)
//#define FILE_DEVICE_MSDK            0x00008800
//
//#define IOCTL_MSDK_CAMERA_CONTROL                   CTL_CODE(FILE_DEVICE_MSDK, 0x0801, METHOD_BUFFERED, FILE_ANY_ACCESS)
//#define IOCTL_MSDK_VIDEO_CONTROL                    CTL_CODE(FILE_DEVICE_MSDK, 0x0802, METHOD_BUFFERED, FILE_ANY_ACCESS)
//#define IOCTL_MSDK_IMAGE_CONTROL                    CTL_CODE(FILE_DEVICE_MSDK, 0x0803, METHOD_BUFFERED, FILE_ANY_ACCESS)
//#define IOCTL_MSDK_JPEG_CONTROL                     CTL_CODE(FILE_DEVICE_MSDK, 0x0804, METHOD_BUFFERED, FILE_ANY_ACCESS)
//
//#define IOCTL_MSDK_BUFFER_CONTROL                   CTL_CODE(FILE_DEVICE_MSDK, 0x0901, METHOD_BUFFERED, FILE_ANY_ACCESS)
//
//#define IOCTL_MSDK_SET_RETURN_MSG_QUEUE_HANDLER     CTL_CODE(FILE_DEVICE_MSDK, 0x0A01, METHOD_BUFFERED, FILE_ANY_ACCESS)
//#define IOCTL_MSDK_UNSET_RETURN_MSG_QUEUE_HANDLER       CTL_CODE(FILE_DEVICE_MSDK, 0x0A02, METHOD_BUFFERED, FILE_ANY_ACCESS)
//
//#define MSDK_CAMERA_FEATURE_START           0       // should be fixed to mapping the feature with MSFT's standard
//#define MSDK_ISP_EXTEND_FEATURE_START       1000
//#define MSDK_POST_PROCESS_FEATURE_START     5000
//#define MSDK_ISP_CCT_FEATURE_START          10000
//// Tmp workaround of CR_111078
//#define IOCTL_MSDK_QUERY_MEM_RESET_COUNT        CTL_CODE(FILE_DEVICE_MSDK, 0x0B01, METHOD_BUFFERED, FILE_ANY_ACCESS)
//
//typedef enum
//{
//    MSDK_BUFF_INIT=0,
//    MSDK_BUFF_DEINIT,
//    MSDK_BUFF_ALLOCATE,
//    MSDK_BUFF_ASSIGN,
//    MSDK_BUFF_ENQUEUE,
//    MSDK_BUFF_QUERY_PHY_VIR_MAPPING,
//    MSDK_BUFF_CONTROL_MAX
//} MSDK_BUFF_CONTROL_ENUM;
//
//#define MSDK_MAGIC_ADDRESS       0xF7654321
//#define MSDK_INVALID_ADDRESS            0xFFFFFFFF
//#define MSDK_MAX_MSG_SIZE               512
//#define MSDK_MIN_VIDEO_BITSTREAM_SIZE   10240   // 10KB
//#define MSDK_MAGIC_BUFFER_SIZE   0xFFFFFFFF
//
///// Magic word to specify the MSDK_IMAGE_BUFFER_CUSTOMIZATION_STRUCT.
//#define MSDK_IMAGE_BUFFER_CUSTOMIZATION_ID   0x1F3D5B79
//
///* callback ID ENUM definition for multimedia driver's callback */
//typedef enum
//{
//    MSDK_WM_CBID_PREVIEW_BUFFER_READY=0,
//    MSDK_WM_CBID_CAPTURE_JPEG_READY,
//    MSDK_WM_CBID_CAPTURE_MEM_REAADY,
//    MSDK_WM_CBID_VIDEO_ENC_YUV_BUFFER_READY,
//    MSDK_WM_CBID_VIDOE_ENC_STREAM_BUFFER_READY,
//    MSDK_WM_CBID_JPEG_ENCODE_MAIN_IMAGE_READY,
//    MSDK_WM_CBID_JPEG_DECODE_READY,
//    MSDK_WM_CBID_JPEG_ENCODE_READY,
//    MSDK_WM_CBID_ISP_CAM_FOCUS,
//    MSDK_WM_CBID_IDP_DIGITAL_ZOOM,
//    MSDK_WM_CBID_VIDEO_DEC_STREAM_BUFFER_READY,
//    MSDK_WM_CBID_IMAGE_Y2R_BUFFER_READY,
//    MSDK_WM_CBID_IMAGE_RESIZE_BUFFER_READY,
//    MSDK_WM_CBID_VIDEO_RECORD_DROP_FRAME_NUMBER,
//    MSDK_WM_CBID_VIDEO_DECODE_RESULT,
//    MSDK_WM_CBID_VIDEO_DECODE_RELEASE_REFERENCE,
//    MSDK_WM_CBID_VIDEO_DECODE_RELEASE_BITSTREAM,
//    MSDK_WM_CBID_VIDEO_FLUSH_END,
//    MSDK_WM_CBID_FD_BUFFER_READY,
//    MSDK_WM_CBID_FD_RESULT,
//    MSDK_WM_CBID_PANORAMA_RESULT,
//    MSDK_WM_CBID_IMAGE_ROTATE_BUFFER_READY,
//        MSDK_WM_CBID_PANORAMA_MEMORY_COPY,
//        MSDK_WM_CBID_PANORAMA_PARTIAL_DECODE,
//        MSDK_WM_CBID_PANORAMA_STITCH_DONE,
//    MSDK_WM_CBID_TV_OUT_BUFFER_READY,
//    MSDK_CBID_MAX
//} MSDK_WM_CALLBACK_ID_ENUM;

typedef enum
{
    MSDK_CAMERA_OPERATION_NORMAL_MODE=0,
    MSDK_CAMERA_OPERATION_META_MODE
} ACDK_CAMERA_OPERATION_MODE_ENUM;
//// Video decode related structures
//typedef enum
//{
//    MSDK_WMV_CODEC_VERSION_WMV7 = 0x00000004L,
//    MSDK_WMV_CODEC_VERSION_WMV8 = 0x00000005L,
//    MSDK_WMV_CODEC_VERSION_WMV9 = 0x00000006L,
//    MSDK_WMV_CODEC_VERSION_MAX
//} MSDK_WMV_CODEC_VERSION_ENUM;
//
//#define MSDK_MAX_VDEC_RELEASE_BITSTREAM_MSG_NUM 5
//
//#define NUM_OF_IDP_OVERLAY_COLOR_PALETTE    256
//
//typedef enum
//{
//  IMAG_COLOR_FMT_RGB565,
//  IMAG_COLOR_FMT_BGR565,
//  IMAG_COLOR_FMT_RGB888,
//  IMAG_COLOR_FMT_BGR888,
//  IMAG_COLOR_FMT_ARGB8888,
//  IMAG_COLOR_FMT_ABGR8888,
//  IMAG_COLOR_FMT_UNKNOWN= 0xFF,
//}imag_color_fmt_rgb_enum;
//
//typedef enum
//{
//  IMAG_ROT_ANGLE_0,
//  IMAG_ROT_ANGLE_90,
//  IMAG_ROT_ANGLE_180,
//  IMAG_ROT_ANGLE_270
//}imag_rot_angle_enum;
//
///* control of frame buffer */
//#define MSDK_MAX_FRAME_BUFF_COUNT       10
//
//typedef enum
//{
//    MSDK_RESET_FRAME_BUFFER=0,
//    MSDK_ASSIGN_FRAME_BUFFER,
//    MSDK_ACTIVATE_FRAME_BUFFER,
//    MSDK_FRAME_BUFFER_CONTROL_MAX
//} MSDK_FRAME_BUFFER_CONTROL_ID_ENUM;
//
//typedef enum
//{
//    MSDK_BUFF_STATUS_READY=0,
//    MSDK_BUFF_STATUS_BUSY,
//    MSDK_BUFF_STATUS_FULL,
//    MSDK_BUFF_STATUS_IDLE
//} MSDK_BUFF_STATUS_ENUM;
//
//#define MSDK_CAMERA_NO_LTK_ROTATE   0xFF
//
//// define for DWORD register key read/write
//#define REG_DWORD_READ  0
//#define REG_DWORD_WRITE 1
//
//#endif __MSDK_COMM_DEFINE_EXP_H__

/*******************************************************************************
*
********************************************************************************/
//acdkcctfeature.h
//
//msdk_sensor_exp.h
//

//#ifndef __MSDK_Sensor_if_H
//#define __MSDK_Sensor_if_H
///* This function defines data structure that can be used by AP, Sensor, MSDK and
//   ISP */
//#include "msdk_feature_exp.h"
//#include "msdk_nvram_camera_exp.h"
//
#define MAX_NUM_OF_SUPPORT_SENSOR 16
//
#define SENSOR_CLOCK_POLARITY_HIGH    0
#define SENSOR_CLOCK_POLARITY_LOW 1
//
#define LENS_DRIVER_ID_DO_NOT_CARE    0xFFFFFFFF
#define SENSOR_DOES_NOT_EXIST     0x00FFFFFF
#define SENSOR_DOES_NOT_KNOW      0xFFFFFFFF

#define SENSOR_FEATURE_START                     3000

/*******************************************************************************
*
********************************************************************************/

/*  */
#define MAX_NUM_OF_SUPPORT_SENSOR 16
/*  */
#define SENSOR_CLOCK_POLARITY_HIGH    0
#define SENSOR_CLOCK_POLARITY_LOW 1
/*  */
#define LENS_DRIVER_ID_DO_NOT_CARE    0xFFFFFFFF
#define SENSOR_DOES_NOT_EXIST     0x00FFFFFF
#define SENSOR_DOES_NOT_KNOW      0xFFFFFFFF

#define SENSOR_FEATURE_START                     3000
typedef enum {
	SENSOR_FEATURE_BEGIN = SENSOR_FEATURE_START,
	SENSOR_FEATURE_GET_RESOLUTION,
	SENSOR_FEATURE_GET_PERIOD,
	SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ,
	SENSOR_FEATURE_SET_ESHUTTER,
	SENSOR_FEATURE_SET_NIGHTMODE,
	SENSOR_FEATURE_SET_GAIN,
	SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER,
	SENSOR_FEATURE_SET_FLASHLIGHT,
	SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ,
	SENSOR_FEATURE_SET_REGISTER,
	SENSOR_FEATURE_GET_REGISTER,
	SENSOR_FEATURE_SET_CCT_REGISTER,
	SENSOR_FEATURE_GET_CCT_REGISTER,
	SENSOR_FEATURE_SET_ENG_REGISTER,
	SENSOR_FEATURE_GET_ENG_REGISTER,
	SENSOR_FEATURE_GET_REGISTER_DEFAULT,
	SENSOR_FEATURE_GET_CONFIG_PARA,
	SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR,
	SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA,
	SENSOR_FEATURE_GET_GROUP_COUNT,
	SENSOR_FEATURE_GET_GROUP_INFO,
	SENSOR_FEATURE_GET_ITEM_INFO,
	SENSOR_FEATURE_SET_ITEM_INFO,
	SENSOR_FEATURE_GET_ENG_INFO,
	SENSOR_FEATURE_GET_LENS_DRIVER_ID,
	SENSOR_FEATURE_SET_YUV_CMD,
	SENSOR_FEATURE_SET_VIDEO_MODE,
	SENSOR_FEATURE_SET_TARGET_FRAME_RATE,
	SENSOR_FEATURE_SET_CALIBRATION_DATA,
	SENSOR_FEATURE_SET_SENSOR_SYNC,
	SENSOR_FEATURE_INITIALIZE_AF,
	SENSOR_FEATURE_CONSTANT_AF,
	SENSOR_FEATURE_INFINITY_AF,
	SENSOR_FEATURE_MOVE_FOCUS_LENS,
	SENSOR_FEATURE_GET_AF_STATUS,
	SENSOR_FEATURE_GET_AE_STATUS,
	SENSOR_FEATURE_GET_AWB_STATUS,
	SENSOR_FEATURE_GET_AF_INF,
	SENSOR_FEATURE_GET_AF_MACRO,
	SENSOR_FEATURE_CHECK_SENSOR_ID,
	SENSOR_FEATURE_SET_AUTO_FLICKER_MODE,
	SENSOR_FEATURE_SET_TEST_PATTERN,
	SENSOR_FEATURE_SET_SOFTWARE_PWDN,
	SENSOR_FEATURE_SINGLE_FOCUS_MODE,
	SENSOR_FEATURE_CANCEL_AF,
	SENSOR_FEATURE_SET_AF_WINDOW,
	SENSOR_FEATURE_GET_EV_AWB_REF,
	SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN,
	SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS,
	SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS,
	SENSOR_FEATURE_SET_AE_WINDOW,
	SENSOR_FEATURE_GET_EXIF_INFO,
	SENSOR_FEATURE_GET_DELAY_INFO,
	SENSOR_FEATURE_CHECK_ALIVE,
	SENSOR_FEATURE_SET_SLAVE_I2C_ID,
	SENSOR_FEATURE_SUSPEND,
	SENSOR_FEATURE_RESUME,
	SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,
	SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO,
	SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO,
	SENSOR_FEATURE_AUTOTEST_CMD,
	SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE,
	SENSOR_FEATURE_GET_TEMPERATURE_VALUE,
	SENSOR_FEATURE_GET_SENSOR_CURRENT_TEMPERATURE,
	SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO,
	SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO, /* Trigger flashlight or not */
	SENSOR_FEATURE_SET_YUV_3A_CMD,
	SENSOR_FEATURE_SET_N3D_I2C_STREAM_REGDATA,
	SENSOR_FEATURE_SET_N3D_STOP_STREAMING,
	SENSOR_FEATURE_SET_N3D_START_STREAMING,
	SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME,
	SENSOR_FEATURE_SET_ESHUTTER_GAIN,
	SENSOR_FEATURE_SET_OB_LOCK,
	SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD,
	SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD,
	SENSOR_FEATURE_GET_YUV_CAPTURE_OUTPUT_JPEG,
	SENSOR_FEATURE_SET_YUV_JPEG_PARA,
	SENSOR_FEATURE_GET_YUV_JPEG_INFO,
	SENSOR_FEATURE_SET_FRAMERATE,
	SENSOR_FEATURE_SET_HDR,
	SENSOR_FEATURE_GET_CROP_INFO,
	SENSOR_FEATURE_GET_VC_INFO,
	SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN,
	SENSOR_FEATURE_SET_AWB_GAIN,
	SENSOR_FEATURE_SET_MIN_MAX_FPS,
	SENSOR_FEATURE_GET_PDAF_INFO,
	SENSOR_FEATURE_GET_PDAF_DATA,
	SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY,
	SENSOR_FEATURE_DEBUG_IMGSENSOR,
	SENSOR_FEATURE_SET_HDR_SHUTTER,
	SENSOR_FEATURE_MAX
} ACDK_SENSOR_FEATURE_ENUM;

typedef enum {
	SENSOR_AF_IDLE = 0,
	SENSOR_AF_FOCUSING,
	SENSOR_AF_FOCUSED,
	SENSOR_AF_ERROR,
	SENSOR_AF_SCENE_DETECTING,
	SENSOR_AF_STATUS_MAX
} ACDK_SENSOR_AF_STATUS_ENUM;

typedef enum {
	SENSOR_AE_IDLE = 0,
	SENSOR_AE_EXPOSING,
	SENSOR_AE_EXPOSED,
	SENSOR_AE_ERROR,
	SENSOR_AE_LOCKED
} ACDK_SENSOR_AE_STATUS_ENUM;

typedef enum {
	SENSOR_AWB_IDLE = 0,
	SENSOR_AWB_BALANCING,
	SENSOR_AWB_BALANCED,
	SENSOR_AWB_ERROR,
	SENSOR_AWB_LOCKED
} ACDK_SENSOR_AWB_STATUS_ENUM;



typedef enum {
	SENSOR_INTERFACE_TYPE_PARALLEL = 0,
	SENSOR_INTERFACE_TYPE_MIPI,
	SENSOR_INTERFACE_TYPE_SERIAL,   /* add for 72 ATV using serial interface */
	SENSOR_INTERFACE_TYPE_MAX
} ACDK_SENSOR_INTERFACE_TYPE_ENUM;

typedef enum {
	SENSOR_OUTPUT_FORMAT_RAW_B = 0,
	SENSOR_OUTPUT_FORMAT_RAW_Gb,
	SENSOR_OUTPUT_FORMAT_RAW_Gr,
	SENSOR_OUTPUT_FORMAT_RAW_R,
	SENSOR_OUTPUT_FORMAT_UYVY,
	SENSOR_OUTPUT_FORMAT_VYUY,
	SENSOR_OUTPUT_FORMAT_YUYV,
	SENSOR_OUTPUT_FORMAT_YVYU,
	SENSOR_OUTPUT_FORMAT_CbYCrY,
	SENSOR_OUTPUT_FORMAT_CrYCbY,
	SENSOR_OUTPUT_FORMAT_YCbYCr,
	SENSOR_OUTPUT_FORMAT_YCrYCb,
	SENSOR_OUTPUT_FORMAT_RAW8_B,
	SENSOR_OUTPUT_FORMAT_RAW8_Gb,
	SENSOR_OUTPUT_FORMAT_RAW8_Gr,
	SENSOR_OUTPUT_FORMAT_RAW8_R,
} ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM;

typedef enum {
	SENSOR_MIPI_1_LANE = 0,
	SENSOR_MIPI_2_LANE,
	SENSOR_MIPI_3_LANE,
	SENSOR_MIPI_4_LANE
} ACDK_SENSOR_MIPI_LANE_NUMBER_ENUM;

typedef enum {
	SCAM_1_DATA_CHANNEL = 0,
	SCAM_2_DATA_CHANNEL,
	SCAM_3_DATA_CHANNEL,
	SCAM_4_DATA_CHANNEL,
} SENSOR_SCAM_DATA_CHANNEL_ENUM;

typedef enum {
	MIPI_OPHY_NCSI2 = 0,
	MIPI_OPHY_CSI2 = 1,
} SENSOR_MIPI_TYPE_ENUM;

typedef enum {
	MIPI_SETTLEDELAY_AUTO = 0,
	MIPI_SETTLEDELAY_MANUAL = 1,
} SENSOR_SETTLEDELAY_MODE_ENUM;

typedef enum {
	COMP8_NONE  = 0,
	COMP8_DI_30 = 0x30,
	COMP8_DI_31 = 0x31,
	COMP8_DI_32 = 0x32,
	COMP8_DI_33 = 0x33,
	COMP8_DI_34 = 0x34,
	COMP8_DI_35 = 0x35,
	COMP8_DI_36 = 0x36,
	COMP8_DI_37 = 0x37,
	COMP8_DI_2A = 0x2A,
} SENSOR_DPCM_TYPE_ENUM;

typedef struct {
	MUINT16 SensorPreviewWidth;
	MUINT16 SensorPreviewHeight;
	MUINT16 SensorFullWidth;
	MUINT16 SensorFullHeight;
	MUINT16 SensorVideoWidth;
	MUINT16 SensorVideoHeight;
	MUINT16 SensorHighSpeedVideoWidth;
	MUINT16 SensorHighSpeedVideoHeight;
	MUINT16 SensorSlimVideoWidth;
	MUINT16 SensorSlimVideoHeight;
	MUINT16 SensorCustom1Width;
	MUINT16 SensorCustom1Height;
	MUINT16 SensorCustom2Width;
	MUINT16 SensorCustom2Height;
	MUINT16 SensorCustom3Width;
	MUINT16 SensorCustom3Height;
	MUINT16 SensorCustom4Width;
	MUINT16 SensorCustom4Height;
	MUINT16 SensorCustom5Width;
	MUINT16 SensorCustom5Height;
	MUINT16 Sensor3DPreviewWidth;
	MUINT16 Sensor3DPreviewHeight;
	MUINT16 Sensor3DFullWidth;
	MUINT16 Sensor3DFullHeight;
	MUINT16 Sensor3DVideoWidth;
	MUINT16 Sensor3DVideoHeight;
	MUINT16 SensorEffectivePreviewWidth;
	MUINT16 SensorEffectivePreviewHeight;
	MUINT16 SensorEffectiveFullWidth;
	MUINT16 SensorEffectiveFullHeight;
	MUINT16 SensorEffectiveVideoWidth;
	MUINT16 SensorEffectiveVideoHeight;
	MUINT16 SensorEffectiveHighSpeedVideoWidth;
	MUINT16 SensorEffectiveHighSpeedVideoHeight;
	MUINT16 SensorEffective3DPreviewWidth;
	MUINT16 SensorEffective3DPreviewHeight;
	MUINT16 SensorEffective3DFullWidth;
	MUINT16 SensorEffective3DFullHeight;
	MUINT16 SensorEffective3DVideoWidth;
	MUINT16 SensorEffective3DVideoHeight;
	MUINT16 SensorPreviewWidthOffset;   /* from effective width to output width */
	MUINT16 SensorPreviewHeightOffset;  /* from effective height to output height */
	MUINT16 SensorFullWidthOffset;  /* from effective width to output width */
	MUINT16 SensorFullHeightOffset; /* from effective height to output height */
	MUINT16 SensorVideoWidthOffset; /* from effective width to output width */
	MUINT16 SensorVideoHeightOffset;    /* from effective height to output height */
	MUINT16 SensorHighSpeedVideoWidthOffset;    /* from effective width to output width */
	MUINT16 SensorHighSpeedVideoHeightOffset;   /* from effective height to output height */
	MUINT16 Sensor3DPreviewWidthOffset; /* from effective width to output width */
	MUINT16 Sensor3DPreviewHeightOffset;    /* from effective height to output height */
	MUINT16 Sensor3DFullWidthOffset;    /* from effective width to output width */
	MUINT16 Sensor3DFullHeightOffset;   /* from effective height to output height */
	MUINT16 Sensor3DVideoWidthOffset;   /* from effective width to output width */
	MUINT16 Sensor3DVideoHeightOffset;  /* from effective height to output height */
} ACDK_SENSOR_RESOLUTION_INFO_STRUCT, *PACDK_SENSOR_RESOLUTION_INFO_STRUCT;


typedef enum
{
  SENSOR_3D_NOT_SUPPORT = 0,
  SENSOR_3D_CAPTURE_FRAME_SEQUENTIAL,
  SENSOR_3D_CAPTURE_SIDE_BY_SIDE,
  SENSOR_3D_CAPTURE_TOP_BOTTOM,
}ACDK_SENSOR_3D_TYPE_ENUM;

typedef enum
{
	RAW_TYPE_10BIT = 0,
	RAW_TYPE_8BIT,
	RAW_TYPE_12BIT,
	RAW_TYPE_MAX
}ACDK_SENSOR_RAW_TYPE_ENUM;


typedef struct {
	MUINT16 SensorPreviewResolutionX;
	MUINT16 SensorPreviewResolutionY;
	MUINT16 SensorFullResolutionX;
	MUINT16 SensorFullResolutionY;
	MUINT8 SensorClockFreq; /* MHz */
	MUINT8 SensorCameraPreviewFrameRate;
	MUINT8 SensorVideoFrameRate;
	MUINT8 SensorStillCaptureFrameRate;
	MUINT8 SensorWebCamCaptureFrameRate;
	MUINT8 SensorClockPolarity; /* SENSOR_CLOCK_POLARITY_HIGH/SENSOR_CLOCK_POLARITY_Low */
	MUINT8 SensorClockFallingPolarity;
	MUINT8 SensorClockRisingCount;  /* 0..15 */
	MUINT8 SensorClockFallingCount; /* 0..15 */
	MUINT8 SensorClockDividCount;   /* 0..15 */
	MUINT8 SensorPixelClockCount;   /* 0..15 */
	MUINT8 SensorDataLatchCount;    /* 0..15 */
	MUINT8 SensorHsyncPolarity;
	MUINT8 SensorVsyncPolarity;
	MUINT8 SensorInterruptDelayLines;
	MINT32 SensorResetActiveHigh;
	MUINT32 SensorResetDelayCount;
	ACDK_SENSOR_INTERFACE_TYPE_ENUM SensroInterfaceType;
	ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM SensorOutputDataFormat;
	ACDK_SENSOR_MIPI_LANE_NUMBER_ENUM SensorMIPILaneNumber;
        CAMERA_ISO_BINNING_INFO_STRUCT  SensorISOBinningInfo;
	MUINT32 CaptureDelayFrame;
	MUINT32 PreviewDelayFrame;
	MUINT32 VideoDelayFrame;
	MUINT32 HighSpeedVideoDelayFrame;
	MUINT32 SlimVideoDelayFrame;
	MUINT32 YUVAwbDelayFrame;
	MUINT32 YUVEffectDelayFrame;
	MUINT32 Custom1DelayFrame;
	MUINT32 Custom2DelayFrame;
	MUINT32 Custom3DelayFrame;
	MUINT32 Custom4DelayFrame;
	MUINT32 Custom5DelayFrame;
	MUINT16 SensorGrabStartX;
	MUINT16 SensorGrabStartY;
	MUINT16 SensorDrivingCurrent;
	MUINT8 SensorMasterClockSwitch;
	MUINT8 AEShutDelayFrame;    /* The frame of setting shutter default 0 for TG int */
	MUINT8 AESensorGainDelayFrame;  /* The frame of setting sensor gain */
	MUINT8 AEISPGainDelayFrame;
	MUINT8 MIPIDataLowPwr2HighSpeedTermDelayCount;
	MUINT8 MIPIDataLowPwr2HighSpeedSettleDelayCount;
	MUINT8 MIPICLKLowPwr2HighSpeedTermDelayCount;
	MUINT8 SensorWidthSampling;
	MUINT8 SensorHightSampling;
	MUINT8 SensorPacketECCOrder;
	SENSOR_MIPI_TYPE_ENUM MIPIsensorType;
	MUINT8 SensorCaptureOutputJPEG; /* JPEG file or not? */
	MUINT8 SensorModeNum;
	MUINT8 IHDR_Support;
	MUINT16 IHDR_LE_FirstLine;
	SENSOR_SETTLEDELAY_MODE_ENUM SettleDelayMode;
	MUINT8 PDAF_Support;
	MUINT8 DPCM_INFO;
	MUINT8 PerFrameCTL_Support;
	SENSOR_SCAM_DATA_CHANNEL_ENUM SCAM_DataNumber;
	MUINT8 SCAM_DDR_En;
	MUINT8 SCAM_CLK_INV;
	MUINT8 SCAM_DEFAULT_DELAY;
	MUINT8 SCAM_CRC_En;
	MUINT8 SCAM_SOF_src;
	MUINT32 SCAM_Timout_Cali;
	ACDK_SENSOR_RAW_TYPE_ENUM  SensorRawType;
	ACDK_SENSOR_3D_TYPE_ENUM   SensorDriver3D;
} ACDK_SENSOR_INFO_STRUCT, *PACDK_SENSOR_INFO_STRUCT;

typedef struct {
	MUINT16 SensorPreviewResolutionX;
	MUINT16 SensorPreviewResolutionY;
	MUINT16 SensorFullResolutionX;
	MUINT16 SensorFullResolutionY;
	MUINT8 SensorClockFreq; /* MHz */
	MUINT8 SensorCameraPreviewFrameRate;
	MUINT8 SensorVideoFrameRate;
	MUINT8 SensorStillCaptureFrameRate;
	MUINT8 SensorWebCamCaptureFrameRate;
	MUINT8 SensorClockPolarity; /* SENSOR_CLOCK_POLARITY_HIGH/SENSOR_CLOCK_POLARITY_Low */
	MUINT8 SensorClockFallingPolarity;
	MUINT8 SensorClockRisingCount;  /* 0..15 */
	MUINT8 SensorClockFallingCount; /* 0..15 */
	MUINT8 SensorClockDividCount;   /* 0..15 */
	MUINT8 SensorPixelClockCount;   /* 0..15 */
	MUINT8 SensorDataLatchCount;    /* 0..15 */
	MUINT8 SensorHsyncPolarity;
	MUINT8 SensorVsyncPolarity;
	MUINT8 SensorInterruptDelayLines;
	MINT32 SensorResetActiveHigh;
	MUINT32 SensorResetDelayCount;
	ACDK_SENSOR_INTERFACE_TYPE_ENUM SensroInterfaceType;
	ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM SensorOutputDataFormat;
	ACDK_SENSOR_MIPI_LANE_NUMBER_ENUM SensorMIPILaneNumber;
	MUINT32 CaptureDelayFrame;
	MUINT32 PreviewDelayFrame;
	MUINT32 VideoDelayFrame;
	MUINT32 HighSpeedVideoDelayFrame;
	MUINT32 SlimVideoDelayFrame;
	MUINT32 YUVAwbDelayFrame;
	MUINT32 YUVEffectDelayFrame;
	MUINT32 Custom1DelayFrame;
	MUINT32 Custom2DelayFrame;
	MUINT32 Custom3DelayFrame;
	MUINT32 Custom4DelayFrame;
	MUINT32 Custom5DelayFrame;
	MUINT16 SensorGrabStartX_PRV;
	MUINT16 SensorGrabStartY_PRV;
	MUINT16 SensorGrabStartX_CAP;
	MUINT16 SensorGrabStartY_CAP;
	MUINT16 SensorGrabStartX_VD;
	MUINT16 SensorGrabStartY_VD;
	MUINT16 SensorGrabStartX_VD1;
	MUINT16 SensorGrabStartY_VD1;
	MUINT16 SensorGrabStartX_VD2;
	MUINT16 SensorGrabStartY_VD2;
	MUINT16 SensorGrabStartX_CST1;
	MUINT16 SensorGrabStartY_CST1;
	MUINT16 SensorGrabStartX_CST2;
	MUINT16 SensorGrabStartY_CST2;
	MUINT16 SensorGrabStartX_CST3;
	MUINT16 SensorGrabStartY_CST3;
	MUINT16 SensorGrabStartX_CST4;
	MUINT16 SensorGrabStartY_CST4;
	MUINT16 SensorGrabStartX_CST5;
	MUINT16 SensorGrabStartY_CST5;
	MUINT16 SensorDrivingCurrent;
	MUINT8 SensorMasterClockSwitch;
	MUINT8 AEShutDelayFrame;    /* The frame of setting shutter default 0 for TG int */
	MUINT8 AESensorGainDelayFrame;  /* The frame of setting sensor gain */
	MUINT8 AEISPGainDelayFrame;
	MUINT8 MIPIDataLowPwr2HighSpeedTermDelayCount;
	MUINT8 MIPIDataLowPwr2HighSpeedSettleDelayCount;
	MUINT8 MIPIDataLowPwr2HSSettleDelayM0;
	MUINT8 MIPIDataLowPwr2HSSettleDelayM1;
	MUINT8 MIPIDataLowPwr2HSSettleDelayM2;
	MUINT8 MIPIDataLowPwr2HSSettleDelayM3;
	MUINT8 MIPIDataLowPwr2HSSettleDelayM4;
	MUINT8 MIPICLKLowPwr2HighSpeedTermDelayCount;
	MUINT8 SensorWidthSampling;
	MUINT8 SensorHightSampling;
	MUINT8 SensorPacketECCOrder;
	SENSOR_MIPI_TYPE_ENUM MIPIsensorType;
	MUINT8 SensorCaptureOutputJPEG; /* JPEG file or not? */
	MUINT8 SensorModeNum;
	MUINT8 IHDR_Support;
	MUINT16 IHDR_LE_FirstLine;
	SENSOR_SETTLEDELAY_MODE_ENUM SettleDelayMode;
	MUINT8 PDAF_Support;
	MUINT8 DPCM_INFO;
	MUINT8 IMGSENSOR_DPCM_TYPE_PRE;
	MUINT8 IMGSENSOR_DPCM_TYPE_CAP;
	MUINT8 IMGSENSOR_DPCM_TYPE_VD;
	MUINT8 IMGSENSOR_DPCM_TYPE_VD1;
	MUINT8 IMGSENSOR_DPCM_TYPE_VD2;
	MUINT8 PerFrameCTL_Support;
	SENSOR_SCAM_DATA_CHANNEL_ENUM SCAM_DataNumber;
	MUINT8 SCAM_DDR_En;
	MUINT8 SCAM_CLK_INV;
	MUINT8 SCAM_DEFAULT_DELAY;
	MUINT8 SCAM_CRC_En;
	MUINT8 SCAM_SOF_src;
	MUINT32 SCAM_Timout_Cali;
} ACDK_SENSOR_INFO2_STRUCT, *PACDK_SENSOR_INFO2_STRUCT;


typedef enum {
	ACDK_CCT_REG_ISP = 0,
	ACDK_CCT_REG_CMOS,
	ACDK_CCT_REG_CCD
} ACDK_CCT_REG_TYPE_ENUM;


/* R/W ISP/Sensor Register */
typedef struct {
	ACDK_CCT_REG_TYPE_ENUM Type;
	MUINT32 RegAddr;
	MUINT32 RegData;
} ACDK_CCT_REG_RW_STRUCT, *PACDK_CCT_REG_RW_STRUCT;

typedef struct {
	ACDK_CCT_REG_TYPE_ENUM Type;    /* ISP, CMOS_SENSOR, CCD_SENSOR */
	MUINT32 DeviceId;
	ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM StartPixelBayerPtn;
	MUINT16 GrabXOffset;
	MUINT16 GrabYOffset;
} ACDK_CCT_SENSOR_INFO_STRUCT, *PACDK_CCT_SENSOR_INFO_STRUCT;



typedef enum {
	CMOS_SENSOR = 0,
	CCD_SENSOR
} SENSOR_TYPE_ENUM;

typedef struct {
	MUINT16 SensorId;
	SENSOR_TYPE_ENUM SensorType;
	ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM SensorOutputDataFormat;
} ACDK_SENSOR_ENG_INFO_STRUCT;

typedef struct {
	MUINT32 RegAddr;
	MUINT32 RegData;
} ACDK_SENSOR_REG_INFO_STRUCT;

typedef struct {
	MUINT8  debugStruct[32];
	MUINT8  debugSubstruct[32];
	MUINT32 isGet;              /* 1 means get, 0 means set */
	MUINT32 value;
} ACDK_SENSOR_DBG_IMGSENSOR_STRUCT;

typedef struct {
	MUINT32 GroupIdx;
	MUINT32 ItemCount;
	MUINT8 *GroupNamePtr;
} ACDK_SENSOR_GROUP_INFO_STRUCT;

typedef struct {
	MUINT32 GroupIdx;
	MUINT32 ItemIdx;
	MUINT8 ItemNamePtr[50]; /* item name */
	MUINT32 ItemValue;  /* item value */
	MINT32 IsTrueFalse; /* is this item for enable/disable functions */
	MINT32 IsReadOnly;  /* is this item read only */
	MINT32 IsNeedRestart;   /* after set this item need restart */
	MUINT32 Min;        /* min value of item value */
	MUINT32 Max;        /* max value of item value */
} ACDK_SENSOR_ITEM_INFO_STRUCT;

typedef enum {
	ACDK_SENSOR_IMAGE_NORMAL = 0,
	ACDK_SENSOR_IMAGE_H_MIRROR,
	ACDK_SENSOR_IMAGE_V_MIRROR,
	ACDK_SENSOR_IMAGE_HV_MIRROR
} ACDK_SENSOR_IMAGE_MIRROR_ENUM;

typedef enum {
	ACDK_SENSOR_OPERATION_MODE_CAMERA_PREVIEW = 0,
	ACDK_SENSOR_OPERATION_MODE_VIDEO,
	ACDK_SENSOR_OPERATION_MODE_STILL_CAPTURE,
	ACDK_SENSOR_OPERATION_MODE_HIGHSP_VIDEO1,
	ACDK_SENSOR_OPERATION_MODE_HIGHSP_VIDEO2,
	ACDK_SENSOR_OPERATION_MODE_WEB_CAPTURE,
	ACDK_SENSOR_OPERATION_MODE_CUSTOM1,
	ACDK_SENSOR_OPERATION_MODE_CUSTOM2,
	ACDK_SENSOR_OPERATION_MODE_CUSTOM3,
	ACDK_SENSOR_OPERATION_MODE_CUSTOM4,
	ACDK_SENSOR_OPERATION_MODE_CUSTOM5,
	ACDK_SENSOR_OPERATION_MODE_MAX
} ACDK_SENSOR_OPERATION_MODE_ENUM;

typedef struct {
	MUINT16 GrabStartX; /* The first grabed column data of the image sensor in pixel clock count */
	MUINT16 GrabStartY; /* The first grabed row data of the image sensor in pixel clock count */
	MUINT16 ExposureWindowWidth;    /* Exposure window width of image sensor */
	MUINT16 ExposureWindowHeight;   /* Exposure window height of image sensor */
	MUINT16 ImageTargetWidth;   /* image captured width */
	MUINT16 ImageTargetHeight;  /* image captuerd height */
	MUINT16 ExposurePixel;  /* exposure window width of image sensor + dummy pixel */
	MUINT16 CurrentExposurePixel;   /* exposure window width of image sensor + dummy pixel */
	MUINT16 ExposureLine;   /* exposure window width of image sensor + dummy line */
	MUINT16 ZoomFactor; /* digital zoom factor */
} ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT;

typedef struct {
	ACDK_SENSOR_IMAGE_MIRROR_ENUM SensorImageMirror;
	MINT32 EnableShutterTansfer;    /* capture only */
	MINT32 EnableFlashlightTansfer; /* flash light capture only */
	ACDK_SENSOR_OPERATION_MODE_ENUM SensorOperationMode;
	MUINT16 ImageTargetWidth;   /* image captured width */
	MUINT16 ImageTargetHeight;  /* image captuerd height */
	MUINT16 CaptureShutter; /* capture only */
	MUINT16 FlashlightDuty; /* flash light capture only */
	MUINT16 FlashlightOffset;   /* flash light capture only */
	MUINT16 FlashlightShutFactor;   /* flash light capture only */
	MUINT16 FlashlightMinShutter;
	ACDK_CAMERA_OPERATION_MODE_ENUM MetaMode;   /* capture only */
	MUINT32 DefaultPclk;    /* Sensor pixel clock(Ex:24000000) */
	MUINT32 Pixels;     /* Sensor active pixel number */
	MUINT32 Lines;      /* Sensor active line number */
	MUINT32 Shutter;    /* Sensor current shutter */
	MUINT32 FrameLines; /* valid+dummy lines for minimum shutter */
} ACDK_SENSOR_CONFIG_STRUCT;

typedef enum {
	MCLK_48MHZ_GROUP = 0x1,
	MCLK_52MHZ_GROUP = 0x2,
} ACKD_SENSOR_MCLK_ENUM;

typedef struct {
	MUINT8 on;
	ACKD_SENSOR_MCLK_ENUM freq;
} ACDK_SENSOR_MCLK_STRUCT;

typedef struct {
	MUINT16 full_w;
	MUINT16 full_h;
	MUINT16 x0_offset;
	MUINT16 y0_offset;
	MUINT16 w0_size;
	MUINT16 h0_size;
	MUINT16 scale_w;
	MUINT16 scale_h;
	MUINT16 x1_offset;
	MUINT16 y1_offset;
	MUINT16 w1_size;
	MUINT16 h1_size;
	MUINT16 x2_tg_offset;
	MUINT16 y2_tg_offset;
	MUINT16 w2_tg_size;
	MUINT16 h2_tg_size;
} SENSOR_WINSIZE_INFO_STRUCT;

typedef struct {
	MUINT32 SensorId;
	SENSOR_WINSIZE_INFO_STRUCT   SensorWinSize;
} SENSOR_WINSIZE_STRUCT, *pSENSOR_WINSIZE_STRUCT;

typedef struct {
	MUINT16 VC_Num;
	MUINT16 VC_PixelNum;
	MUINT16 ModeSelect;   /* 0: auto mode, 1:direct mode  */
	MUINT16 EXPO_Ratio;   /* 1/1, 1/2, 1/4, 1/8 */
	MUINT16 ODValue;      /* OD Vaule */
	MUINT16 RG_STATSMODE; /* STATS divistion mdoe 0: 16x16, 1:8x8, 2:4x4, 3:1x1*/
	MUINT16 VC0_ID;
	MUINT16 VC0_DataType;
	MUINT16 VC0_SIZEH;
	MUINT16 VC0_SIZEV;
	MUINT16 VC1_ID;
	MUINT16 VC1_DataType;
	MUINT16 VC1_SIZEH;
	MUINT16 VC1_SIZEV;
	MUINT16 VC2_ID;
	MUINT16 VC2_DataType;
	MUINT16 VC2_SIZEH;
	MUINT16 VC2_SIZEV;
	MUINT16 VC3_ID;
	MUINT16 VC3_DataType;
	MUINT16 VC3_SIZEH;
	MUINT16 VC3_SIZEV;
} SENSOR_VC_INFO_STRUCT, *pSENSOR_VC_INFO_STRUCT;
typedef struct {
	MUINT32 ABS_GAIN_GR;
	MUINT32 ABS_GAIN_R;
	MUINT32 ABS_GAIN_B;
	MUINT32 ABS_GAIN_GB;
} SET_SENSOR_AWB_GAIN, *PSET_SENSOR_AWB_GAIN;
typedef struct {
	MUINT32 i4OffsetX;      /* start offset of first PD block */
	MUINT32 i4OffsetY;
	MUINT32 i4PitchX;       /* PD block pitch */
	MUINT32 i4PitchY;
	MUINT32 i4PairNum;      /* PD pair num in one block */
	MUINT32 i4SubBlkW;      /* sub block width (one PD pair in one sub block) */
	MUINT32 i4SubBlkH;      /* sub block height */
	MUINT32 i4PosL[16][2];  /* left pd pixel position in one block*/
	MUINT32 i4PosR[16][2];  /* right pd pixel position in one block*/
} SET_PD_BLOCK_INFO_T, *PSET_PD_BLOCK_INFO_T;


typedef enum {
	SENSORIF_PARALLEL = 0,
	SENSORIF_SERIAL,
	SENSORIF_MIPI_DHY0,
	SENSORIF_MIPI_DHY1,
	SENSORIF_MIPI_DHY2
} SENSOR_PHY_TYPE_ENUM;

typedef enum {
	DATA_8BIT_FMT        = 0x0,
	DATA_10BIT_FMT       = 0x1,
	DATA_12BIT_FMT       = 0x2,
	DATA_YUV422_FMT      = 0x3,
} INDATA_FORMAT_ENUM;

typedef struct {
	MINT32 mSensorDev;
	MUINT8 GpioEnable;
	SENSOR_PHY_TYPE_ENUM SensroInterfaceType;
	INDATA_FORMAT_ENUM SensorIndataformat;
} IMGSENSOR_GPIO_STRUCT;
/*******************************************************************************
*
********************************************************************************/

//* function pointer that provided to MSDK, all return value are declared as MUINT32 are
//   reserved for returing error code if necessary. Currently, it is not necessary, please return
//   ERROR_NONE. This function pointer structure is defined here because sensor driver should
//   be customized, the developer has to comply with the rule. */
//typedef struct
//{
//   MUINT32 (* SensorOpen)(void);
//   MUINT32 (* SensorGetInfo) (MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
//                               MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
//   MUINT32 (* SensorGetResolution) (MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
//   MUINT32 (* SensorFeatureControl) (MSDK_SENSOR_FEATURE_ENUM FeatureId, MUINT8 *pFeaturePara,MUINT32 *pFeatureParaLen);
//   MUINT32 (* SensorControl) (MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
//   MUINT32 (* SensorClose)(void);
//  // For YUV Sensor
//} SENSOR_FUNCTION_STRUCT, *PSENSOR_FUNCTION_STRUCT;
//
//typedef struct
//{
				 
				   
				   
				 
				 
					  
					  
					
					
							 

				
				  
											
												 

				
				
					 
														 
											   
									 
																			  
				
					  
				   
				   
				
					  
				   
				   
				
					  
				   
				   
				
					  
				   
				   
												 
				
					 
					
					
					 
											 
				
															 
				   
											 
//  MUINT32 SensorId;
//  MUINT32 (* SensorInit)(PSENSOR_FUNCTION_STRUCT *pfFunc);
//} MSDK_SENSOR_INIT_FUNCTION_STRUCT, *PMSDK_SENSOR_INIT_FUNCTION_STRUCT;
//
//MUINT32 GetSensorInitFuncList(PMSDK_SENSOR_INIT_FUNCTION_STRUCT pSensorList);
//#endif /* __MSDK_Sensor_if_H */
											 


			  
					   
				 
					
					
				   
					   

			  
							
							
							
							
					 

				
				   
				   
										  
									   
						
/*******************************************************************************
*
********************************************************************************/
//
//msdk_nvram_camera_exp.h
//

//#ifndef __MSDK_NVRAM_CAMERA_EXP_H
//#define __MSDK_NVRAM_CAMERA_EXP_H
//
////#include "msdk_isp_exp.h"
//typedef unsigned char MUINT8;
//typedef unsigned short MUINT16;
//typedef unsigned int MUINT32;
//
//#define MAXIMUM_NVRAM_CAMERA_PARA_FILE_SIZE         8192
//#define MAXIMUM_NVRAM_CAMERA_3A_FILE_SIZE           8192
//#define MAXIMUM_NVRAM_CAMERA_SHADING_FILE_SIZE      20480
//#define MAXIMUM_NVRAM_CAMERA_DEFECT_FILE_SIZE       20480
#define MAXIMUM_NVRAM_CAMERA_SENSOR_FILE_SIZE       4096

//#define MAIN_NVRAM_CAMERA_PARA_FILE_OFFSET      4096
//#define MAIN_NVRAM_CAMERA_3A_FILE_OFFSET        4096
//#define MAIN_NVRAM_CAMERA_SHADING_FILE_OFFSET   10240
//#define MAIN_NVRAM_CAMERA_DEFECT_FILE_OFFSET    10240
//#define MAIN_NVRAM_CAMERA_SENSOR_FILE_OFFSET    2048
//
//#define NVRAM_CAMERA_PARA_FILE_VERSION          1
//#define NVRAM_CAMERA_3A_FILE_VERSION            1
//#define NVRAM_CAMERA_SHADING_FILE_VERSION       1
//#define NVRAM_CAMERA_DEFECT_FILE_VERSION        1
#define NVRAM_CAMERA_SENSOR_FILE_VERSION        1

//// camera common parameters and sensor parameters
//typedef struct
//{
//    MUINT32 CommReg[64];
//} ISP_COMMON_PARA_STRUCT, *PISP_COMMON_PARA_STRUCT;
//
//typedef struct
//{
//    MUINT8 Shading;
//    MUINT8 NR1;
//    MUINT8 NR2;
//    MUINT8 Edge;
//    MUINT8 AutoDefect;
//    MUINT8 Saturation;
//    MUINT8 Contrast;
//    MUINT8 Reserved;
//}   ISP_TUNING_INDEX_STRUCT, *PISP_TUNING_INDEX_STRUCT;
//
//typedef struct
//{
//    ISP_TUNING_INDEX_STRUCT Idx;
//    MUINT32 ShadingReg[3][5];//binning
//    MUINT32 NR1Reg[7][11];
//    MUINT32 NR2Reg[7][5];
//    MUINT32 EdgeReg[7][3];
//    MUINT32 AutoDefect[3][5];
//    MUINT32 Saturation[7][4];
//    MUINT32 Contrast[3][3];
//} ISP_TUNING_PARA_STRUCT, *PISP_TUNING_PARA_STRUCT;
//
//typedef struct
//{
//    MUINT32 TargetTime;/*minimum exposure time, unit:(us)*/
//    MUINT32 ShutterDelayTime; /*shutter delay time*/
//}   SHUTTER_DELAY_STRUCT, *PSHUTTER_DELAY_STRUCT;
//
//typedef struct
//{
//    MUINT32 Version;
//    MUINT32 SensorId;        // ID of sensor module
//    ISP_COMMON_PARA_STRUCT  ISPComm;
//    ISP_TUNING_PARA_STRUCT  ISPTuning;
//    SHUTTER_DELAY_STRUCT MShutter;
//    MUINT8 CameraData[MAXIMUM_NVRAM_CAMERA_PARA_FILE_SIZE/2-8-sizeof(ISP_COMMON_PARA_STRUCT)-sizeof(ISP_TUNING_PARA_STRUCT)-sizeof(SHUTTER_DELAY_STRUCT)];
//} NVRAM_CAMERA_PARA_STRUCT, *PNVRAM_CAMERA_PARA_STRUCT;
//
//// camera 3A parameters
//#define MAX_ISO_LUT_NO 3
//#define AF_MAX_DEF_PATH_NO      (3) // AF PATH NO in NVRAM
//#define AF_MAX_TABLE_NO         (24)
//#define AF_MZ_DOF_MAX_NO        (8)
//#define AF_MZ_DOF_LUT_IDX_NO    (3)
//#define AE_TOTAL_GAMMA_NO 5
//#define GAMMA_FAST_LUT_NO 64
//
//typedef enum
//{
//    AF_PARAM_VERSION=0,
//    AF_PARAM_PEAK_THRE,
//    AF_PARAM_NOFOCUS_THRE,
//    AF_PARAM_STOP_THRE,
//    AF_PARAM_HYSTERESIS_THRE,
//    AF_PARAM_HYSTERESIS_DIR,
//    AF_PARAM_SLOW_MOVE_MAX_STEP,
//    AF_PARAM_SLOW_BACK_MAX_STEP,
//    AF_PARAM_WAIT_STABLE_THRE2,
//    AF_PARAM_WAIT_STABLE_THRE3,
//    AF_PARAM_STARTUP_POS,
//    AF_PARAM_STARTUP_WAIT_FRAME,
//    AF_PARAM_SEARCH_OPTION,
//    // for continuous AF
//    AF_PARAM_CONT_DD_STEP_RATIO,
//    AF_PARAM_CONT_DD_RANGE_THRE,
//    AF_PARAM_CONT_FV_DELTA, //0828NEW
//    AF_PARAM_CONT_FV_BLUR,  //0828NEW
//    AF_PARAM_CONT_FV_DELTA_HIGH,
//    AF_PARAM_CONT_FV_DELTA_LOW,
//    AF_PARAM_CONT_AE_DELTA_HIGH,
//    AF_PARAM_CONT_AE_DELTA_LOW,
//    AF_PARAM_CONT_WAIT_STABLE_AE,
//    AF_PARAM_CONT_WAIT_STABLE_FV_HIGH,
//    AF_PARAM_CONT_WAIT_STABLE_FV_LOW,
//    AF_PARAM_CONT_WAIT_STABLE_FREEZE,
//    AF_PARAM_CONT_JUMP_COUNT_THRE,
//    AF_PARAM_CONT_WAIT_STABLE_MAX,
//    // for MZ AF
//    AF_PARAM_MZ_MAX_INF_DOF,
//    AF_PARAM_MZ_MAX_MACRO_DOF,
//    // for continuous AF append
//    AF_PARAM_CONT_PAN_LUMA_DELTA, // 0832NEW
//    AF_PARAM_CONT_PAN_HOLD_FRAME, // 0832NEW
//    AF_PARAM_CONT_PAN_OVER_SPEEDUP, // 0832NEW
//    AF_PARAM_CONT_WINDOW_CONFIG_X,// 0832NEW
//    AF_PARAM_CONT_WINDOW_CONFIG_Y,// 0832NEW
//    AF_PARAM_CONT_STOP_SEARCH_CONFIG,// 0836NEW
//    // for single/multi window AF append
//    AF_PARAM_WIN_SIZE_LEVEL_CONFIG,// 0836NEW
//    // END TAG
//    AF_PARAM_END_TAG,
//    AF_PARAM_MAX_NO = 38
//} AF_PARAM_ENUM;
//
//enum
//{
//    AE_SELECT_PREIVEW=0,
//    AE_SELECT_VIDEO,
//    AE_SELECT_NIGHT,
//    AE_SELECT_MODE_NO
//};
//
////AE
///* ISO priority */
//typedef struct
//{
//    MUINT16  IsoGain[MAX_ISO_LUT_NO];
//    MUINT16  IsoValue[MAX_ISO_LUT_NO];
//}aeIsoLutStruct, MSDK_CCT_AE_ISO_LUT_STRUCT, *PMSDK_CCT_AE_ISO_LUT_STRUCT;
//
//typedef struct
//{
//    MUINT8 target_mean;
//    MUINT8 init_expo_idx;
//    /* capture flare offset cal */
//    MUINT8 flare_fix_flag;
//    MUINT8   flare_fix_offset;
//    MUINT8   flare_search_dn_ratio;
//    char    flare_min_offset;
//    /* preview & video gamma no */
//    MUINT8 preview_gamma_no;
//    MUINT8 preview_flare_offset;
//    /* gamma mean */
//    MUINT8 indoor_outdoor_ev;
//    MUINT8 indoor_min_gamma_mean;
//    MUINT8 outdoor_max_gamma_mean;
//    /* sky detection */
//    MUINT8 sky_ev;
//    MUINT8 sky_mean;
//    MUINT8 sky_max_window_mean;
//    /* max under expo bin */
//    MUINT8 min_under_expo_bin;
//    /* lowlight gamma limit */
//    MUINT8 lowlight_ev;
//    MUINT8 lowlight_min_target_mean;
//    /* enable flag */
//    MUINT8 non_over_exposure_stretch_enabled;
//    MUINT8 full_dark_background_check_enabled;
//    MUINT8 backlight_stretch_enabled;
//    /* fast gamma lut for gamma mean */
//    MUINT8 gamma_lut[AE_TOTAL_GAMMA_NO][GAMMA_FAST_LUT_NO];
//    MUINT8 grass_detect_enable;/* grass detection flag */
//    MUINT8 reserve_2;/* padding for even size */
//}aeMeteringCommStruct, MSDK_CCT_AE_METERING_COMM_STRUCT, *PMSDK_CCT_AE_METERING_COMM_STRUCT;
//
//typedef struct
//{
//    /* method select */
//    MUINT8 aeSmoothMethod;/* 0:disabled, 1:enable */
//    MUINT8 aeSmoothFilterTape;/* media filter tape no, NA for curve like enable */
//    /* normal */
//    MUINT8 aeSmoothNormalLastWeight;/* 0~128 */
//    MUINT8 aeSmoothNormalMaxStep;/* 1~4, NA for curve like enable */
//    /* fast */
//    MUINT8 aeSmoothFastLastWeight;/* 0~128, NA for curve like enable */
//    MUINT8 aeSmoothFastMaxStep;/* 5~40, NA for curve like enable */
//    MUINT8 aeSmoothFastMaxChangeStep;/* 5~40, NA for curve like enable */
//    MUINT8 aeSmoothFastCheckCount;/* 0~3 */
//    MUINT8 aeSmoothFastBackCheckCount;/* 0~3, NA for curve like enable */
//    MUINT8 aeSmoothFastBackCount;/* 0~3, NA for curve like enable */
//    MUINT8 aeSmoothFastTransitionCount;/* 0~3, NA for curve like enable */
//    MUINT8 aeSmoothNormalToFastTh;/* 1~4 */
//    MUINT8 aeSmoothFastToNormalTh;/* 5~15*/
//    /* 6328 new smooth mode */
//    MUINT8 aeSmoothFastSmoothEnable;/* 0: disable, 1:enable */
//    MUINT8 aeSmoothNormalSpeedEnable;/* 0: disable, 1:enable */
//    MUINT8 aeSmoothMovingStopCheckEnable;/* 0: disable, 1:enable */
//    MUINT8 aeSmoothMovingStopNormalCheckCount;/* 1~4 check count */
//    MUINT8 aeSmoothMovingStopFastCheckCount;/* 1~4 check count */
//    MUINT8 reserve_1;/* padding for even size */
//    MUINT8 reserve_2;/* padding for even size */
//}aeSmoothModeStruct;
//
//typedef struct
//{
//    aeIsoLutStruct aeIsoPara;/* 2*6=12 */
//    aeMeteringCommStruct aeMeteringComm;/* 22+5*64=342 */
//    aeSmoothModeStruct ae_smooth[AE_SELECT_MODE_NO];/* 3*20=60 */
//    MUINT8 reserve_1[66];/* 480-12-342-60=66 */
//} nvram_ae_para_struct, NVRAM_AE_PARA_STRUCT, *PNVRAM_AE_PARA_STRUCT;
//
////AWB
//typedef struct
//{
//    MUINT32      reg[30];//0x280~0x2F8
//    MUINT16  LightSource[7][4];  /* { Rgain, GRgain, Bgain, GBgain} */
//                                                            /*A, TL84,CWF,D65,D75,Flash,Manual*/
//    MUINT8        AWB_speed_fast;    /*awb converge ratio in 1 step in fast state
//                                                          when gain difference is higher than AWB_thres_fast, 128 base*/
//    MUINT8        AWB_speed_mid;     /*awb converge ratio in 1 step in fast state
//                                                          when gain difference is between AWB_thres_mid and AWB_thres_fast, 128 base*/
//    MUINT8        AWB_speed_slow;    /*awb converge ratio in 1 step in fast state
//                                                          when gain difference is smaller than AWB_thres_mid , 128 base*/
//    MUINT8        AWB_thres_fast;      /*gain difference for AWB_speed_fast*/
//    MUINT8        AWB_thres_mid;      /*gain difference for AWB_speed_mid*/
//    MUINT8        AWBPreference[3][3];  /*A, TL8, Daylight; RcompRatio,GcompRatio,BcompRatio*/
//    MUINT8        change_state_gain_thr;  /*gain difference between target & current to enter change state*/
//    MUINT8        change_state_frame_thr; /*frame number to enter change state when gain difference is over threshold*/
//    MUINT8        stable_state_gain_thr;  /*gain difference between target & current to enter stable state*/
//    MUINT8        stable_state_frame_thr;  /*frame number to enter stable state when gain difference is smaller than threshold*/
//    MUINT8        finetune_state_gain_thr; /*gain difference between target & current to enter finetune state*/
//    MUINT8        finetune_state_frame_thr;/*frame number to enter finetuen state when gain difference is over threshold*/
//    MUINT8        awbSmFrameNum;  /*Frame number used to smooth current target gain*/
//    MUINT8        AWBPreferEn;    /*Enable awb preference**/
//    MUINT8        MWBFixedWinEn;  /*0: fixed gain for MWB, 1:fixed window for MWB*/
//    MUINT8        ALightMethod;//0: Use TL84, 1: Use Small A box
//    MUINT8        awbSmallAWinEn; //use small a window
//    MUINT8        awbSmallTL84WinEn; //use small TL84 window
//    MUINT8        awbSmallCWFWinEn; //use small CWF window
//    MUINT8        awbSmallDWinEn; //use small Day window
//    MUINT8        awbCaptureFastConvergeEn;//capture fast converge
//    MUINT8        reserved[25];
//}awb_v2_para_struct, AWB_V2_PARA_STRUCT, *PAWB_V2_PARA_STRUCT;
//
//typedef struct
//{
//    MUINT8       LightSource[3][3][3];
//    MUINT8       CCMPreferEn;
//}ccm_para_struct, CCM_PARA_STRUCT, *PCCM_PARA_STRUCT;
//
//typedef struct
//{
//    awb_v2_para_struct              AWB;
//    ccm_para_struct                 CCM;
//}nvram_awb_para_struct, NVRAM_AWB_PARA_STRUCT, *PNVRAM_AWB_PARA_STRUCT;
//
//// AF
//
//typedef struct {
//    MUINT16      af_calibration_offset;
//    MUINT16      af_calibration_data;
//} nvram_lens_calibration_struct;
//
//typedef struct
//{
//    MUINT16      af_table_num;
//    MUINT16      af_home_idx;
//    MUINT16      af_macro_idx;
//    MUINT16      af_infinite_idx;
//    MUINT16      af_hyper_pos;
//    MUINT16      af_me_home_pos;
//    MUINT16      af_me_macro_pos;
//    MUINT16      af_filter_type;
//} af_table_range_struct;
//
//typedef struct
//{
//    MUINT16  pos;
//    MUINT16  distance;
//} af_table_step_struct;
//
//typedef struct {
//    af_table_range_struct   table_range;
//    af_table_step_struct    table_step[AF_MAX_TABLE_NO];
//} nvram_af_table_info_struct;
//
//typedef struct
//{
//    MUINT16 dof_lut_size;
//    MUINT16 dof_lut[AF_MZ_DOF_MAX_NO][AF_MZ_DOF_LUT_IDX_NO];/* 0: step, 1:macro, 2:inf */
//}nvram_af_mz_dof_info_struct;
//
//typedef struct
//{
//    nvram_lens_calibration_struct   lens_calibration;           // 1 words
//    nvram_af_table_info_struct      table_info[AF_MAX_DEF_PATH_NO];     // provide 4+24 words * 3 = 84 words
//    nvram_af_mz_dof_info_struct     dof_table_info;/* 2+3*2*8 = 50 bytes = 25 words */
//    MUINT16                          para_info[AF_PARAM_MAX_NO]; // 32/2 = 16 words + 3 words
//} nvram_af_para_struct, NVRAM_AF_PARA_STRUCT, *PNVRAM_AF_PARA_STRUCT;
//
//typedef struct
//{
//    MUINT32 Version;
//    MUINT32 SensorId;        // ID of sensor module
//    NVRAM_AE_PARA_STRUCT    AE;
//    NVRAM_AWB_PARA_STRUCT   AWB;
//    NVRAM_AF_PARA_STRUCT    AF;
//    MUINT8 CameraData[MAXIMUM_NVRAM_CAMERA_3A_FILE_SIZE/2-8-sizeof(NVRAM_AE_PARA_STRUCT)-sizeof(NVRAM_AWB_PARA_STRUCT)-sizeof(NVRAM_AF_PARA_STRUCT)];
//} NVRAM_CAMERA_3A_STRUCT, *PNVRAM_CAMERA_3A_STRUCT;
//
//
////Defect table
//#define MAX_DEFECT_PRV_SIZE     (255)
//#define MAX_DEFECT_CAP_SIZE     (511)
//
//typedef struct
//{
//    MUINT32 Version;
//    MUINT32 SensorId;        // ID of sensor module
//    MUINT16 PreviewSize;
//    MUINT16 CaptureSize;
//    MUINT32 PreviewTable[MAX_DEFECT_PRV_SIZE];
//    MUINT32 CaptureTable1[MAX_DEFECT_CAP_SIZE];
//    MUINT8 CameraData[MAXIMUM_NVRAM_CAMERA_DEFECT_FILE_SIZE/2-12-4*MAX_DEFECT_PRV_SIZE-4*MAX_DEFECT_CAP_SIZE];
//} ISP_DEFECT_STRUCT, *PISP_DEFECT_STRUCT;
//
//typedef struct
//{
//   ISP_DEFECT_STRUCT Defect;
//} NVRAM_CAMERA_DEFECT_STRUCT, *PNVRAM_CAMERA_DEFECT_STRUCT;
//
//// Shading table
//#define MAX_SHADING_PRV_SIZE    (320)
//#define MAX_SHADING_CAP_SIZE    (896)
//
//typedef struct
//{
//    MUINT32 Version;
//    MUINT32 SensorId;        // ID of sensor module
//    MUINT16 PreviewSize;
//    MUINT16 CaptureSize;
//    MUINT32 PreviewTable[MAX_SHADING_PRV_SIZE];
//    MUINT32 CaptureTable[MAX_SHADING_CAP_SIZE];
//    MUINT8 CameraData[MAXIMUM_NVRAM_CAMERA_SHADING_FILE_SIZE/2-12-4*MAX_SHADING_PRV_SIZE-4*MAX_SHADING_CAP_SIZE];
//} ISP_SHADING_STRUCT, *PISP_SHADING_STRUCT;
//
//typedef struct
//{
//    ISP_SHADING_STRUCT  Shading;
//} NVRAM_CAMERA_SHADING_STRUCT, *PNVRAM_CAMERA_SHADING_STRUCT;






/* Sensor table */
#define MAXIMUM_SENSOR_CCT_REG_NUMBER   100
#define MAXIMUM_SENSOR_ENG_REG_NUMBER   100

typedef struct {
	MUINT32 Addr;
	MUINT32 Para;
} SENSOR_REG_STRUCT;

typedef struct {
	MUINT32 Version;
	MUINT32 SensorId;   /* ID of sensor module */
	SENSOR_REG_STRUCT SensorEngReg[MAXIMUM_SENSOR_ENG_REG_NUMBER];
	SENSOR_REG_STRUCT SensorCCTReg[MAXIMUM_SENSOR_CCT_REG_NUMBER];
	MUINT8 CameraData[MAXIMUM_NVRAM_CAMERA_SENSOR_FILE_SIZE / 2 - 8 - sizeof(SENSOR_REG_STRUCT) * (MAXIMUM_SENSOR_ENG_REG_NUMBER + MAXIMUM_SENSOR_CCT_REG_NUMBER)];
} NVRAM_SENSOR_DATA_STRUCT, *PNVRAM_SENSOR_DATA_STRUCT;

#define MAX_SENSOR_CAL_SIZE_KERNEL     (1024)   /* Byte */
#define MAX_SHADING_DATA_TBL ((MAX_SENSOR_CAL_SIZE_KERNEL-8)/4)
typedef struct
{
	MUINT32 DataFormat;
	MUINT32 DataSize;
	MUINT32 ShadingData[MAX_SHADING_DATA_TBL];
} SET_SENSOR_CALIBRATION_DATA_STRUCT, *PSET_SENSOR_CALIBRATION_DATA_STRUCT;

//void GetCameraDefaultPara(MUINT32 SensorId,
//                          PNVRAM_CAMERA_PARA_STRUCT pCameraParaDefault,
//                          PNVRAM_CAMERA_3A_STRUCT pCamera3ADefault,
//                          PNVRAM_CAMERA_SHADING_STRUCT pCameraShadingDefault,
//                          PNVRAM_CAMERA_DEFECT_STRUCT pCameraDefectDefault);
//
///* define the LID and total record for NVRAM interface */
//#define CFG_FILE_CAMERA_PARA_REC_SIZE           sizeof(NVRAM_CAMERA_PARA_STRUCT)
//#define CFG_FILE_CAMERA_3A_REC_SIZE             sizeof(NVRAM_CAMERA_3A_STRUCT)
//#define CFG_FILE_CAMERA_SHADING_REC_SIZE        sizeof(NVRAM_CAMERA_SHADING_STRUCT)
//#define CFG_FILE_CAMERA_DEFECT_REC_SIZE         sizeof(NVRAM_CAMERA_DEFECT_STRUCT)
//#define CFG_FILE_CAMERA_SENSOR_REC_SIZE         sizeof(NVRAM_SENSOR_DATA_STRUCT)
//
//#define CFG_FILE_CAMERA_PARA_REC_TOTAL          1
//#define CFG_FILE_CAMERA_3A_REC_TOTAL            1
//#define CFG_FILE_CAMERA_SHADING_REC_TOTAL       1
//#define CFG_FILE_CAMERA_DEFECT_REC_TOTAL        1
//#define CFG_FILE_CAMERA_SENSOR_REC_TOTAL        1
//
//#endif /* __MSDK_NVRAM_CAMERA_EXP_H */
/*******************************************************************************
*
********************************************************************************/
/*******************************************************************************
*
********************************************************************************/
/*******************************************************************************
*
********************************************************************************/
/*******************************************************************************
*
********************************************************************************/
/*******************************************************************************
*
********************************************************************************/
//kd_imgsensor.h

typedef struct {
	MSDK_SCENARIO_ID_ENUM ScenarioId[2];
	ACDK_SENSOR_INFO_STRUCT * pInfo[2];
	ACDK_SENSOR_CONFIG_STRUCT * pConfig[2];
} ACDK_SENSOR_GETINFO_STRUCT, *PACDK_SENSOR_GETINFO_STRUCT;

typedef struct {
	MUINT32 SensorId;
	ACDK_SENSOR_INFO2_STRUCT *pInfo;
	ACDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution;
} IMAGESENSOR_GETINFO_STRUCT, *PIMAGESENSOR_GETINFO_STRUCT;

typedef struct {
	CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera;
	ACDK_SENSOR_FEATURE_ENUM FeatureId;
	MUINT8 *pFeaturePara;
	MUINT32 *pFeatureParaLen;
} ACDK_SENSOR_FEATURECONTROL_STRUCT, *PACDK_SENSOR_FEATURECONTROL_STRUCT;

typedef struct {
	CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera;
	MSDK_SCENARIO_ID_ENUM ScenarioId;
	ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow;
	ACDK_SENSOR_CONFIG_STRUCT *pSensorConfigData;
} ACDK_SENSOR_CONTROL_STRUCT;

typedef struct {
	ACDK_SENSOR_RESOLUTION_INFO_STRUCT *pResolution[2];
} ACDK_SENSOR_PRESOLUTION_STRUCT;



typedef struct regval_list {
	MUINT32 reg_addr;
	MUINT32 value;
	MUINT32 bytes;
} REGVAL_LIST_STRUCT;

#define KDIMGSENSOR_REGVAL_LIST_MAX_NUM 256

typedef struct format_struct {
	MUINT8 *desc;
	MUINT32 pixelformat;
	/* REGVAL_LIST_STRUCT regs[KDIMGSENSOR_REGVAL_LIST_MAX_NUM]; */
	int (*pfInitCfg)(void);
} IMGSENSOR_FORMAT_STRUCT;

typedef struct {
	IMGSENSOR_FORMAT_STRUCT format;
	MUINT32 u4InClk;    /* Common part                                     //hard coded */
	MUINT32 u4OutClk;   /* Common part                                    // */
	MUINT32 u4TotalPixelPerLine;    /* By modes */
	MUINT32 u4TotalLinesPerFrame;   /* By modes and frame rate setting */
	MUINT32 u4ActualWidth;  /* By modes */
	MUINT32 u4ActualHeight; /* By modes */
	MUINT32 u4Width;    /* By modes */
	MUINT32 u4Height;   /* By modes */
	MUINT32 u4FrameTimeInus;    /* By modes and frame rate setting */
	MUINT32 u4MinFrameTimeInus; /* By modes */
	MUINT32 u4LineTimeInus; /* By modes */
	MUINT32 u4FinePixCntPerus;  /* Common part */
	MUINT32 u4MinFineTimeInus;  /* By modes */
	MUINT32 u4MaxFineTimeInus;  /* By modes */
	MUINT32 u4XStart;
	MUINT32 u4XEnd;
	MUINT32 u4YStart;
	MUINT32 u4YEnd;
} stImgSensorFormat;

#ifdef CONFIG_COMPAT
/* Redifne structures for compat_ioctl */
typedef struct {
	MSDK_SCENARIO_ID_ENUM ScenarioId[2];
	compat_uptr_t pInfo[2];
	compat_uptr_t pConfig[2];
} COMPAT_ACDK_SENSOR_GETINFO_STRUCT;

typedef struct {
	MUINT32 SensorId;
	compat_uptr_t pInfo;
	compat_uptr_t pSensorResolution;
} COMPAT_IMAGESENSOR_GETINFO_STRUCT;

typedef struct {
	CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera;
	ACDK_SENSOR_FEATURE_ENUM FeatureId;
	compat_uptr_t pFeaturePara;
	compat_uptr_t pFeatureParaLen;
} COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT;

typedef struct {
	CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera;
	MSDK_SCENARIO_ID_ENUM ScenarioId;
	compat_uptr_t pImageWindow;
	compat_uptr_t pSensorConfigData;
} COMPAT_ACDK_SENSOR_CONTROL_STRUCT;

typedef struct {
	compat_uptr_t pResolution[2];
} COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT;

#endif


/*******************************************************************************
*
********************************************************************************/
/*  */
/* adoption to winmo driver files */
/*  */

/* typedef */
#define kal_uint8 u8
#define MSDK_SCENARIO_ID_ENUM                         ACDK_SCENARIO_ID_ENUM
#define MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT            ACDK_SENSOR_EXPOSURE_WINDOW_STRUCT
#define MSDK_SENSOR_CONFIG_STRUCT                     ACDK_SENSOR_CONFIG_STRUCT
#define MSDK_SENSOR_FEATURE_ENUM                      ACDK_SENSOR_FEATURE_ENUM
#define MSDK_SENSOR_REG_INFO_STRUCT                   ACDK_SENSOR_REG_INFO_STRUCT
#define MSDK_SENSOR_GROUP_INFO_STRUCT                 ACDK_SENSOR_GROUP_INFO_STRUCT
#define MSDK_SENSOR_ITEM_INFO_STRUCT                  ACDK_SENSOR_ITEM_INFO_STRUCT
#define MSDK_SENSOR_ENG_INFO_STRUCT                   ACDK_SENSOR_ENG_INFO_STRUCT
#define MSDK_SENSOR_INFO_STRUCT                       ACDK_SENSOR_INFO_STRUCT
#define MSDK_SENSOR_RESOLUTION_INFO_STRUCT            ACDK_SENSOR_RESOLUTION_INFO_STRUCT
#define MSDK_SENSOR_DBG_IMGSENSOR_INFO_STRUCT         ACDK_SENSOR_DBG_IMGSENSOR_STRUCT
#define MSDK_SCENARIO_ID_ENUM                         ACDK_SCENARIO_ID_ENUM
#define MSDK_SCENARIO_ID_CAMERA_PREVIEW               ACDK_SCENARIO_ID_CAMERA_PREVIEW
#define MSDK_SCENARIO_ID_VIDEO_PREVIEW                ACDK_SCENARIO_ID_VIDEO_PREVIEW
#define MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG          ACDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG
#define MSDK_SENSOR_OPERATION_MODE_VIDEO              ACDK_SENSOR_OPERATION_MODE_VIDEO
#define MSDK_SCENARIO_ID_CAMERA_ZSD                   ACDK_SCENARIO_ID_CAMERA_ZSD
#define MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4          ACDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4																		 
#define MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW            ACDK_SCENARIO_ID_CAMERA_3D_PREVIEW
#define MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE            ACDK_SCENARIO_ID_CAMERA_3D_CAPTURE
#define MSDK_SCENARIO_ID_CAMERA_3D_VIDEO              ACDK_SCENARIO_ID_CAMERA_3D_VIDEO
#define MSDK_SENSOR_OPERATION_MODE_VIDEO	      ACDK_SENSOR_OPERATION_MODE_VIDEO
#define MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM    	      ACDK_SCENARIO_ID_CAMERA_CAPTURE_MEM
/*******************************************************************************
*
********************************************************************************/

/*******************************************************************************
*
********************************************************************************/
/* for new simplifed sensor driver */
typedef struct {
	MUINT32(*SensorOpen)(void);
	MUINT32(*SensorGetInfo)(MUINT32 *pScenarioId[2], MSDK_SENSOR_INFO_STRUCT * pSensorInfo[2], MSDK_SENSOR_CONFIG_STRUCT * pSensorConfigData[2]);
	MUINT32(*SensorGetResolution)(MSDK_SENSOR_RESOLUTION_INFO_STRUCT * pSensorResolution[2]);
	MUINT32(*SensorFeatureControl)(CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera, MSDK_SENSOR_FEATURE_ENUM FeatureId, MUINT8 *pFeaturePara, MUINT32 *pFeatureParaLen);
	MUINT32(*SensorControl)(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	MUINT32(*SensorClose)(void);
} MULTI_SENSOR_FUNCTION_STRUCT, *PMULTI_SENSOR_FUNCTION_STRUCT;

typedef struct {
	MUINT32(*SensorOpen)(void);
	MUINT32(*SensorGetInfo)(MUINT32 *pScenarioId[2], MSDK_SENSOR_INFO_STRUCT * pSensorInfo[2], MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData[2]);
	MUINT32(*SensorGetResolution)(MSDK_SENSOR_RESOLUTION_INFO_STRUCT * pSensorResolution[2]);
	MUINT32(*SensorFeatureControl)(CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera, MSDK_SENSOR_FEATURE_ENUM FeatureId, MUINT8 *pFeaturePara, MUINT32 *pFeatureParaLen);
	MUINT32(*SensorControl)(CAMERA_DUAL_CAMERA_SENSOR_ENUM InvokeCamera, MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	MUINT32(*SensorClose)(void);
} MULTI_SENSOR_FUNCTION_STRUCT2, *PMULTI_SENSOR_FUNCTION_STRUCT2;


typedef struct {
	MUINT32(*SensorOpen)(void);
	MUINT32(*SensorGetInfo)(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
				MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	MUINT32(*SensorGetResolution)(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
	MUINT32(*SensorFeatureControl)(MSDK_SENSOR_FEATURE_ENUM FeatureId, MUINT8 *pFeaturePara, MUINT32 *pFeatureParaLen);
	MUINT32(*SensorControl)(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	MUINT32(*SensorClose)(void);
#if 1 /* isp suspend resume patch */
	MSDK_SCENARIO_ID_ENUM ScenarioId;
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT imageWindow;
	MSDK_SENSOR_CONFIG_STRUCT sensorConfigData;
#endif
} SENSOR_FUNCTION_STRUCT, *PSENSOR_FUNCTION_STRUCT;

typedef struct {
	MUINT32 SensorId;
	MUINT8 drvname[32];
	MUINT32(*SensorInit)(PSENSOR_FUNCTION_STRUCT *pfFunc);
} ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT, *PACDK_KD_SENSOR_INIT_FUNCTION_STRUCT;

#define KDIMGSENSOR_DUAL_SHIFT 16
#define KDIMGSENSOR_DUAL_MASK_MSB 0xFFFF0000
#define KDIMGSENSOR_DUAL_MASK_LSB 0x0000FFFF

#define KDIMGSENSOR_NOSENSOR    "non_sensor"

#define KDIMGSENSOR_MAX_INVOKE_DRIVERS  (2)
#define KDIMGSENSOR_INVOKE_DRIVER_0     (0)
#define KDIMGSENSOR_INVOKE_DRIVER_1     (1)

/* For sensor synchronize the exposure time / sensor gain and isp gain. */
typedef struct {
	MUINT16 u2ISPNewRGain;
	MUINT16 u2ISPNewGrGain;
	MUINT16 u2ISPNewGbGain;
	MUINT16 u2ISPNewBGain;
	MUINT16 u2SensorNewExpTime;
	MUINT16 u2SensorNewGain;
	MUINT8 uSensorExpDelayFrame;
	MUINT8 uSensorGainDelayFrame;
	MUINT8 uISPGainDelayFrame;
	MUINT8 uDummy;
} ACDK_KD_SENSOR_SYNC_STRUCT, *PACDK_KD_SENSOR_SYNC_STRUCT;


typedef struct {
	MUINT16 AeRefLV05Shutter;   /* Sensor AE Shutter under Lv05 */
	MUINT16 AeRefLV13Shutter;   /* Sensor AE Shutter under Lv13 */
	MUINT16 AeRefLV05Gain;  /* Sensor AE Gain under Lv05 */
	MUINT16 AeRefLV13Gain;  /* Sensor AE Gain under Lv13 */
} SENSOR_AE_REF_STRUCT, *PSENSOR_AE_REF_STRUCT;


typedef struct {
	MUINT16 AwbRefD65Rgain; /* Sensor AWB R Gain under D65 */
	MUINT16 AwbRefD65Bgain; /* Sensor AWB B Gain under D65 */
	MUINT16 AwbRefCWFRgain; /* Sensor AWB R Gain under CWF */
	MUINT16 AwbRefCWFBgain; /* Sensor AWB B Gain under CWF */
} SENSOR_AWB_GAIN_REF_STRUCT, *PSENSOR_AWB_GAIN_REF_STRUCT;


typedef struct {
	SENSOR_AE_REF_STRUCT SensorAERef;   /* AE Ref information for ASD usage */
	SENSOR_AWB_GAIN_REF_STRUCT SensorAwbGainRef;    /* AWB Gain Ref information for ASD usage */
	MUINT32 SensorLV05LV13EVRef;    /* EV calculate  for ASD usage */
} SENSOR_AE_AWB_REF_STRUCT, *PSENSOR_AE_AWB_REF_STRUCT;



typedef struct {
	MUINT16 AeCurShutter;   /* Current Sensor AE Shutter */
	MUINT16 AeCurGain;  /* Current Sensor AE Gain */
} SENSOR_AE_CUR_STRUCT, *PSENSOR_AE_CUR_STRUCT;


typedef struct {
	MUINT16 AwbCurRgain;    /* Current Sensor AWB R Gain */
	MUINT16 AwbCurBgain;    /* Current Sensor AWB R Gain */
} SENSOR_AWB_GAIN_CUR_STRUCT, *PSENSOR_AWB_GAIN_CUR_STRUCT;


typedef struct {
	SENSOR_AE_CUR_STRUCT SensorAECur;   /* AE Current information for ASD usage */
	SENSOR_AWB_GAIN_CUR_STRUCT SensorAwbGainCur;    /* AWB Gain Current information for ASD usage */
} SENSOR_AE_AWB_CUR_STRUCT, *PSENSOR_AE_AWB_CUR_STRUCT;


typedef struct {
	MUINT32 FNumber;
	MUINT32 AEISOSpeed;
	MUINT32 AWBMode;
	MUINT32 CapExposureTime;
	MUINT32 FlashLightTimeus;
	MUINT32 RealISOValue;
} SENSOR_EXIF_INFO_STRUCT, *PSENSOR_EXIF_INFO_STRUCT;

typedef struct {
	MUINT32 InitDelay;
	MUINT32 EffectDelay;
	MUINT32 AwbDelay;
	MUINT32 AFSwitchDelayFrame;
	MUINT32 EvDelay;
	MUINT32 SatDelay;
	MUINT32 BrightDelay;
	MUINT32 ContrastDelay;
} SENSOR_DELAY_INFO_STRUCT, *PSENSOR_DELAY_INFO_STRUCT;

typedef struct {
	MUINT32 u4Fno;
	MUINT32 Exposuretime;
	MUINT32 Gain;
	MUINT32 GAIN_BASE;
} SENSOR_FLASHLIGHT_AE_INFO_STRUCT, *PSENSOR_FLASHLIGHT_AE_INFO_STRUCT;

/* multisensor driver */

typedef struct {
	MUINT32 drvIndex[KDIMGSENSOR_MAX_INVOKE_DRIVERS];   /* max 2 driver sumultaneously */
} SENSOR_DRIVER_INDEX_STRUCT, *PSENSOR_DRIVER_INDEX_STRUCT;

/* hardcode by GPIO module, should be sync with.(cust_gpio_usage.h) */
#define GPIO_CAMERA_INVALID 0xFF
/*  */
typedef enum {
	IMGSENSOR_SOCKET_POS_NONE = 0xFFFFFFFF,
	IMGSENSOR_SOCKET_POS_RIGHT = 0x1,
	IMGSENSOR_SOCKET_POS_LEFT = 0x2,
} IMGSENSOR_SOCKET_POSITION_ENUM;
/*  */
typedef enum {
	IMGSENSOR_SET_I2C_ID_STATE = 0x00,
	IMGSENSOR_SET_I2C_ID_FORCE
} IMGSENSOR_SET_I2C_ID_ENUM;

#define GPIO_CAMERA_INVALID 0xFF


typedef enum {
	SENSOR_3A_AE_LOCK = 0,
	SENSOR_3A_AE_UNLOCK,
	SENSOR_3A_AWB_LOCK,
	SENSOR_3A_AWB_UNLOCK,
	SENSOR_3A_AE_ON,
	SENSOR_3A_AE_OFF,
	SENSOR_3A_AWB_ON,
	SENSOR_3A_AWB_OFF,
	SENSOR_3A_AF_INFINITY,
	/* SENSOR_3A_AF_LOCK, */
	/* SENSOR_3A_AF_UNLOCK, */
} ACDK_SENSOR_3A_LOCK_ENUM;


typedef struct {
	MUINT32 tgtWidth;
	MUINT32 tgtHeight;
	MUINT32 quality;

} ACDK_SENSOR_JPEG_OUTPUT_PARA;


/* for JPEG Sensor Info */
typedef struct {
	MUINT32 u4FileSize; /* For input sensor width */
	MUINT32 u4SrcW;     /* For input sensor width */
	MUINT32 u4SrcH;     /* For input sensor height */
} ACDK_SENSOR_JPEG_INFO;


#endif              /* _KD_IMGSENSOR_DATA_H */

