/*
 * File: isp_camera_cmd.h
 * Description: The structure and API definition ISP camera command
 *  It is a header file that define structure and API for ISP camera command
 * (C)Copyright Altek Digital Inc. 2013
 *
 * History
 *   2013/09/18; Aaron Chuang; Initial version
 *   2013/12/05; Bruce Chung; 2nd version
 */


#ifndef _ISPCAMERA_CMD_H_
#define _ISPCAMERA_CMD_H_

/*
 *@addtogroup ISPCameraCmd
 *@{
 */

/******Include File******/


#include "mtype.h"


/******Public Constant Definition******/

#define T_MEMSIZE (936*1024)
#define T_SPI_CMD_LENGTH 64
#define USPI_TIMEOUT_GETDATA 200 /*ms*/
#define ISPCMD_DUMMYBYTES  4
#define FWVER_INFOSIZE_MAX 34
#define AEZONECFG_MAXGROUP 5
#define AFZONECFG_MAXGROUP 5
#define ISPCMD_LENFLDBYTES 2
#define ISPCMD_OPCODEBYTES 2
#define ReportRegCount 27
#define ISPCMD_CKSUMBYTES  2

/*length field size = 2, opcode field size = 2, dummy bytes = 4*/
#define ISPCMD_HDRSIZE (ISPCMD_LENFLDBYTES+ISPCMD_OPCODEBYTES)

/*length field size = 2, opcode field size = 2, dummy bytes = 4*/
#define ISPCMD_HDRSIZEWDUMMY (ISPCMD_LENFLDBYTES+\
				ISPCMD_OPCODEBYTES+\
				ISPCMD_DUMMYBYTES)

#define ISPCMD_FILENAME_SIZE 15
#define FD_SUPPORT_NUM 5
#define ISPCMD_EXEBIN_ADDRBYTES 4
#define ISPCMD_EXEBIN_TOTALSIZEBYTES 4
#define ISPCMD_EXEBIN_BLOCKSIZEBYTES 4
#define ISPCMD_EXEBIN_CKSUMBYTES 4
#define ISPCMD_EXEBIN_INFOBYTES (ISPCMD_EXEBIN_ADDRBYTES+\
				ISPCMD_EXEBIN_TOTALSIZEBYTES+\
				ISPCMD_EXEBIN_BLOCKSIZEBYTES+\
				ISPCMD_EXEBIN_CKSUMBYTES)

/* Definition for Error code array number*/
#define MAX_RECERRORCODE_NUM 10

/*log buffer size*/
#define	LEVEL_LOG_BUFFER_SIZE (1024*4)



/*D2 calibration profile*/
#define ISPCMD_CAMERA_GET_SYSTEMINFORMATION 0x3001
#define ISPCMD_CAMERA_SET_BASICPARAMETERS 0x3002
#define ISPCMD_CAMERA_GET_BASICPARAMETERS 0x3003

#define ISPCMD_CAMERA_SET_SENSORMODE 0x300A
#define ISPCMD_CAMERA_GET_SENSORMODE 0x300B
#define ISPCMD_CAMERA_SET_OUTPUTFORMAT 0x300D
#define ISPCMD_CAMERA_PREVIEWSTREAMONOFF 0x3010

/* D2 Bulk Data*/
#define ISPCMD_BULK_WRITE_BASICCODE 0x2002
#define ISPCMD_BULK_WRITE_ADVANCEDCODE 0x2004
#define ISPCMD_BULK_WRITE_BOOTCODE 0x2008
#define ISPCMD_BULK_READ_MEMORY 0x2101
#define ISPCMD_BULK_READ_COMLOG 0x2102
#define ISPCMD_BULK_WRITE_CALIBRATION_DATA 0x210B

/*D2 basic setting*/
#define ISPCMD_BASIC_SET_DEPTH_3A_INFO 0x10B9
#define ISPCMD_BASIC_SET_DEPTH_INPUT_WOL 0x10BB

/*D2 system cmd*/
#define ISPCMD_SYSTEM_CHANGEMODE 0x0010
#define ISPCMD_SYSTEM_GET_STATUSOFMODECHANGE 0x0011
#define ISPCMD_SYSTEM_GET_STATUSOFLASTEXECUTEDCOMMAND 0x0015
#define ISPCMD_SYSTEM_GET_ERRORCODE 0x0016
#define ISPCMD_SYSTEM_SET_ISPREGISTER 0x0100
#define ISPCMD_SYSTEM_GET_ISPREGISTER 0x0101
#define ISPCMD_SYSTEM_SET_COMLOGLEVEL 0x0109
#define ISPCMD_SYSTEM_GET_CHIPTESTREPORT 0x010A
#define ISPCMD_SYSTEM_GET_IRQSTATUS 0x0113
#define ISPCMD_SYSTEM_GET_POLLINGCOMMANDSTATUS 0x0114

/*operarion code*/
#define ISPCMD_MINIISPOPEN 0x4000

/* Command error code*/
#define T_ERROR_NO_ERROR 0
#define T_ERROR_FAILURE	0x50
#define T_ERROR_COMMAND_NOT_SUPPORTED 0x51
#define T_ERROR_DATA_INVALID 0x52
#define T_ERROR_DATA_OUT_OF_RANGE 0x53
#define T_ERROR_SYSTEM_ALREADY_IN_STATE	0x54
#define T_ERROR_SYSTEM_IN_INVALIDE_STATE 0x55

/*constants for Sensors' control to switch on*/
#define T_REAR1_SENSOR_ON 0
#define T_REAR2_SENSOR_ON 1
#define T_REAR1_REAR2_SENSOR_ON 2
#define T_FRONT_SENSOR_ON 3
#define T_FRONT_REAR1_SENSOR_ON 4
#define T_FRONT_REAR2_SENSOR_ON 5

/*constants for Sensor mode for Sensor mode control*/
#define T_SENSOR_BINMODE 0
#define T_SENSOR_FRMODE 1
#define T_SENSOR_FRBINMODE 2
#define T_SENSOR_HDMODE 3
#define T_SENSOR_FHDMODE 4
#define T_SENSOR_TESTMODE 99

/*constants for Preview mode for Change mode control*/
#define T_PREVIEW_TEST 0
#define T_PREVIEW_STILL 1
#define T_PREVIEW_RSVD0 2
#define T_PREVIEW_RSVD1 3
#define T_PREVIEW_BYPASS 4
#define T_PREVIEW_PWRDWN 5

/*constants for anti-flicker mode*/
#define T_FREQ_AUTO 0
#define T_FREQ_OFF 1
#define T_FREQ_50HZ 2
#define T_FREQ_60HZ 3

/*constants for auto-exposure mode*/
#define T_S1EXP_DISABLE 0
#define T_S1EXP_AUTO 1

/*constants for auto-exposure mode*/
#define T_S1FOCUS_DISABLE 0
#define T_S1FOCUS_AUTO 1
#define T_EXP_AUTO 0
#define T_EXP_DISABLE 1
/*constants for exposure compensation*/
#define T_COMPEXPIDX_MIN 0
#define T_COMPEXPIDX_MAX 12
#define T_COMPEXPIDX_DEFAULT 6
#define T_COMPEXPVALUE_MIN (-2) /* -2EV*/
#define T_COMPEXPVALUE_MAX (2)  /*+2EV*/

/*constants for white balance*/
#define T_WB_AUTO 0
#define T_WB_INCANDESCENT 1
#define T_WB_FLUORESCENT 2
#define T_WB_WARM_FLUORESCENT 3
#define T_WB_DAYLIGHT 4
#define T_WB_CLOUDY_DAYLIGHT 5
#define T_WB_TWILIGHT 6
#define T_WB_SHADE 7

/*constants for focus mode*/
#define T_AF_FIXED 0
#define T_AF_AUTO 1
#define T_AF_INFINITY 2
#define T_AF_MACRO 3
#define T_AF_CONTINUEVIDEO 4
#define T_AF_CONTINUEPICTURE 5
#define T_AF_EDOF 6

/*constants for ISO level*/
#define T_ISO_AUTO 0
#define T_ISO_100 1
#define T_ISO_200 2
#define T_ISO_400 3
#define T_ISO_800 4
#define T_ISO_1600 5
#define T_ISO_3200 6
#define T_ISO_6400 7
#define T_ISO_12800 8

/*constants for Scene mode*/
#define T_SCENE_AUTO 0
#define T_SCENE_ACTION 1
#define T_SCENE_PORTRAIT 2
#define T_SCENE_LANDSCAPE 3
#define T_SCENE_NIGHT 4
#define T_SCENE_NIGHTPORTRAIT 5
#define T_SCENE_THEATER 6
#define T_SCENE_BEACH 7
#define T_SCENE_SNOW 8
#define T_SCENE_SUNSET 9
#define T_SCENE_STEADYPHOTO 10
#define T_SCENE_FIREWORKS 11
#define T_SCENE_SPORTS 12
#define T_SCENE_PARTY 13
#define T_SCENE_CANDLELIGHT 14

/*constants for Take mode*/
#define T_TAKEPIC_SINGLE 0
#define T_TAKEPIC_CONTINUE 1
#define T_TAKEPIC_STOP 2

/*constants for AE metering mode*/
#define T_AE_AUTO 0
#define T_AE_AVERAGE 1
#define T_AE_CENTRWEIGHT 2
#define T_AE_SPOT 3

/* constants for memory dump mode*/
#define T_MEMDUMP_CPURUN 0
#define T_MEMDUMP_CPUHALT 1

/* Constants for Chip test Results*/
#define T_CHIPTEST_RESULT_OK	0x01
#define T_CHIPTEST_RESULT_NG	0xFF

/******Public Type Declaration******/

/**
 *@typedef RECORDER_SENSORTYPE
 *@brief define Sensor Type
 */
enum isp_cmd_sensor_type {
	REAR_1,
	REAR_2,
	FRONT_1,
	SENSOR_TYPEMAX
};

/**
 *@struct RECORDER_SENSORINFO
 *@brief Sensor info definition
 */
#pragma pack(1)
struct isp_cmd_sensor_info {
	bool on; /* On/off flag*/
	u8 sensor_mode; /* Sensor mode*/
};

/**
 *@struct RECORDER_TXINFO
 *@brief MIPI Transmit info
 */
#pragma pack(1)
struct isp_cmd_tx_info {
	bool on; /* On/off flag*/
};

/* ISP operation mode*/
enum ispctrl_operation_mode {
	ISPCTRL_TEST_MODE,
	ISPCTRL_STILLLV_MODE,
	ISPCTRL_VIDEOLV_MODE,
	ISPCTRL_CONCURRENT_MODE,
	ISPCTRL_BYPASS_MODE,
	ISPCTRL_POWERDOWN_MODE
};

/*mode id*/
/*define for ISP decide mode*/
enum mini_isp_mode {
	MINI_ISP_MODE_NORMAL = 0x0000,
	MINI_ISP_MODE_E2A = 0x0001,
	MINI_ISP_MODE_A2E = 0x0002,
	MINI_ISP_MODE_GET_CHIP_ID = 0x0004,
	MINI_ISP_MODE_BYPASS = 0x1000,
	MINI_ISP_MODE_QUARTER_BYPASS = 0x1001,
};

/**
 *@struct isp_cmd_system_info
 *@brief ISP master cmd for system info definition
 */
#pragma pack(1)
struct isp_cmd_system_info {
	u8 fw_version_info[FWVER_INFOSIZE_MAX]; /*Firmware version info*/
	u16 isp_chip_id; /* ISP Chip ID*/
	u16 cis1_sensor_id; /*CIS-1 sensor ID*/
	u16 cis2_sensor_id; /*CIS-2 sensor ID*/
	u8 current_isp_mode; /* current ISP mode*/
};



/**
 *@struct isp_cmd_depth_3a_info
 *@brief depth 3A information
 */
#pragma pack(1)
struct isp_cmd_depth_3a_info {
	u32 main_cam_exp_time;
	u16 main_cam_exp_gain;
	u16 main_cam_amb_r_gain;
	u16 main_cam_amb_g_gain;
	u16 main_cam_amb_b_gain;
	u16 main_cam_iso;
	u16 main_cam_bv;
	u16 main_cam_vcm_position;
	u8  main_cam_vcm_status;
	u32 sub_cam_exp_time;
	u16 sub_cam_exp_gain;
	u16 sub_cam_amb_r_gain;
	u16 sub_cam_amb_g_gain;
	u16 sub_cam_amb_b_gain;
	u16 sub_cam_iso;
	u16 sub_cam_bv;
	u16 sub_cam_vcm_position;
	u8  sub_cam_vcm_status;
};

/**
 *@struct isp_cmd_basic_para
 *@brief ISP master cmd for basic parameters access
 */
#pragma pack(1)
struct isp_cmd_basic_para {
	u8  anti_flicker_mode;
	u8  auto_exposure[SENSOR_TYPEMAX];
	s8  ev_comp[SENSOR_TYPEMAX];
	u8  ae_mode[SENSOR_TYPEMAX];
	u8  white_balance_mode[SENSOR_TYPEMAX];
	u8  focus_mode[SENSOR_TYPEMAX];
	u8  iso_level[SENSOR_TYPEMAX];
	u8  dz_level;
	u8  flash_light_mode;
	u8  sharpness;
	u8  contrast;
	u8  saturation;
	u8  digital_light_mode;
	u8  scene_mode[SENSOR_TYPEMAX];
	u8  picture_format;
	u16 picture_width;
	u16 picture_height;
	u8  lv_format;
	u8  lv_fps_min;
	u8  lv_fps_max;
	u16 lv_width;
	u16 lv_height;
	u8  video_stabilization_mode;
	u8  three_dnr_mode;
	u8  wdr_mode;
	u8  hdr_mode;
	u8  super_resolution_mode;
	u8  zero_shutter_lag_mode;
	u8  histogram_mode;
	u8  shutter_key_mode;
};

/**
 *@struct isp_cmd_ae_zone_info
 *@brief ISP Command information for AE control
 */
#pragma pack(1)
struct isp_cmd_ae_zone_info {
	u16 ae_zone_start_x;
	u16 ae_zone_start_y;
	u16 ae_zone_width;
	u16 ae_zone_height;
	u16 ae_weight_factor;
};

/**
 *@struct isp_cmd_af_zone_info
 *@brief ISP Command information for AF control
 */
#pragma pack(1)
struct isp_cmd_af_zone_info {
	u16 af_zone_start_x;
	u16 af_zone_start_y;
	u16 af_zone_width;
	u16 af_zone_height;
	u16 af_weight_factor;
};


#pragma pack(1)
struct memmory_dump_hdr_info {
	u32 start_addr;
	u32 total_size;
	u32 block_size;
	u32 dump_mode;
};


/**
 *@struct common_log_hdr_info
 *@brief Bulk data for memory dump header
 */
#pragma pack(1)
struct common_log_hdr_info {
	u32 total_size;
	u32 block_size;
};




/******Public Function Prototype******/


/******End of File******/

/**
 *@}
 */

#endif /* _ISPCAMERA_CMD_H_*/
