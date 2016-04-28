// PIControllerErrors.h
//
// This file defines symbols for each error code used in C/C++ programs.
//
// This file is automagically generated from the central error code list.
// DO NOT ADD ERROR CODES IN THIS FILE! Use the error list and the generation tool instead!

#ifndef __PI_CONTROLLER_ERROS_H__
#define __PI_CONTROLLER_ERROS_H__

//////////////////////////////////////////////////
//
// Dll Errors - DLL errors occured in GCS DLL
//
#define PI_UNKNOWN_AXIS_IDENTIFIER  -1001  /**< \ingroup err
		Unknown axis identifier */
#define PI_NR_NAV_OUT_OF_RANGE  -1002  /**< \ingroup err
		Number for NAV out of range--must be in [1,10000] */
#define PI_INVALID_SGA  -1003  /**< \ingroup err
		Invalid value for SGA--must be one of {1, 10, 100, 1000} */
#define PI_UNEXPECTED_RESPONSE  -1004  /**< \ingroup err
		Controller sent unexpected response */
#define PI_NO_MANUAL_PAD  -1005  /**< \ingroup err
		No manual control pad installed, calls to SMA and related commands are not allowed */
#define PI_INVALID_MANUAL_PAD_KNOB  -1006  /**< \ingroup err
		Invalid number for manual control pad knob */
#define PI_INVALID_MANUAL_PAD_AXIS  -1007  /**< \ingroup err
		Axis not currently controlled by a manual control pad */
#define PI_CONTROLLER_BUSY  -1008  /**< \ingroup err
		Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm) */
#define PI_THREAD_ERROR  -1009  /**< \ingroup err
		Internal error--could not start thread */
#define PI_IN_MACRO_MODE  -1010  /**< \ingroup err
		Controller is (already) in macro mode--command not valid in macro mode */
#define PI_NOT_IN_MACRO_MODE  -1011  /**< \ingroup err
		Controller not in macro mode--command not valid unless macro mode active */
#define PI_MACRO_FILE_ERROR  -1012  /**< \ingroup err
		Could not open file to write or read macro */
#define PI_NO_MACRO_OR_EMPTY  -1013  /**< \ingroup err
		No macro with given name on controller, or macro is empty */
#define PI_MACRO_EDITOR_ERROR  -1014  /**< \ingroup err
		Internal error in macro editor */
#define PI_INVALID_ARGUMENT  -1015  /**< \ingroup err
		One or more arguments given to function is invalid (empty string, index out of range, ...) */
#define PI_AXIS_ALREADY_EXISTS  -1016  /**< \ingroup err
		Axis identifier is already in use by a connected stage */
#define PI_INVALID_AXIS_IDENTIFIER  -1017  /**< \ingroup err
		Invalid axis identifier */
#define PI_COM_ARRAY_ERROR  -1018  /**< \ingroup err
		Could not access array data in COM server */
#define PI_COM_ARRAY_RANGE_ERROR  -1019  /**< \ingroup err
		Range of array does not fit the number of parameters */
#define PI_INVALID_SPA_CMD_ID  -1020  /**< \ingroup err
		Invalid parameter ID given to SPA or SPA? */
#define PI_NR_AVG_OUT_OF_RANGE  -1021  /**< \ingroup err
		Number for AVG out of range--must be >0 */
#define PI_WAV_SAMPLES_OUT_OF_RANGE  -1022  /**< \ingroup err
		Incorrect number of samples given to WAV */
#define PI_WAV_FAILED  -1023  /**< \ingroup err
		Generation of wave failed */
#define PI_MOTION_ERROR  -1024  /**< \ingroup err
		Motion error while axis in motion, call CLR to resume operation */
#define PI_RUNNING_MACRO  -1025  /**< \ingroup err
		Controller is (already) running a macro */
#define PI_PZT_CONFIG_FAILED  -1026  /**< \ingroup err
		Configuration of PZT stage or amplifier failed */
#define PI_PZT_CONFIG_INVALID_PARAMS  -1027  /**< \ingroup err
		Current settings are not valid for desired configuration */
#define PI_UNKNOWN_CHANNEL_IDENTIFIER  -1028  /**< \ingroup err
		Unknown channel identifier */
#define PI_WAVE_PARAM_FILE_ERROR  -1029  /**< \ingroup err
		Error while reading/writing wave generator parameter file */
#define PI_UNKNOWN_WAVE_SET  -1030  /**< \ingroup err
		Could not find description of wave form. Maybe WG.INI is missing? */
#define PI_WAVE_EDITOR_FUNC_NOT_LOADED  -1031  /**< \ingroup err
		The WGWaveEditor DLL function was not found at startup */
#define PI_USER_CANCELLED  -1032  /**< \ingroup err
		The user cancelled a dialog */
#define PI_C844_ERROR  -1033  /**< \ingroup err
		Error from C-844 Controller */
#define PI_DLL_NOT_LOADED  -1034  /**< \ingroup err
		DLL necessary to call function not loaded, or function not found in DLL */
#define PI_PARAMETER_FILE_PROTECTED  -1035  /**< \ingroup err
		The open parameter file is protected and cannot be edited */
#define PI_NO_PARAMETER_FILE_OPENED  -1036  /**< \ingroup err
		There is no parameter file open */
#define PI_STAGE_DOES_NOT_EXIST  -1037  /**< \ingroup err
		Selected stage does not exist */
#define PI_PARAMETER_FILE_ALREADY_OPENED  -1038  /**< \ingroup err
		There is already a parameter file open. Close it before opening a new file */
#define PI_PARAMETER_FILE_OPEN_ERROR  -1039  /**< \ingroup err
		Could not open parameter file */
#define PI_INVALID_CONTROLLER_VERSION  -1040  /**< \ingroup err
		The version of the connected controller is invalid */
#define PI_PARAM_SET_ERROR  -1041  /**< \ingroup err
		Parameter could not be set with SPA--parameter not defined for this controller! */
#define PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED  -1042  /**< \ingroup err
		The maximum number of wave definitions has been exceeded */
#define PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED  -1043  /**< \ingroup err
		The maximum number of wave generators has been exceeded */
#define PI_NO_WAVE_FOR_AXIS_DEFINED  -1044  /**< \ingroup err
		No wave defined for specified axis */
#define PI_CANT_STOP_OR_START_WAV  -1045  /**< \ingroup err
		Wave output to axis already stopped/started */
#define PI_REFERENCE_ERROR  -1046  /**< \ingroup err
		Not all axes could be referenced */
#define PI_REQUIRED_WAVE_NOT_FOUND  -1047  /**< \ingroup err
		Could not find parameter set required by frequency relation */
#define PI_INVALID_SPP_CMD_ID  -1048  /**< \ingroup err
		Command ID given to SPP or SPP? is not valid */
#define PI_STAGE_NAME_ISNT_UNIQUE  -1049  /**< \ingroup err
		A stage name given to CST is not unique */
#define PI_FILE_TRANSFER_BEGIN_MISSING  -1050  /**< \ingroup err
		A uuencoded file transfered did not start with \"begin\" followed by the proper filename */
#define PI_FILE_TRANSFER_ERROR_TEMP_FILE  -1051  /**< \ingroup err
		Could not create/read file on host PC */
#define PI_FILE_TRANSFER_CRC_ERROR  -1052  /**< \ingroup err
		Checksum error when transfering a file to/from the controller */
#define PI_COULDNT_FIND_PISTAGES_DAT  -1053  /**< \ingroup err
		The PiStages.dat database could not be found. This file is required to connect a stage with the CST command */
#define PI_NO_WAVE_RUNNING  -1054  /**< \ingroup err
		No wave being output to specified axis */
#define PI_INVALID_PASSWORD  -1055  /**< \ingroup err
		Invalid password */
#define PI_OPM_COM_ERROR  -1056  /**< \ingroup err
		Error during communication with OPM (Optical Power Meter), maybe no OPM connected */
#define PI_WAVE_EDITOR_WRONG_PARAMNUM  -1057  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect number of parameters */
#define PI_WAVE_EDITOR_FREQUENCY_OUT_OF_RANGE  -1058  /**< \ingroup err
		WaveEditor: Frequency out of range */
#define PI_WAVE_EDITOR_WRONG_IP_VALUE  -1059  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect index for integer parameter */
#define PI_WAVE_EDITOR_WRONG_DP_VALUE  -1060  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect index for floating point parameter */
#define PI_WAVE_EDITOR_WRONG_ITEM_VALUE  -1061  /**< \ingroup err
		WaveEditor: Error during wave creation, could not calculate value */
#define PI_WAVE_EDITOR_MISSING_GRAPH_COMPONENT  -1062  /**< \ingroup err
		WaveEditor: Graph display component not installed */
#define PI_EXT_PROFILE_UNALLOWED_CMD  -1063  /**< \ingroup err
		User Profile Mode: Command is not allowed, check for required preparatory commands */
#define PI_EXT_PROFILE_EXPECTING_MOTION_ERROR  -1064  /**< \ingroup err
		User Profile Mode: First target position in User Profile is too far from current position */
#define PI_EXT_PROFILE_ACTIVE  -1065  /**< \ingroup err
		Controller is (already) in User Profile Mode */
#define PI_EXT_PROFILE_INDEX_OUT_OF_RANGE  -1066  /**< \ingroup err
		User Profile Mode: Block or Data Set index out of allowed range */
#define PI_PROFILE_GENERATOR_NO_PROFILE  -1067  /**< \ingroup err
		ProfileGenerator: No profile has been created yet */
#define PI_PROFILE_GENERATOR_OUT_OF_LIMITS  -1068  /**< \ingroup err
		ProfileGenerator: Generated profile exceeds limits of one or both axes */
#define PI_PROFILE_GENERATOR_UNKNOWN_PARAMETER  -1069  /**< \ingroup err
		ProfileGenerator: Unknown parameter ID in Set/Get Parameter command */
#define PI_PROFILE_GENERATOR_PAR_OUT_OF_RANGE  -1070  /**< \ingroup err
		ProfileGenerator: Parameter out of allowed range */
#define PI_EXT_PROFILE_OUT_OF_MEMORY  -1071  /**< \ingroup err
		User Profile Mode: Out of memory */
#define PI_EXT_PROFILE_WRONG_CLUSTER  -1072  /**< \ingroup err
		User Profile Mode: Cluster is not assigned to this axis */
#define PI_EXT_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER  -1073  /**< \ingroup err
		Unknown cluster identifier */
#define PI_INVALID_DEVICE_DRIVER_VERSION  -1074  /**< \ingroup err
		The installed device driver doesn't match the required version. Please see the documentation to determine the required device driver version. */
#define PI_INVALID_LIBRARY_VERSION  -1075  /**< \ingroup err
		The library used doesn't match the required version. Please see the documentation to determine the required library version. */
#define PI_INTERFACE_LOCKED  -1076  /**< \ingroup err
		The interface is currently locked by another function. Please try again later. */
//
//  End of Dll Errors
//////////////////////////////////////////////////

//////////////////////////////////////////////////
//
// Controller Errors - Errors set by the controller or the GCS DLL
//
#define PI_CNTR_NO_ERROR  0  /**< \ingroup err
		No error */
#define PI_CNTR_PARAM_SYNTAX  1  /**< \ingroup err
		Parameter syntax error */
#define PI_CNTR_UNKNOWN_COMMAND  2  /**< \ingroup err
		Unknown command */
#define PI_CNTR_COMMAND_TOO_LONG  3  /**< \ingroup err
		Command length out of limits or command buffer overrun */
#define PI_CNTR_SCAN_ERROR  4  /**< \ingroup err
		Error while scanning */
#define PI_CNTR_MOVE_WITHOUT_REF_OR_NO_SERVO  5  /**< \ingroup err
		Unallowable move attempted on unreferenced axis, or move attempted with servo off */
#define PI_CNTR_INVALID_SGA_PARAM  6  /**< \ingroup err
		Parameter for SGA not valid */
#define PI_CNTR_POS_OUT_OF_LIMITS  7  /**< \ingroup err
		Position out of limits */
#define PI_CNTR_VEL_OUT_OF_LIMITS  8  /**< \ingroup err
		Velocity out of limits */
#define PI_CNTR_SET_PIVOT_NOT_POSSIBLE  9  /**< \ingroup err
		Attempt to set pivot point while U,V and W not all 0 */
#define PI_CNTR_STOP  10  /**< \ingroup err
		Controller was stopped by command */
#define PI_CNTR_SST_OR_SCAN_RANGE  11  /**< \ingroup err
		Parameter for SST or for one of the embedded scan algorithms out of range */
#define PI_CNTR_INVALID_SCAN_AXES  12  /**< \ingroup err
		Invalid axis combination for fast scan */
#define PI_CNTR_INVALID_NAV_PARAM  13  /**< \ingroup err
		Parameter for NAV out of range */
#define PI_CNTR_INVALID_ANALOG_INPUT  14  /**< \ingroup err
		Invalid analog channel */
#define PI_CNTR_INVALID_AXIS_IDENTIFIER  15  /**< \ingroup err
		Invalid axis identifier */
#define PI_CNTR_INVALID_STAGE_NAME  16  /**< \ingroup err
		Unknown stage name */
#define PI_CNTR_PARAM_OUT_OF_RANGE  17  /**< \ingroup err
		Parameter out of range */
#define PI_CNTR_INVALID_MACRO_NAME  18  /**< \ingroup err
		Invalid macro name */
#define PI_CNTR_MACRO_RECORD  19  /**< \ingroup err
		Error while recording macro */
#define PI_CNTR_MACRO_NOT_FOUND  20  /**< \ingroup err
		Macro not found */
#define PI_CNTR_AXIS_HAS_NO_BRAKE  21  /**< \ingroup err
		Axis has no brake */
#define PI_CNTR_DOUBLE_AXIS  22  /**< \ingroup err
		Axis identifier specified more than once */
#define PI_CNTR_ILLEGAL_AXIS  23  /**< \ingroup err
		Illegal axis */
#define PI_CNTR_PARAM_NR  24  /**< \ingroup err
		Incorrect number of parameters */
#define PI_CNTR_INVALID_REAL_NR  25  /**< \ingroup err
		Invalid floating point number */
#define PI_CNTR_MISSING_PARAM  26  /**< \ingroup err
		Parameter missing */
#define PI_CNTR_SOFT_LIMIT_OUT_OF_RANGE  27  /**< \ingroup err
		Soft limit out of range */
#define PI_CNTR_NO_MANUAL_PAD  28  /**< \ingroup err
		No manual pad found */
#define PI_CNTR_NO_JUMP  29  /**< \ingroup err
		No more step-response values */
#define PI_CNTR_INVALID_JUMP  30  /**< \ingroup err
		No step-response values recorded */
#define PI_CNTR_AXIS_HAS_NO_REFERENCE  31  /**< \ingroup err
		Axis has no reference sensor */
#define PI_CNTR_STAGE_HAS_NO_LIM_SWITCH  32  /**< \ingroup err
		Axis has no limit switch */
#define PI_CNTR_NO_RELAY_CARD  33  /**< \ingroup err
		No relay card installed */
#define PI_CNTR_CMD_NOT_ALLOWED_FOR_STAGE  34  /**< \ingroup err
		Command not allowed for selected stage(s) */
#define PI_CNTR_NO_DIGITAL_INPUT  35  /**< \ingroup err
		No digital input installed */
#define PI_CNTR_NO_DIGITAL_OUTPUT  36  /**< \ingroup err
		No digital output configured */
#define PI_CNTR_NO_MCM  37  /**< \ingroup err
		No more MCM responses */
#define PI_CNTR_INVALID_MCM  38  /**< \ingroup err
		No MCM values recorded */
#define PI_CNTR_INVALID_CNTR_NUMBER  39  /**< \ingroup err
		Controller number invalid */
#define PI_CNTR_NO_JOYSTICK_CONNECTED  40  /**< \ingroup err
		No joystick configured */
#define PI_CNTR_INVALID_EGE_AXIS  41  /**< \ingroup err
		Invalid axis for electronic gearing, axis can not be slave */
#define PI_CNTR_SLAVE_POSITION_OUT_OF_RANGE  42  /**< \ingroup err
		Position of slave axis is out of range */
#define PI_CNTR_COMMAND_EGE_SLAVE  43  /**< \ingroup err
		Slave axis cannot be commanded directly when electronic gearing is enabled */
#define PI_CNTR_JOYSTICK_CALIBRATION_FAILED  44  /**< \ingroup err
		Calibration of joystick failed */
#define PI_CNTR_REFERENCING_FAILED  45  /**< \ingroup err
		Referencing failed */
#define PI_CNTR_OPM_MISSING  46  /**< \ingroup err
		OPM (Optical Power Meter) missing */
#define PI_CNTR_OPM_NOT_INITIALIZED  47  /**< \ingroup err
		OPM (Optical Power Meter) not initialized or cannot be initialized */
#define PI_CNTR_OPM_COM_ERROR  48  /**< \ingroup err
		OPM (Optical Power Meter) Communication Error */
#define PI_CNTR_MOVE_TO_LIMIT_SWITCH_FAILED  49  /**< \ingroup err
		Move to limit switch failed */
#define PI_CNTR_REF_WITH_REF_DISABLED  50  /**< \ingroup err
		Attempt to reference axis with referencing disabled */
#define PI_CNTR_AXIS_UNDER_JOYSTICK_CONTROL  51  /**< \ingroup err
		Selected axis is controlled by joystick */
#define PI_CNTR_COMMUNICATION_ERROR  52  /**< \ingroup err
		Controller detected communication error */
#define PI_CNTR_DYNAMIC_MOVE_IN_PROCESS  53  /**< \ingroup err
		MOV! motion still in progress */
#define PI_CNTR_UNKNOWN_PARAMETER  54  /**< \ingroup err
		Unknown parameter */
#define PI_CNTR_NO_REP_RECORDED  55  /**< \ingroup err
		No commands were recorded with REP */
#define PI_CNTR_INVALID_PASSWORD  56  /**< \ingroup err
		Password invalid */
#define PI_CNTR_INVALID_RECORDER_CHAN  57  /**< \ingroup err
		Data Record Table does not exist */
#define PI_CNTR_INVALID_RECORDER_SRC_OPT  58  /**< \ingroup err
		Source does not exist; number too low or too high */
#define PI_CNTR_INVALID_RECORDER_SRC_CHAN  59  /**< \ingroup err
		Source Record Table number too low or too high */
#define PI_CNTR_PARAM_PROTECTION  60  /**< \ingroup err
		Protected Param: current Command Level (CCL) too low */
#define PI_CNTR_AUTOZERO_RUNNING  61  /**< \ingroup err
		Command execution not possible while Autozero is running */
#define PI_CNTR_NO_LINEAR_AXIS  62  /**< \ingroup err
		Autozero requires at least one linear axis */
#define PI_CNTR_INIT_RUNNING  63  /**< \ingroup err
		Initialization still in progress */
#define PI_CNTR_READ_ONLY_PARAMETER  64  /**< \ingroup err
		Parameter is read-only */
#define PI_CNTR_PAM_NOT_FOUND  65  /**< \ingroup err
		Parameter not found in non-volatile memory */
#define PI_CNTR_VOL_OUT_OF_LIMITS  66  /**< \ingroup err
		Voltage out of limits */
#define PI_CNTR_WAVE_TOO_LARGE  67  /**< \ingroup err
		Not enough memory available for requested wave curve */
#define PI_CNTR_NOT_ENOUGH_DDL_MEMORY  68  /**< \ingroup err
		Not enough memory available for DDL table; DDL can not be started */
#define PI_CNTR_DDL_TIME_DELAY_TOO_LARGE  69  /**< \ingroup err
		Time delay larger than DDL table; DDL can not be started */
#define PI_CNTR_DIFFERENT_ARRAY_LENGTH  70  /**< \ingroup err
		The requested arrays have different lengths; query them separately */
#define PI_CNTR_GEN_SINGLE_MODE_RESTART  71  /**< \ingroup err
		Attempt to restart the generator while it is running in single step mode */
#define PI_CNTR_ANALOG_TARGET_ACTIVE  72  /**< \ingroup err
		Motion commands and wave generator activation are not allowed when analog target is active */
#define PI_CNTR_WAVE_GENERATOR_ACTIVE  73  /**< \ingroup err
		Motion commands are not allowed when wave generator output is active; use WGO to disable generator output */
#define PI_CNTR_AUTOZERO_DISABLED  74  /**< \ingroup err
		No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix) */
#define PI_CNTR_NO_WAVE_SELECTED  75  /**< \ingroup err
		Generator started (WGO) without having selected a wave table (WSL). */
#define PI_CNTR_IF_BUFFER_OVERRUN  76  /**< \ingroup err
		Interface buffer did overrun and command couldn't be received correctly */
#define PI_CNTR_NOT_ENOUGH_RECORDED_DATA  77  /**< \ingroup err
		Data Record Table does not hold enough recorded data */
#define PI_CNTR_TABLE_DEACTIVATED  78  /**< \ingroup err
		Data Record Table is not configured for recording */
#define PI_CNTR_OPENLOOP_VALUE_SET_WHEN_SERVO_ON  79  /**< \ingroup err
		Open-loop commands (SVA, SVR) are not allowed when servo is on */
#define PI_LABVIEW_ERROR  100  /**< \ingroup err
		PI LabVIEW driver reports error. See source control for details. */
#define PI_CNTR_NO_AXIS  200  /**< \ingroup err
		No stage connected to axis */
#define PI_CNTR_NO_AXIS_PARAM_FILE  201  /**< \ingroup err
		File with axis parameters not found */
#define PI_CNTR_INVALID_AXIS_PARAM_FILE  202  /**< \ingroup err
		Invalid axis parameter file */
#define PI_CNTR_NO_AXIS_PARAM_BACKUP  203  /**< \ingroup err
		Backup file with axis parameters not found */
#define PI_CNTR_RESERVED_204  204  /**< \ingroup err
		PI internal error code 204 */
#define PI_CNTR_SMO_WITH_SERVO_ON  205  /**< \ingroup err
		SMO with servo on */
#define PI_CNTR_UUDECODE_INCOMPLETE_HEADER  206  /**< \ingroup err
		uudecode: incomplete header */
#define PI_CNTR_UUDECODE_NOTHING_TO_DECODE  207  /**< \ingroup err
		uudecode: nothing to decode */
#define PI_CNTR_UUDECODE_ILLEGAL_FORMAT  208  /**< \ingroup err
		uudecode: illegal UUE format */
#define PI_CNTR_CRC32_ERROR  209  /**< \ingroup err
		CRC32 error */
#define PI_CNTR_ILLEGAL_FILENAME  210  /**< \ingroup err
		Illegal file name (must be 8-0 format) */
#define PI_CNTR_FILE_NOT_FOUND  211  /**< \ingroup err
		File not found on controller */
#define PI_CNTR_FILE_WRITE_ERROR  212  /**< \ingroup err
		Error writing file on controller */
#define PI_CNTR_DTR_HINDERS_VELOCITY_CHANGE  213  /**< \ingroup err
		VEL command not allowed in DTR Command Mode */
#define PI_CNTR_POSITION_UNKNOWN  214  /**< \ingroup err
		Position calculations failed */
#define PI_CNTR_CONN_POSSIBLY_BROKEN  215  /**< \ingroup err
		The connection between controller and stage may be broken */
#define PI_CNTR_ON_LIMIT_SWITCH  216  /**< \ingroup err
		The connected stage has driven into a limit switch, call CLR to resume operation */
#define PI_CNTR_UNEXPECTED_STRUT_STOP  217  /**< \ingroup err
		Strut test command failed because of an unexpected strut stop */
#define PI_CNTR_POSITION_BASED_ON_ESTIMATION  218  /**< \ingroup err
		While MOV! is running position can only be estimated! */
#define PI_CNTR_POSITION_BASED_ON_INTERPOLATION  219  /**< \ingroup err
		Position was calculated during MOV motion */
#define PI_CNTR_SEND_BUFFER_OVERFLOW  301  /**< \ingroup err
		Send buffer overflow */
#define PI_CNTR_VOLTAGE_OUT_OF_LIMITS  302  /**< \ingroup err
		Voltage out of limits */
#define PI_CNTR_VOLTAGE_SET_WHEN_SERVO_ON  303  /**< \ingroup err
		Attempt to set voltage when servo on */
#define PI_CNTR_RECEIVING_BUFFER_OVERFLOW  304  /**< \ingroup err
		Received command is too long */
#define PI_CNTR_EEPROM_ERROR  305  /**< \ingroup err
		Error while reading/writing EEPROM */
#define PI_CNTR_I2C_ERROR  306  /**< \ingroup err
		Error on I2C bus */
#define PI_CNTR_RECEIVING_TIMEOUT  307  /**< \ingroup err
		Timeout while receiving command */
#define PI_CNTR_TIMEOUT  308  /**< \ingroup err
		A lengthy operation has not finished in the expected time */
#define PI_CNTR_MACRO_OUT_OF_SPACE  309  /**< \ingroup err
		Insufficient space to store macro */
#define PI_CNTR_EUI_OLDVERSION_CFGDATA  310  /**< \ingroup err
		Configuration data has old version number */
#define PI_CNTR_EUI_INVALID_CFGDATA  311  /**< \ingroup err
		Invalid configuration data */
#define PI_CNTR_HARDWARE_ERROR  333  /**< \ingroup err
		Internal hardware error */
#define PI_CNTR_UNKNOWN_ERROR  555  /**< \ingroup err
		BasMac: unknown controller error */
#define PI_CNTR_NOT_ENOUGH_MEMORY  601  /**< \ingroup err
		Not enough memory */
#define PI_CNTR_HW_VOLTAGE_ERROR  602  /**< \ingroup err
		Hardware voltage error */
#define PI_CNTR_HW_TEMPERATURE_ERROR  603  /**< \ingroup err
		Hardware temperature out of range */
#define PI_CNTR_TOO_MANY_NESTED_MACROS  1000  /**< \ingroup err
		Too many nested macros */
#define PI_CNTR_MACRO_ALREADY_DEFINED  1001  /**< \ingroup err
		Macro already defined */
#define PI_CNTR_NO_MACRO_RECORDING  1002  /**< \ingroup err
		Macro recording not activated */
#define PI_CNTR_INVALID_MAC_PARAM  1003  /**< \ingroup err
		Invalid parameter for MAC */
#define PI_CNTR_MACRO_DELETE_ERROR  1004  /**< \ingroup err
		Deleting macro failed */
#define PI_CNTR_ALREADY_HAS_SERIAL_NUMBER  2000  /**< \ingroup err
		Controller already has a serial number */
#define PI_CNTR_SECTOR_ERASE_FAILED  4000  /**< \ingroup err
		Sector erase failed */
#define PI_CNTR_FLASH_PROGRAM_FAILED  4001  /**< \ingroup err
		Flash program failed */
#define PI_CNTR_FLASH_READ_FAILED  4002  /**< \ingroup err
		Flash read failed */
#define PI_CNTR_HW_MATCHCODE_ERROR  4003  /**< \ingroup err
		HW match code missing/invalid */
#define PI_CNTR_FW_MATCHCODE_ERROR  4004  /**< \ingroup err
		FW match code missing/invalid */
#define PI_CNTR_HW_VERSION_ERROR  4005  /**< \ingroup err
		HW version missing/invalid */
#define PI_CNTR_FW_VERSION_ERROR  4006  /**< \ingroup err
		FW version missing/invalid */
#define PI_CNTR_FW_UPDATE_ERROR  4007  /**< \ingroup err
		FW update failed */
//
//  End of Controller Errors
//////////////////////////////////////////////////

//////////////////////////////////////////////////
//
// Interface Errors - Interface errors occuring while communicating with the controller
//
#define COM_NO_ERROR  0  /**< \ingroup err
		No error occurred during function call */
#define COM_ERROR  -1  /**< \ingroup err
		Error during com operation (could not be specified) */
#define SEND_ERROR  -2  /**< \ingroup err
		Error while sending data */
#define REC_ERROR  -3  /**< \ingroup err
		Error while receiving data */
#define NOT_CONNECTED_ERROR  -4  /**< \ingroup err
		Not connected (no port with given ID open) */
#define COM_BUFFER_OVERFLOW  -5  /**< \ingroup err
		Buffer overflow */
#define CONNECTION_FAILED  -6  /**< \ingroup err
		Error while opening port */
#define COM_TIMEOUT  -7  /**< \ingroup err
		Timeout error */
#define COM_MULTILINE_RESPONSE  -8  /**< \ingroup err
		There are more lines waiting in buffer */
#define COM_INVALID_ID  -9  /**< \ingroup err
		There is no interface or DLL handle with the given ID */
#define COM_NOTIFY_EVENT_ERROR  -10  /**< \ingroup err
		Event/message for notification could not be opened */
#define COM_NOT_IMPLEMENTED  -11  /**< \ingroup err
		Function not supported by this interface type */
#define COM_ECHO_ERROR  -12  /**< \ingroup err
		Error while sending \"echoed\" data */
#define COM_GPIB_EDVR  -13  /**< \ingroup err
		IEEE488: System error */
#define COM_GPIB_ECIC  -14  /**< \ingroup err
		IEEE488: Function requires GPIB board to be CIC */
#define COM_GPIB_ENOL  -15  /**< \ingroup err
		IEEE488: Write function detected no listeners */
#define COM_GPIB_EADR  -16  /**< \ingroup err
		IEEE488: Interface board not addressed correctly */
#define COM_GPIB_EARG  -17  /**< \ingroup err
		IEEE488: Invalid argument to function call */
#define COM_GPIB_ESAC  -18  /**< \ingroup err
		IEEE488: Function requires GPIB board to be SAC */
#define COM_GPIB_EABO  -19  /**< \ingroup err
		IEEE488: I/O operation aborted */
#define COM_GPIB_ENEB  -20  /**< \ingroup err
		IEEE488: Interface board not found */
#define COM_GPIB_EDMA  -21  /**< \ingroup err
		IEEE488: Error performing DMA */
#define COM_GPIB_EOIP  -22  /**< \ingroup err
		IEEE488: I/O operation started before previous operation completed */
#define COM_GPIB_ECAP  -23  /**< \ingroup err
		IEEE488: No capability for intended operation */
#define COM_GPIB_EFSO  -24  /**< \ingroup err
		IEEE488: File system operation error */
#define COM_GPIB_EBUS  -25  /**< \ingroup err
		IEEE488: Command error during device call */
#define COM_GPIB_ESTB  -26  /**< \ingroup err
		IEEE488: Serial poll-status byte lost */
#define COM_GPIB_ESRQ  -27  /**< \ingroup err
		IEEE488: SRQ remains asserted */
#define COM_GPIB_ETAB  -28  /**< \ingroup err
		IEEE488: Return buffer full */
#define COM_GPIB_ELCK  -29  /**< \ingroup err
		IEEE488: Address or board locked */
#define COM_RS_INVALID_DATA_BITS  -30  /**< \ingroup err
		RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits with 1.5 stop bits */
#define COM_ERROR_RS_SETTINGS  -31  /**< \ingroup err
		RS-232: Error configuring the COM port */
#define COM_INTERNAL_RESOURCES_ERROR  -32  /**< \ingroup err
		Error dealing with internal system resources (events, threads, ...) */
#define COM_DLL_FUNC_ERROR  -33  /**< \ingroup err
		A DLL or one of the required functions could not be loaded */
#define COM_FTDIUSB_INVALID_HANDLE  -34  /**< \ingroup err
		FTDIUSB: invalid handle */
#define COM_FTDIUSB_DEVICE_NOT_FOUND  -35  /**< \ingroup err
		FTDIUSB: device not found */
#define COM_FTDIUSB_DEVICE_NOT_OPENED  -36  /**< \ingroup err
		FTDIUSB: device not opened */
#define COM_FTDIUSB_IO_ERROR  -37  /**< \ingroup err
		FTDIUSB: IO error */
#define COM_FTDIUSB_INSUFFICIENT_RESOURCES  -38  /**< \ingroup err
		FTDIUSB: insufficient resources */
#define COM_FTDIUSB_INVALID_PARAMETER  -39  /**< \ingroup err
		FTDIUSB: invalid parameter */
#define COM_FTDIUSB_INVALID_BAUD_RATE  -40  /**< \ingroup err
		FTDIUSB: invalid baud rate */
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_ERASE  -41  /**< \ingroup err
		FTDIUSB: device not opened for erase */
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_WRITE  -42  /**< \ingroup err
		FTDIUSB: device not opened for write */
#define COM_FTDIUSB_FAILED_TO_WRITE_DEVICE  -43  /**< \ingroup err
		FTDIUSB: failed to write device */
#define COM_FTDIUSB_EEPROM_READ_FAILED  -44  /**< \ingroup err
		FTDIUSB: EEPROM read failed */
#define COM_FTDIUSB_EEPROM_WRITE_FAILED  -45  /**< \ingroup err
		FTDIUSB: EEPROM write failed */
#define COM_FTDIUSB_EEPROM_ERASE_FAILED  -46  /**< \ingroup err
		FTDIUSB: EEPROM erase failed */
#define COM_FTDIUSB_EEPROM_NOT_PRESENT  -47  /**< \ingroup err
		FTDIUSB: EEPROM not present */
#define COM_FTDIUSB_EEPROM_NOT_PROGRAMMED  -48  /**< \ingroup err
		FTDIUSB: EEPROM not programmed */
#define COM_FTDIUSB_INVALID_ARGS  -49  /**< \ingroup err
		FTDIUSB: invalid arguments */
#define COM_FTDIUSB_NOT_SUPPORTED  -50  /**< \ingroup err
		FTDIUSB: not supported */
#define COM_FTDIUSB_OTHER_ERROR  -51  /**< \ingroup err
		FTDIUSB: other error */
#define COM_PORT_ALREADY_OPEN  -52  /**< \ingroup err
		Error while opening the COM port: was already open */
#define COM_PORT_CHECKSUM_ERROR  -53  /**< \ingroup err
		Checksum error in received data from COM port */
#define COM_SOCKET_NOT_READY  -54  /**< \ingroup err
		Socket not ready, you should call the function again */
#define COM_SOCKET_PORT_IN_USE  -55  /**< \ingroup err
		Port is used by another socket */
#define COM_SOCKET_NOT_CONNECTED  -56  /**< \ingroup err
		Socket not connected (or not valid) */
#define COM_SOCKET_TERMINATED  -57  /**< \ingroup err
		Connection terminated (by peer) */
#define COM_SOCKET_NO_RESPONSE  -58  /**< \ingroup err
		Can't connect to peer */
#define COM_SOCKET_INTERRUPTED  -59  /**< \ingroup err
		Operation was interrupted by a nonblocked signal */
//
//  End of Interface Errors
//////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//  obsolet, provided for backward compatibility
#define PI_CNTR_MOVE_WITHOUT_INI				5			// name "INI" is misleading - we mean "reference"
#define PI_CNTR_INVALID_AXIS					23			// misleading name: INVALID means something different than "illegal"
//
/////////////////////////////////////////////////////////////////////////////

#endif // __PI_CONTROLLER_ERROS_H__

