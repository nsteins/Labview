// summary:	Declares the functions class
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the BENCHPIEZO_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// BENCHPRECISIONPIEZO_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef BENCHPRECISIONPIEZODLL_EXPORTS
#define BENCHPRECISIONPIEZO_API __declspec(dllexport)
#else
#define BENCHPRECISIONPIEZO_API __declspec(dllimport)
#endif

#include <OaIdl.h>

#pragma once

/** @defgroup BenchtopPrecisionPiezo Benchtop Precision Piezo
 *  This section details the Structures and Functions relavent to the  @ref PPC001_page "Benchtop Piezo"<br />
 *  For an example of how to connect to the device and perform simple operations use the following links:
 *  <list type=bullet>
 *    <item> \ref namespaces_ppc_ex_1 "Example of using the Thorlabs.MotionControl.Benchtop.PrecisionPiezo.DLL from a C or C++ project."<br />
 *									  This requires the DLL to be dynamical linked. </item>
 *    <item> \ref namespaces_ppc_ex_2 "Example of using the Thorlabs.MotionControl.Benchtop.PrecisionPiezo.DLL from a C# project"<br />
 *									  This uses Marshalling to load and access the C DLL. </item>
 *  </list>
 *  The Thorlabs.MotionControl.Benchtop.PrecisionPiezo.DLL requires the following DLLs
 *  <list type=bullet>
 *    <item> Thorlabs.MotionControl.DeviceManager. </item>
 *  </list>
 *  @{
 */

extern "C"
{
	/// \cond NOT_MASTER

	/// <summary> Values that represent FT_Status. </summary>
	enum FT_Status : short
	{
		FT_OK = 0x00, /// <OK - no error.
		FT_InvalidHandle = 0x01, ///<Invalid handle.
		FT_DeviceNotFound = 0x02, ///<Device not found.
		FT_DeviceNotOpened = 0x03, ///<Device not opened.
		FT_IOError = 0x04, ///<I/O error.
		FT_InsufficientResources = 0x05, ///<Insufficient resources.
		FT_InvalidParameter = 0x06, ///<Invalid parameter.
		FT_DeviceNotPresent = 0x07, ///<Device not present.
		FT_IncorrectDevice = 0x08 ///<Incorrect device.
	 };

	/// <summary> Values that represent THORLABSDEVICE_API. </summary>
	enum MOT_MotorTypes
	{
		MOT_NotMotor = 0,
		MOT_DCMotor = 1,
		MOT_StepperMotor = 2,
		MOT_BrushlessMotor = 3,
		MOT_CustomMotor = 100,
	};
	/// \endcond

	/// <summary> Information about the device generated from serial number. </summary>
	#pragma pack(1)
	struct TLI_DeviceInfo
	{
		/// <summary> The device Type ID, see \ref C_DEVICEID_page "Device serial numbers". </summary>
		DWORD typeID;
		/// <summary> The device description. </summary>
		char description[65];
		/// <summary> The device serial number. </summary>
		char serialNo[9];
		/// <summary> The USB PID number. </summary>
		DWORD PID;

		/// <summary> <c>true</c> if this object is a type known to the Motion Control software. </summary>
		bool isKnownType;
		/// <summary> The motor type (if a motor).
		/// 		  <list type=table>
		///				<item><term>MOT_NotMotor</term><term>0</term></item>
		///				<item><term>MOT_DCMotor</term><term>1</term></item>
		///				<item><term>MOT_StepperMotor</term><term>2</term></item>
		///				<item><term>MOT_BrushlessMotor</term><term>3</term></item>
		///				<item><term>MOT_CustomMotor</term><term>100</term></item>
		/// 		  </list> </summary>
		MOT_MotorTypes motorType;

		/// <summary> <c>true</c> if the device is a piezo device. </summary>
		bool isPiezoDevice;
		/// <summary> <c>true</c> if the device is a laser. </summary>
		bool isLaser;
		/// <summary> <c>true</c> if the device is a custom type. </summary>
		bool isCustomType;
		/// <summary> <c>true</c> if the device is a rack. </summary>
		bool isRack;
		/// <summary> Defines the number of channels available in this device. </summary>
		short maxChannels;
	};

	/// <summary> Structure containing the Hardware Information. </summary>
	/// <value> Hardware Information retrieved from tthe device. </value>
	struct TLI_HardwareInformation
	{
		/// <summary> The device serial number. </summary>
		/// <remarks> The device serial number is a serial number,<br />starting with 2 digits representing the device type<br /> and a 6 digit unique value.</remarks>
		DWORD serialNumber;
		/// <summary> The device model number. </summary>
		/// <remarks> The model number uniquely identifies the device type as a string. </remarks>
		char modelNumber[8];
		/// <summary> The device type. </summary>
		/// <remarks> Each device type has a unique Type ID: see \ref C_DEVICEID_page "Device serial numbers" </remarks>
		WORD type;
		/// <summary> The number of channels the device provides. </summary>
		short numChannels;
		/// <summary> The device notes read from the device. </summary>
		char notes[48];
		/// <summary> The device firmware version. </summary>
		DWORD firmwareVersion;
		/// <summary> The device hardware version. </summary>
		WORD hardwareVersion;
		/// <summary> The device dependant data. </summary>
		BYTE deviceDependantData[12];
		/// <summary> The device modification state. </summary>
		WORD modificationState;
	};

	/// <summary> The Piezo Control Modes. </summary>
	/// \ingroup Common
	enum PZ_ControlModeTypes : short
	{
		PZ_Undefined = 0, ///<Undefined
		PZ_OpenLoop = 1, ///<Open Loop mode.
		PZ_CloseLoop = 2, ///<Closed Loop mode.
		PZ_OpenLoopSmooth = 3, ///<Open Loop mode with smoothing.
		PZ_CloseLoopSmooth = 4 ///<Closed Loop mode with smoothing.
	};

	/// <summary> The Piezo Input Source Flags. </summary>
	/// \ingroup Common
	enum PZ_InputSourceFlags : short
	{
		PZ_SoftwareOnly = 0, ///<Only read input from software.
		PZ_ExternalSignal = 0x01, ///<Read input from software and External Signal.
		PZ_Potentiometer = 0x02, ///<Read input from software and Potentiometer.
		PZ_All = PZ_ExternalSignal | PZ_Potentiometer ///<Read input from all sources.
	};

	/// <summary>The Piezo Output LUT Operating Flags. </summary>
	/// \ingroup Common
	enum PZ_OutputLUTModes : short
	{
		PZ_Continuous = 0x01, ///<LUT waveform output continuously.
		PZ_Fixed = 0x02, ///<LUT waveform output for a Fixed number of cycles.
		PZ_OutputTrigEnable = 0x04, ///<Enable Output Triggering.
		PZ_InputTrigEnable = 0x08, ///<Enable Input triggering.
		PZ_OutputTrigSenseHigh = 0x10, ///<Output trigger sense is high.
		PZ_InputTrigSenseHigh = 0x20, ///<Input trigger sense is high.
		PZ_OutputGated = 0x40, ///<Output is gated.
		PZ_OutputTrigRepeat = 0x80, ///<Output trigger repeats.
	};

	/// <summary> Values that represent PPC_FilterState. </summary>
	enum PPC_DerivFilterState : short
	{
		/// <summary> An enum constant representing the filter on option. </summary>
		DerivFilterOn = 0x01,
		/// <summary> An enum constant representing the filter off option. </summary>
		DerivFilterOff = 0x02,
	};

	/// <summary> Values that represent PPC_FilterState. </summary>
	enum PPC_NotchFilterState : short
	{
		/// <summary> An enum constant representing the filter on option. </summary>
		NotchFilterOn = 0x01,
		/// <summary> An enum constant representing the filter off option. </summary>
		NotchFilterOff = 0x02,
	};

	/// <summary> Values that represent PPC_NotchFilterChannel. </summary>
	enum PPC_NotchFilterChannel : short
	{
		/// <summary> An enum constant representing the notch filter 1 option. </summary>
		NotchFilter1 = 0x01,
		/// <summary> An enum constant representing the notch filter 2 option. </summary>
		NotchFilter2 = 0x02,
		/// <summary> An enum constant representing the notch filter both option. </summary>
		NotchFilterBoth = 0x03,
	};

	/// <summary> Values that represent PPC_IOControlMode. </summary>
	enum PPC_IOControlMode : short
	{
		/// <summary> An enum constant representing the software only option. </summary>
		SWOnly = 0x00,
		/// <summary> An enum constant representing the extent bnc option. </summary>
		ExtBNC = 0x01,
		/// <summary> An enum constant representing the joystick option. </summary>
		Joystick = 0x02,
		/// <summary> An enum constant representing the joystick bnc option. </summary>
		JoystickBnc = 0x03,
	};

	/// <summary> Values that represent PPC_IOOutputModeMode. </summary>
	enum PPC_IOOutputMode : short
	{
		/// <summary> An enum constant representing the hv option. </summary>
		HV = 0x01,
		/// <summary> An enum constant representing the position raw option. </summary>
		PosRaw = 0x02,
		/// <summary> An enum constant representing the position corrected option. </summary>
		PosCorrected = 0x03,
	};

	/// <summary> Values that represent PPC_IOOutput Bandwidth. </summary>
	enum PPC_IOOutputBandwidth : short
	{
		/// <summary> An enum constant representing the operation unfiltered option. </summary>
		OP_Unfiltered = 0x01,
		/// <summary> An enum constant representing the operation 200 Hz option. </summary>
		OP_200Hz = 0x02,
	};

	/// <summary> Values that represent PPC_IOSourceDefinition. </summary>
	enum PPC_IOFeedbackSourceDefinition : short
	{
		/// <summary> An enum constant representing the operation unfiltered option. </summary>
		StrainGauge = 0x01,
		/// <summary> An enum constant representing the operation 200 Hz option. </summary>
		Capacitive = 0x02,
	};

	/// <summary> Values that represent PPC_DisplayIntensity. </summary>
	enum PPC_DisplayIntensity : short
	{
		/// <summary> An enum constant representing the bright option. </summary>
		Bright = 0x01,
		/// <summary> An enum constant representing the dim option. </summary>
		Dim = 0x02,
		/// <summary> An enum constant representing the off option. </summary>
		Off = 0x03,
	};

	/// <summary> PPC PID consts. </summary>
	struct PPC_PIDConsts
	{
		/// <summary> PID constants proportional. </summary>
		/// <remarks> The PID Proportional Gain constant, range 0 to 10000</remarks>
		float PIDConstsP;
		/// <summary> PID constants integral. </summary>
		/// <remarks> The PID Integral Gain constant, range 0 to 10000</remarks>
		float PIDConstsI;
		/// <summary> PID constants differential. </summary>
		/// <remarks> The PID Differential Gain constant, range 0 to 10000</remarks>
		float PIDConstsD;
		/// <summary> PID constants derivative low pass filter cut-off frequency. </summary>
		/// <remarks> The PID Differential Gain filter, range 0 to 10000</remarks>
		float PIDConstsDFc;
		/// <summary> PID Derivative filter on or off. </summary>
		/// <remarks> The Filter Enabled state:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Filter Enabled</term></item>
		///				<item><term>2</term><term>Filter Disabled.</term></item>
		/// 		  </list></remarks>
		PPC_DerivFilterState PIDDerivFilterOn;
	};

	/// <summary> PPC notch filter parameters. </summary>
	struct PPC_NotchParams
	{
		/// <summary> Flags that define which filters are updated with this structure. </summary>
		/// <remarks> The Notch Filter selection flags:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Filter 1 parameters are updated.</term></item>
		///				<item><term>2</term><term>Filter 2 parameters are updated.</term></item>
		///				<item><term>2</term><term>Filter 1 and 2 parameters are updated.</term></item>
		/// 		  </list></remarks>
		PPC_NotchFilterChannel filterNo;
		/// <summary> Notch filter 1 centre frequency. </summary>
		/// <remarks> The Notch Filter Center Frequency, range 20 to 500Hz</remarks>
		float filter1Fc;
		/// <summary> Notch filter 1 Q. </summary>
		/// <remarks> The Notch Filter Q Value, range 0.2 to 100</remarks>
		float filter1Q;
		/// <summary> Notch filter 1 on or off. </summary>
		/// <remarks> The Filter Enabled state:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Filter Enabled</term></item>
		///				<item><term>2</term><term>Filter Disabled.</term></item>
		/// 		  </list></remarks>
		PPC_NotchFilterState notchFilter1On;
		/// <summary> Notch filter 2 centre frequency. </summary>
		/// <remarks> The Notch Filter Center Frequency, range 20 to 500Hz</remarks>
		float filter2Fc;
		/// <summary> Notch filter 2 Q. </summary>
		/// <remarks> The Notch Filter Q Value, range 0.2 to 100</remarks>
		float filter2Q;
		/// <summary> Notch filter 2 on or off. </summary>
		/// <remarks> The Filter Enabled state:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Filter Enabled</term></item>
		///				<item><term>2</term><term>Filter Disabled.</term></item>
		/// 		  </list></remarks>
		PPC_NotchFilterState notchFilter2On;

	};

	/// <summary> PPC i/o settings. </summary>
	struct PPC_IOSettings
	{
		/// <summary> Voltage/Position control input source. </summary>
		/// <remarks> The enabled input sources:
		/// 		  <list type=table>
		///				<item><term>0</term><term>Software input only.</term></item>
		///				<item><term>1</term><term>Software and external BNC.</term></item>
		///				<item><term>2</term><term>Software and external joystick.</term></item>
		///				<item><term>2</term><term>Software, External BNC and Joystick.</term></item>
		/// 		  </list></remarks>
		PPC_IOControlMode controlSrc;
		/// <summary> Monitor output BNC signal. </summary>
		/// <remarks> The Monitor output modes:
		/// 		  <list type=table>
		///				<item><term>1</term><term>High Voltage output.</term></item>
		///				<item><term>2</term><term>Position (Uncorrected).</term></item>
		///				<item><term>2</term><term>Position (Corrected).</term></item>
		/// 		  </list></remarks>
		PPC_IOOutputMode monitorOPSig;
		/// <summary> Monitor output bandwidth. </summary>
		/// <remarks> The output bandwidth modes:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Unfiltered.</term></item>
		///				<item><term>2</term><term>200Hz filter.</term></item>
		/// 		  </list></remarks>
		PPC_IOFeedbackSourceDefinition monitorOPBandwidth;
		/// <summary> Type of feedback in operation. </summary>
		/// <remarks> The feedback loop mode (read only):
		/// 		  <list type=table>
		///				<item><term>1</term><term>Strain Gauge.</term></item>
		///				<item><term>2</term><term>Capacitive.</term></item>
		/// 		  </list></remarks>
		PPC_IOFeedbackSourceDefinition feedbackSrc;
		/// <summary> Brightness of the from panel LED's. </summary>
		/// <remarks> The display brightness:
		/// 		  <list type=table>
		///				<item><term>1</term><term>Bright.</term></item>
		///				<item><term>2</term><term>Dim.</term></item>
		///				<item><term>3</term><term>Off.</term></item>
		/// 		  </list></remarks>
		PPC_DisplayIntensity FPBrightness;
		/// <summary> reserved field. </summary>
		WORD reserved1;
	};
	#pragma pack()

    /// <summary> Build the DeviceList. </summary>
    /// <remarks> This function builds an internal collection of all devices found on the USB that are not currently open. <br />
    /// 		  NOTE, if a device is open, it will not appear in the list until the device has been closed. </remarks>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_BuildDeviceList(void);

	/// <summary> Gets the device list size. </summary>
	/// 		  \include CodeSnippet_identification.cpp
	/// <returns> Number of devices in device list. </returns>
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListSize();

	/// <summary> Get the entire contents of the device list. </summary>
	/// <param name="stringsReceiver"> Outputs a SAFEARRAY of strings holding device serial numbers. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceList(SAFEARRAY** stringsReceiver);

	/// <summary> Get the contents of the device list which match the supplied typeID. </summary>
	/// <param name="stringsReceiver"> Outputs a SAFEARRAY of strings holding device serial numbers. </param>
	/// <param name="typeID">					    The typeID of devices to match. </param>
	/// <param name="typeID">The typeID of devices to match, see \ref C_DEVICEID_page "Device serial numbers". </param>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);

	/// <summary> Get the contents of the device list which match the supplied typeIDs. </summary>
	/// <param name="stringsReceiver"> Outputs a SAFEARRAY of strings holding device serial numbers. </param>
	/// <param name="typeIDs"> list of typeIDs of devices to be matched, see \ref C_DEVICEID_page "Device serial numbers"</param>
	/// <param name="length"> length of type list</param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);

	/// <summary> Get the entire contents of the device list. </summary>
	/// <param name="receiveBuffer"> a buffer in which to receive the list as a comma separated string. </param>
	/// <param name="sizeOfBuffer">	The size of the output string buffer. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer);

	/// <summary> Get the contents of the device list which match the supplied typeID. </summary>
	/// <param name="receiveBuffer"> a buffer in which to receive the list as a comma separated string. </param>
	/// <param name="sizeOfBuffer">	The size of the output string buffer. </param>
	/// <param name="typeID"> The typeID of devices to be matched, see \ref C_DEVICEID_page "Device serial numbers"</param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID);

	/// <summary> Get the contents of the device list which match the supplied typeIDs. </summary>
	/// <param name="receiveBuffer"> a buffer in which to receive the list as a comma separated string. </param>
	/// <param name="sizeOfBuffer">	The size of the output string buffer. </param>
	/// <param name="typeIDs"> list of typeIDs of devices to be matched, see \ref C_DEVICEID_page "Device serial numbers"</param>
	/// <param name="length"> length of type list</param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	BENCHPRECISIONPIEZO_API short __cdecl TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length);

	/// <summary> Get the device information from the USB port. </summary>
	/// <remarks> The Device Info is read from the USB port not from the device itself.<remarks>
	/// <param name="serialNo"> The serial number of the device. </param>
	/// <param name="info">    The <see cref="TLI_DeviceInfo"/> device information. </param>
	/// <returns> 1 if successful, 0 if not. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	BENCHPRECISIONPIEZO_API short _cdecl TLI_GetDeviceInfo(char const * serialNo, TLI_DeviceInfo *info);

	/// <summary> Open the device for communications. </summary>
	/// <param name="serialNo">	The serial no of the device to be connected. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_connectionN.cpp
	/// <seealso cref="PPC_Close(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_Open(char const * serialNo);

	/// <summary> Disconnect and close the device. </summary>
	/// <param name="serialNo">	The serial no of the device to be disconnected. </param>
    /// 		  \include CodeSnippet_connectionN.cpp
	/// <seealso cref="PPC_Open(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API void __cdecl PPC_Close(char const * serialNo);

	/// <summary> Sends a command to the device to make it identify iteself. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	BENCHPRECISIONPIEZO_API void __cdecl PPC_Identify(char const * serialNo);

	/// <summary> Tells the device that it is being disconnected. </summary>
	/// <remarks> This does not disconnect the communications.<br />
	/// 		  To disconnect the communications, call the <see cref="PPC_Close(char const * serialNo)" /> function. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_Disconnect(char const * serialNo);

	/// <summary> Gets the hardware information from the device. </summary>
	/// <param name="serialNo">		    The device serial no. </param>
	/// <param name="modelNo">		    Address of a buffer to receive the model number string. Minimum 8 characters </param>
	/// <param name="sizeOfModelNo">	    The size of the model number buffer, minimum of 8 characters. </param>
	/// <param name="type">		    Address of a WORD to receive the hardware type number. </param>
	/// <param name="numChannels">	    Address of a short to receive the  number of channels. </param>
	/// <param name="notes">		    Address of a buffer to receive the notes describing the device. </param>
	/// <param name="sizeOfNotes">		    The size of the notes buffer, minimum of 48 characters. </param>
	/// <param name="firmwareVersion"> Address of a DWORD to receive the  firmware version number made up of 4 byte parts. </param>
	/// <param name="hardwareVersion"> Address of a WORD to receive the  hardware version number. </param>
	/// <param name="modificationState">	    Address of a WORD to receive the hardware modification state number. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetHardwareInfo(char const * serialNo, char * modelNo, DWORD sizeOfModelNo, WORD * type, WORD * numChannels, 
															char * notes, DWORD sizeOfNotes, DWORD * firmwareVersion, WORD * hardwareVersion, WORD * modificationState);

	/// <summary> Gets the hardware information in a block. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="hardwareInfo"> Address of a TLI_HardwareInformation structure to receive the hardware information. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetHardwareInfoBlock(char const  * serialNo, TLI_HardwareInformation *hardwareInfo);

	/// <summary> Gets version number of the device firmware. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device firmware version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	BENCHPRECISIONPIEZO_API DWORD __cdecl PPC_GetFirmwareVersion(char const * serialNo);

	/// <summary> Gets version number of the device software. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device software version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	BENCHPRECISIONPIEZO_API DWORD __cdecl PPC_GetSoftwareVersion(char const * serialNo);

	/// <summary> Update device with stored settings. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
    /// 		  \include CodeSnippet_connection1.cpp
	BENCHPRECISIONPIEZO_API bool __cdecl PPC_LoadSettings(char const * serialNo);

	/// <summary> Disable the channel so that motor can be moved by hand. </summary>
	/// <remarks> When disabled power is removed from the actuator.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_EnableChannel(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_DisableChannel(char const * serialNo);

	/// <summary> Enable channel for computer control. </summary>
	/// <remarks> When enabled power is applied to the actuator so it is fixed in position.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_DisableChannel(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_EnableChannel(char const * serialNo);


	/// <summary> Registers a callback on the message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="functionPointer"> A function pointer to be called whenever messages are received. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_RegisterMessageCallback(char const * serialNo, void (* functionPointer)());

	/// <summary> Gets the MessageQueue size. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> number of messages in the queue. </returns>
	BENCHPRECISIONPIEZO_API int __cdecl PPC_MessageQueueSize(char const * serialNo);

	/// <summary> Clears the device message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_ClearMessageQueue(char const * serialNo);

	/// <summary> Get the next MessageQueue item if it is available. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> Address of the WORD to receive the message type. </param>
	/// <param name="messageID"> Address of the WORD to receive themessage ID. </param>
	/// <param name="messageData"> Address of the DWORD to receive the messageData. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	BENCHPRECISIONPIEZO_API bool __cdecl PPC_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary> Get the next MessageQueue item if it is available. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> Address of the WORD to receive the message type. </param>
	/// <param name="messageID"> Address of the WORD to receive themessage ID. </param>
	/// <param name="messageData"> Address of the DWORD to receive the messageData. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	BENCHPRECISIONPIEZO_API bool __cdecl PPC_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary> Starts the internal polling loop which continuously requests position and status. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="milliseconds">	    The milliseconds polling rate. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="PPC_StopPolling(char const * serialNo)" />
	/// <seealso cref="PPC_PollingDuration(char const * serialNo)" />
	/// <seealso cref="PPC_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="PPC_RequestPosition(char const * serialNo)" />
	/// \include CodeSnippet_connectionN.cpp
	BENCHPRECISIONPIEZO_API bool __cdecl PPC_StartPolling(char const * serialNo, int milliseconds);

	/// <summary> Gets the polling loop duration. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The time between polls in milliseconds or 0 if polling is not active. </returns>
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="PPC_StopPolling(char const * serialNo)" />
	/// \include CodeSnippet_connectionN.cpp
	BENCHPRECISIONPIEZO_API long __cdecl PPC_PollingDuration(char const * serialNo);

	/// <summary> Stops the internal polling loop. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="PPC_PollingDuration(char const * serialNo)" />
	/// \include CodeSnippet_connectionN.cpp
	BENCHPRECISIONPIEZO_API void __cdecl PPC_StopPolling(char const * serialNo);

	/// <summary> Requests that all settings are download from device. </summary>
	/// <remarks> This function requests that the device upload all it's settings to the DLL.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_RequestSettings(char const * serialNo);

	/// <summary> Requests the status bits and position. </summary>
	/// <remarks> This needs to be called to get the device to send it's current position and status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PPC_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="PPC_GetPosition(char const * serialNo)" />
	/// <seealso cref="PPC_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_RequestStatus(char const * serialNo);

	/// <summary> Request the status bits which identify the current device state. </summary>
	/// <remarks> This needs to be called to get the device to send it's current status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PPC_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_RequestStatusBits(char const * serialNo);

	/// <summary>Get the current status bits. </summary>
	/// <remarks> This returns the latest status bits received from the device.<br />
	/// 		  To get new status bits, use <see cref="PPC_RequestStatusBits(char const * serialNo)" /> <br />
	///			  or use <see cref="BPC_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns>	The status bits from the device <list type=table>
	///				<item><term>0x00000001</term><term>Piezo actuator connected (1=Connected, 0=Not connected).</term></item>
	///				<item><term>0x00000002</term><term>For Future Use.</term></item>
	///				<item><term>0x00000004</term><term>For Future Use.</term></item>
	///				<item><term>0x00000008</term><term>For Future Use.</term></item>
	///				<item><term>0x00000010</term><term>Piezo channel has been zeroed (1=Zeroed, 0=Not zeroed).</term></item>
	///				<item><term>0x00000020</term><term>Piezo channel is zeroing (1=Zeroing, 0=Not zeroing).</term></item>
	///				<item><term>0x00000040</term><term>For Future Use.</term></item>
	///				<item><term>0x00000080</term><term>For Future Use.</term></item>
	///				<item><term>0x00000100</term><term>Strain gauge feedback connected (1=Connected, 0=Not connected).</term></item>
	///				<item><term>0x00000200</term><term>For Future Use.</term></item>
	///				<item><term>0x00000400</term><term>Position control mode (1=Closed loop, 0=Open loop).</term></item>
	///				<item><term>0x00000800</term><term>For Future Use.</term></item>
	///				<item><term>0x00001000</term><term></term></item>
	///				<item><term>...</term><term></term></item>
	///				<item><term>0x00080000</term><term></term></item>
	///				<item><term>0x00100000</term><term>Digital input 1 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00200000</term><term>Digital input 2 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00400000</term><term>Digital input 3 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00800000</term><term>Digital input 4 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x01000000</term><term>Digital input 5 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x02000000</term><term>Digital input 6 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x04000000</term><term>Digital input 7 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x08000000</term><term>Digital input 8 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x10000000</term><term>For Future Use.</term></item>
	///				<item><term>0x20000000</term><term>Active (1=Indicates Unit Is Active, 0=Not Active).</term></item>
	///				<item><term>0x40000000</term><term>For Future Use.</term></item>
	///				<item><term>0x80000000</term><term>Channel enabled (1=Enabled, 0=Disabled).</term></item>
	/// 		  </list> <remarks> Bits 21 to 28 (Digital Input States) are only applicable if the associated digital input is fitted to your controller - see the relevant handbook for more details. </remarks> </returns>
	/// <seealso cref="PPC_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="PPC_RequestStatus(char const * serialNo)" />
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	BENCHPRECISIONPIEZO_API DWORD __cdecl PPC_GetStatusBits(char const * serialNo);

	/// <summary> Requests the current output voltage or position depending on current mode. </summary>
	/// <remarks> This needs to be called to get the device to send it's current position.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PPC_GetPosition(char const * serialNo)" />
	/// <seealso cref="PPC_StartPolling(char const * serialNo, int milliseconds)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_RequestPosition(char const * serialNo);

	/// <summary>	Resets all parameters to power-up values. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_ResetParameters(char const * serialNo);

	/// <summary> Sets the voltage output to zero and defines the ensuing actuator position az zero. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetZero(char const * serialNo);

	/// <summary> Gets the Position Control Mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The control mode <list type=table>
	///				<item><term>Open Loop</term><term>1</term></item>
	///				<item><term>Closed Loop</term><term>2</term></item>
	///				<item><term>Open Loop smoothed</term><term>3</term></item>
	///				<item><term>Closed Loop smoothed</term><term>4</term></item>
	/// 		  </list> </returns>
	/// <seealso cref="PPC_SetPositionControlMode(char const * serialNo, PZ_ControlModeTypes mode)" />
	BENCHPRECISIONPIEZO_API PZ_ControlModeTypes __cdecl PPC_GetPositionControlMode(char const * serialNo);

	/// <summary> Sets the Position Control Mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="mode"> The control mode <list type=table>
	///				<item><term>Open Loop</term><term>1</term></item>
	///				<item><term>Closed Loop</term><term>2</term></item>
	///				<item><term>Open Loop smoothed</term><term>3</term></item>
	///				<item><term>Closed Loop smoothed</term><term>4</term></item>
	/// 		  </list>. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_GetPositionControlMode(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetPositionControlMode(char const * serialNo, PZ_ControlModeTypes mode);

	// Voltage Functions
	
	/// <summary> Gets the maximum output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The maximum output voltage, range 0 to 1500 (0.0 to 150.0 Volts). </returns>
	/// <seealso cref="PPC_SetMaxOutputVoltage(char const * serialNo, short eVoltage)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetMaxOutputVoltage(char const * serialNo);

	/// <summary> Sets the maximum output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="eVoltage">	The maximum output voltage, 0 to 1500 (0.0 to 150.0 Volts). </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_GetMaxOutputVoltage(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetMaxOutputVoltage(char const * serialNo, short eVoltage);

	/// <summary> Gets the set Output Voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The voltage as a percentage of MaxOutputVoltage,<br />
	/// 		  range -32767 to 32767 equivalent to -100% to 100%. </returns>
	/// <seealso cref="PPC_SetOutputVoltage(char const * serialNo, short volts)" />
	/// <seealso cref="PPC_SetMaxOutputVoltage(char const * serialNo, short eVoltage)" />
	/// <seealso cref="PPC_GetMaxOutputVoltage(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetOutputVoltage(char const * serialNo);

	/// <summary> Sets the output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="volts"> The voltage as a percentage of MaxOutputVoltage,<br />
	/// 		  range -32767 to 32767 equivalent to -100% to 100%. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_GetOutputVoltage(char const * serialNo)" />
	/// <seealso cref="PPC_SetMaxOutputVoltage(char const * serialNo, short eVoltage)" />
	/// <seealso cref="PPC_GetMaxOutputVoltage(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetOutputVoltage(char const * serialNo, short volts);

	/// <summary> Gets the maximum travel of the device. </summary>
	/// <remarks> This requires an actuator with built in position sensing</remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns>	The distance in steps of 100nm,<br />
	/// 			range 0 to 65535 (10000 is equivalent to 1mm). </returns>
	BENCHPRECISIONPIEZO_API WORD __cdecl PPC_GetMaximumTravel(char const * serialNo);

	/// <summary> Gets the position when in closed loop mode. </summary>
	/// <remarks> The result is undefined if not in closed loop mode</remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The position as a percentage of maximum travel,<br />
	/// 		  range -32767 to 32767, equivalent to -100 to 100%. </returns>
	/// <seealso cref="PPC_SetPosition(char const * serialNo, short position)" />
	/// <seealso cref="PPC_SetPositionControlMode(char const * serialNo, PZ_ControlModeTypes mode)" />
	/// <seealso cref="PPC_GetPositionControlMode(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetPosition(char const * serialNo);

	/// <summary> Sets the position when in closed loop mode. </summary>
	/// <remarks> The command is ignored if not in closed loop mode</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="position"> The position as a percentage of maximum travel,<br />
	/// 		  range 0 to 32767, equivalent to 0 to 100%. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PPC_GetPosition(char const * serialNo)" />
	/// <seealso cref="PPC_SetPositionControlMode(char const * serialNo, PZ_ControlModeTypes mode)" />
	/// <seealso cref="PPC_GetPositionControlMode(char const * serialNo)" />
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetPosition(char const * serialNo, short position);

	/// <summary> Gets the PPC IO Settings. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="ioSettings"> The i/o settings <see cref="PPC_IOSettings"/>. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetIOSettings(const char * serialNo, PPC_IOSettings *ioSettings);

	/// <summary> Sets the PPC IO Setting. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="ioSettings"> The i/o setting <see cref="PPC_IOSettings"/>. </returns> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetIOSettings(const char * serialNo, PPC_IOSettings *ioSettings);

	/// <summary> Gets the PPC Notch Filter Parameters. </summary>
	/// <remarks> The PPC Notch Filter Parameters. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="notchParams"> Options for controlling the notch <see cref="PPC_NotchParams"/>. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetNotchParams(const char * serialNo, PPC_NotchParams *notchParams);

	/// <summary> Sets the PPC Notch Filter Parameters. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="notchParams"> The PPC Notch Filter Parameters <see cref="PPC_NotchParams"/>. </returns> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetNotchParams(const char * serialNo, PPC_NotchParams *notchParams);

	/// <summary> Gets the PPC PID Constants. </summary>
	/// <remarks> The PPC PID Constants. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="pidConsts"> The PID consts <see cref="PPC_PIDConsts"/>. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_GetPIDConsts(const char * serialNo, PPC_PIDConsts *pidConsts);

	/// <summary> Sets the PPC PID Constants. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="pidConsts"> The PPC PID Constants <see cref="PPC_PIDConsts"/>. </returns> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	BENCHPRECISIONPIEZO_API short __cdecl PPC_SetPIDConsts(const char * serialNo, PPC_PIDConsts *pidConsts);
}
/** @} */ // BenchtopPiezo
