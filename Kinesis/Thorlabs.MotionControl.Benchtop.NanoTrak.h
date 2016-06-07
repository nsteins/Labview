// summary:	Declares the functions class
// The following ifdef block is the standard way of creating macros which make exporting from a
// DLL simpler. All files within this DLL are compiled with the NANOTRAKDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project that
// uses this DLL. This way any other project whose source files include this file see 
// NANOTRAK_API functions as being imported from a DLL, whereas this DLL sees
// symbols defined with this macro as being exported.
#ifdef NANOTRAKDLL_EXPORTS
#define NANOTRAK_API __declspec(dllexport)
#else
#define NANOTRAK_API __declspec(dllimport)
#endif

#include <OaIdl.h>

#pragma once

/** @defgroup BenchtopNanoTrak Benchtop NanoTrak
 *  This section details the Structures and Functions relavent to the  @ref BNT001_page "Benchtop NanoTrak"<br />
 *  For an example of how to connect to the device and perform simple operations use the following links:
 *  <list type=bullet>
 *    <item> \ref namespaces_bnt_ex_1 "Example of using the Thorlabs.MotionControl.Benchtop.NanoTrak.DLL from a C or C++ project."<br />
 *									  This requires the DLL to be dynamical linked. </item>
 *    <item> \ref namespaces_bnt_ex_2 "Example of using the Thorlabs.MotionControl.Benchtop.NanoTrak.DLL from a C# project"<br />
 *									  This uses Marshalling to load and access the C DLL. </item>
 *  </list>
 *  The Thorlabs.MotionControl.Benchtop.NanoTrak.DLL requires the following DLLs
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

	/// <summary> NanoTrak Signal States. </summary>
	/// \ingroup Common
	enum NT_SignalState
	{
		NT_BadSignal = 0x00, ///<The bad signal state.
		NT_GoodSignal = 0x01, ///<The good signal state.
	};

	/// <summary> NanoTrak Operating mode. </summary>
	/// \ingroup Common
	enum NT_Mode : WORD
	{
		NT_ModeUndefined = 0x00, ///< Undefined nanoTrak mode.
		NT_Piezo = 0x01, ///< The Piezo nanoTrak mode.
		NT_Latch = 0x02, ///< The Latched NanoTrak mode.
		NT_Tracking = 0x03, ///< The Tracking NanoTrak mode.
		NT_HorizontalTracking = 0x04, ///< The Horizontal Tracking NanoTrak mode.
		NT_VerticalTracking = 0x05, ///< The Vertical Tracking NanoTrak mode.
	};

	/// <summary> Values that represent NanoTrak Operating Modes. </summary>
	/// \ingroup Common
	enum NT_ControlMode : WORD
	{
		NT_ControlModeUndefined = 0x00, ///<undefined nanoTrak mode.
		NT_OpenLoop = 0x01, ///<the open loop nanoTrak mode.
		NT_ClosedLoop = 0x02, ///<the closed loop nanoTrak mode.
		NT_OpenLoopSmoothed = 0x03, ///<the smoothed open loop nanoTrak mode.
		NT_ClosedLoopSmoothed = 0x04, ///<the smoother closed loop nanoTrak mode.
	};

	/// <summary> NanoTrak Feedback Source. </summary>
	/// \ingroup Common
	enum NT_FeedbackSource : WORD
	{
		NT_FeedbackSourceUndefined = 0x00, ///< Undefined.
		NT_TIA = 0x01, ///< The TIA input source.
		NT_BNC_1v = 0x02, ///< The  BNC input source at 1V.
		NT_BNC_2v = 0x03, ///< The  BNC input source at 2V.
		NT_BNC_5v = 0x04, ///< The  BNC input source at 5V.
		NT_BNC_10v = 0x05, ///< The  BNC input source at 10V
	};

	/// <summary> Defines the number TIA ranges. </summary>
	#define NUM_TIA_RANGES	16

	/// <summary> NanoTrak TIA Ranges. </summary>
	/// \ingroup Common
	enum NT_TIARange : WORD
	{
		NT_TIARange1_3nA = 0x0003, ///< 3nA range.
		NT_TIARange2_10nA = 0x0004, ///< 10nA range.
		NT_TIARange3_30nA = 0x0005, ///< 30nA range.
		NT_TIARange4_100nA = 0x0006, ///< 100 nA range.
		NT_TIARange5_300nA = 0x0007, ///< 300 nA range.
		NT_TIARange6_1uA = 0x0008, ///< 1 uA range.
		NT_TIARange7_3uA = 0x0009, ///< 3 uA range.
		NT_TIARange8_10uA = 0x000A, ///< 10 uA range.
		NT_TIARange9_30uA = 0x000B, ///< 30 uA range.
		NT_TIARange10_100uA = 0x000C, ///< 100 uA range.
		NT_TIARange11_300uA = 0x000D, ///< 300 uA range.
		NT_TIARange12_1mA = 0x000E, ///< 1 mA range.
		NT_TIARange13_3mA = 0x000F, ///< 3 mA range.
		NT_TIARange14_10mA = 0x0010 ///< 10 mA range.
	};

	/// <summary> The Odd or Even TIA Range specification. </summary>
	/// \ingroup Common
	enum NT_OddOrEven : WORD
	{
		NT_OddAndEven = 0x0001, ///< Both odd or even TIA ranges. 
		NT_Odd = 0x0002, ///< Odd TIA ranges.
		NT_Even = 0x0003 ///< Even TIA ranges.
	};

	/// <summary> The in range specification. </summary>
	/// \ingroup Common
	enum NT_UnderOrOver : WORD
	{
		NT_InRange = 0x0001, ///< Signal is in range. 
		NT_UnderRange = 0x0002, ///< Signal is under range. 
		NT_OverRange = 0x0003 ///< Signal is over range. 
	};

	/// <summary> Circle diameter mode types. </summary>
	/// \ingroup Common
	enum NT_CircleDiameterMode : WORD
	{
		NT_ParameterCircleMode = 0x0001, ///<Use the fixed diameter definition.
		NT_AbsPowerCircleMode = 0x0002, ///<Use the algorithm to calculate circle diameter.
		NT_LUTCircleMode = 0x0003 ///<Use the circle look up table (LUT).
	};

	/// <summary> The methods used by circle diameter algorithm. </summary>
	/// \ingroup Common
	enum NT_CircleAdjustment : WORD
	{
		NT_LinearCircleAdjustment = 0x0001, ///<Linear Adjustment method.
		NT_LogCircleAdjustment = 0x0002, ///<Logarithmic circle adjustment.
		NT_SquareCircleAdjustment = 0x0003, ///<Square circle adjustment.
		NT_CubeCircleAdjustment = 0x0004 ///<Cubic circle adjustment.
	};

	/// <summary> TIA Ranging Mode. </summary>
	/// \ingroup Common
	enum NT_TIARangeMode : WORD
	{
		NT_TIARangeModeUndefined = 0x0000, ///<Undefined.
		NT_AutoRangeAtSelected = 0x0001, ///<Autorange TIA at selected range.
		NT_ManualRangeAtSelected = 0x0002, ///<Manual TIA at selected range.
		NT_ManualRangeAtParameter = 0x0003, ///<Manual TIA at supplied parameter range.
		NT_AutoRangeAtParameter = 0x0004 ///<Autorange TIA at supplied parameter range.
	};

	/// <summary> Values that represent Low Pass Frequencdies. </summary>
	/// \ingroup Common
	enum NT_LowPassFrequency : WORD
	{
		NT_LowPassNone, ///<Low Pass Filter Diasabled.
		NT_LowPass_1Hz, ///<Low Pass Filter at 1 Hz.
		NT_LowPass_3Hz, ///<Low Pass Filter at 3 Hz.
		NT_LowPass_10Hz, ///<Low Pass Filter at 10 Hz.
		NT_LowPass_30Hz, ///<Low Pass Filter at 30 Hz.
		NT_LowPass_100Hz ///<Low Pass Filter at 100 Hz.
	};

	/// <summary> NanoTrak Output Voltage Range. </summary>
	/// \ingroup Common
	enum NT_VoltageRange : WORD
	{
		NT_VoltageRangeUndefined = 0x0000, ///<Undefined.
		NT_VoltageRange_5v = 0x0001, ///<5v output rangen.
		NT_VoltageRange_10v = 0x0002 ///<10v output range.
	};

	/// <summary> NanoTrak Output Route. </summary>
	/// \ingroup Common
	enum NT_OutputVoltageRoute : WORD
	{
		NT_SMAOnly = 0x0001, ///<Output to SMA connector only.
		NT_HubOrSMA = 0x0002 ///<Output to SMA connector and Hub.
	};

	/// <summary> The Input mode units. </summary>
	/// \ingroup Common
	enum NT_PowerInputUnits : WORD
	{
		NT_Amps, ///<Display as mAmps.
		NT_Watts, ///<Display as mWatts
		NT_Db ///<Display as dBm.
	};

	/// <summary> The SMA units. </summary>
	/// \ingroup Common
	enum NT_SMA_Units : WORD
	{
		NT_Voltage, ///<SMA output as Voltage.
		NT_FullRange, ///<SMA output as % Full Range.
		NT_UserDefined ///<SMA output as User Defined.
	};

	/// <summary> HV amp output current limit definitions. </summary>
	enum BNT_CurrentLimit : WORD
	{
		NT_CurrentLimit_100mA = 0x00,
		NT_CurrentLimit_250mA = 0x01,
		NT_CurrentLimit_500mA = 0x02
	};

	/// <summary> HV amp output low pass filter definitions. </summary>
	enum BNT_OutputLowPassFilter : WORD
	{
		NT_OutputFilter_10Hz = 0x0,
		NT_OutputFilter_100Hz = 0x1,
		NT_OutputFilter_5kHz = 0x2,
		NT_OutputFilter_None = 0x3,
	};

	/// <summary> Feedback signal selection definitions. </summary>
	enum BNT_FeedbackSignalSelection : WORD
	{
		NT_FeedbackSignalDC = 0x0000,
		NT_FeedbackSignalAC = 0xFFFF,
	};

	/// <summary> Feedback signal selection definitions. </summary>
	enum BNT_BNCTriggerModes : WORD
	{
		NT_BNCModeTrigger = 0x0000,
		NT_BNCModeLVOut = 0xFFFF,
	};

	/// <summary> Trigger/LV Out BNCs mode selection definitions. </summary>
	struct BNT_IO_Settings
	{
		/// <summary> Channel ident. </summary>
		WORD channel;
		/// <summary> Channel HV amp output current limit. </summary>
		BNT_CurrentLimit amplifierCurrentLimit;
		/// <summary> HV amp output low pass filter setting FOR BOTH CHANNELS. </summary>
		BNT_OutputLowPassFilter amplifierLowPassFilter;
		/// <summary> Channel feedback signal (legacy AC or DC differential). </summary>
		BNT_FeedbackSignalSelection feedbackSignal;
		/// <summary> CHANNEL INDEPENDANT Trigger input/output or LV output mode selection. DOES NOT APPLY TO MODULE VARIANTS. </summary>
		BNT_BNCTriggerModes BNCtriggerOrLowVoltageOut;
	};

#pragma pack(1)

	/// <summary> Structure used to hold any parameter which has a horizontal and vertical component, i.e. position. </summary>
	struct NT_HVComponent
	{
		/// <summary> The horizontal component, range 0 to 65535, usage dependant. </summary>
		WORD horizontalComponent;
		/// <summary> The vertical component, range 0 to 65535, useage dependant. </summary>
		WORD verticalComponent;
	};

	/// <summary> Structure containing the scan circle parameters. </summary>
	struct NT_CircleParameters
	{
		/// <summary> Circle diameter mode, <see cref="NT_CircleDiameterMode" />
		///		<list type=table>
		///			<item><term>Use supplied Parameters.</term><term>1</term></item>
		///			<item><term>Use supplied Algorithm.</term><term>2</term></item>
		///			<item><term>use Lookup Table (LUT)</term><term>3</term></item>
		///		</list> </summary>
		NT_CircleDiameterMode mode;
		/// <summary> Circle Diameter where 65535 corresponding to maximum voltage, 10 NT units. </summary>
		WORD diameter;
		/// <summary> Number of samples read per revolution (multiple of 4). </summary>
		WORD samplesPerRevolution;
		/// <summary> Minimum diameter imposed on algorithm 0 to 32767 in NT units. </summary>
		WORD minDiameter;
		/// <summary> Maximum diameter imposed on algorithm 0 to 32767 in NT units. </summary>
		WORD maxDiameter;
		/// <summary> Algorithm adjustment method: <see cref="NT_CircleAdjustment" />
		/// 	 <list type=table>
		///			<item><term>Linear Algorithm.</term><term>1</term></item>
		///			<item><term>Logarithmic algorithm.</term><term>2</term></item>
		///			<item><term>Square algorithm</term><term>3</term></item>
		///			<item><term>Cubic algorithm</term><term>4</term></item>
		///		</list> </summary>
		NT_CircleAdjustment algorithmAdjustment;
	};

	/// <summary> Structure containing circle the diameter lookup table entries. </summary>
	struct NT_CircleDiameterLUT
	{
		/// <summary> Array of diameters for each TIA range with 65535 corresponding to maximum voltage. </summary>
		WORD LUTdiameter[NUM_TIA_RANGES];
	};

	/// <summary> Structure containing TIA range parameters. </summary>
	struct NT_TIARangeParameters
	{
		/// <summary> Chooses automatic or manual ranging and currently selected or new range. <see cref="NT_TIARangeMode" />
		/// 	<list type=table>
		///			<item><term>Automatic Ranging at current range.</term><term>1</term></item>
		///			<item><term>Manual Ranging at current range.</term><term>2</term></item>
		///			<item><term>Manual Ranging using supplied range</term><term>3</term></item>
		///			<item><term>Automatic ranging using supplied range</term><term>4</term></item>
		///		</list>  </summary>
		NT_TIARangeMode mode;
		/// <summary> Up switching threshold as a percentage of the current range.<br />
		/// 		  When in an automatic mode, the range rises above this threshold, the TIA range will rise to the higher TIA range.<br />
		/// 		  Range 0 to 1000 equivalent to 0 to 100% </summary>
		WORD upLimit;
		/// <summary> Down switching threshold as a percentage of the current range.<br />
		/// 		  When in an automatic mode, the range drops below this threshold, the TIA range will drop to the lower TIA range.<br />
		/// 		  Range 0 to 1000 equivalent to 0 to 100%  </summary>
		WORD downLimit;
		/// <summary> The number of samples for averaging to give auto-range input from 2 to 32. </summary>
		short settleSamples;
		/// <summary> parameter determining which TIA ranges to include when auto ranging: <see cref="NT_OddOrEven" />
		/// 	<list type=table>
		///			<item><term>All TIA Ranges.</term><term>1</term></item>
		///			<item><term>Only use Odd TIA ranges.</term><term>2</term></item>
		///			<item><term>Only use even TIA ranges</term><term>3</term></item>
		///		</list> </summary>
		NT_OddOrEven changeToOddOrEven;
		/// <summary> TIA Range to use if in manual mode: <br />
		/// 		  <see cref="NT_TIARange" /> for TIA ranges. </summary>
		NT_TIARange newRange;
	};

	/// <summary> Structure containing long pass filter parameters. </summary>
	/// <value> Options for controlling long pass filters. </value>
	struct NT_LowPassFilterParameters
	{
		/// <summary> Filter 1 Low Pass Filter
		/// 	<list type=table>
		///			<item><term>No Filter.</term><term>0</term></item>
		///			<item><term>1 Hz Filter.</term><term>1</term></item>
		///			<item><term>3 Hz Filter.</term><term>2</term></item>
		///			<item><term>10 Hz Filter</term><term>3</term></item>
		///			<item><term>30 Hz Filter.</term><term>4</term></item>
		///			<item><term>100 Hz Filter.</term><term>5</term></item>
		///		</list> </summary>
		NT_LowPassFrequency param1;
		/// <summary> Currently not used because there is only one filter controlled. </summary>
		NT_LowPassFrequency param2;
		/// <summary> Currently not used. </summary>
		NT_LowPassFrequency param3;
		/// <summary> Currently not used. </summary>
		NT_LowPassFrequency param4;
		/// <summary> Currently not used. </summary>
		NT_LowPassFrequency param5;
	};

	/// <summary> Structure containing TIA reading. </summary>
	struct NT_TIAReading
	{
		/// <summary> Absolute reading. </summary>
		float absoluteReading;
		/// <summary> Relative reading 0-32767 for 0 - 100% of selected current range value. </summary>
		WORD relativeReading;
		/// <summary> Selected current range from the <see cref="NT_TIARange" />. </summary>
		NT_TIARange selectedRange;
		/// <summary> Indicates whether read signal is in range, over range or under range. </summary>
		NT_UnderOrOver underOrOverRead;
	};


	/// <summary> Structure containing electronic input and output settings. </summary>
	struct NT_IOSettings
	{
		/// <summary> Low Voltage Output range: <see cref="NT_VoltageRange" /> <list type=table>
		///			<item><term>0 - 5V output range.</term><term>1</term></item>
		///			<item><term>0 - 10V output range.</term><term>2</term></item>
		///		</list> </summary>
		NT_VoltageRange lowVoltageOutRange;
		/// <summary> Low Voltage Output routing: <see cref="NT_OutputVoltageRoute" /> <list type=table>
		///			<item><term>Output to SMA connector.</term><term>1</term></item>
		///			<item><term>Output to Hub and SMA connector.</term><term>2</term></item>
		///		</list> </summary>
		/// <summary> </summary>
		NT_OutputVoltageRoute lowVoltageOutputRoute;
		/// <summary> Not in use. </summary>
		WORD notYetInUse;
		/// <summary> Not in use. </summary>
		WORD unused;
	};

	/// <summary> structure containing NT Gain parameters. </summary>
	struct NT_GainParameters
	{
		/// <summary> The control mode (Always set to 0x02). </summary>
		WORD controlMode;
		/// <summary> The gain setting, range is 100 to 10000. </summary>
		short gain;
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
	NANOTRAK_API short __cdecl TLI_BuildDeviceList(void);

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
	NANOTRAK_API short __cdecl TLI_GetDeviceListSize();

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
	NANOTRAK_API short __cdecl TLI_GetDeviceList(SAFEARRAY** stringsReceiver);

	/// <summary> Get the contents of the device list which match the supplied typeID. </summary>
	/// <param name="stringsReceiver"> Outputs a SAFEARRAY of strings holding device serial numbers. </param>
	/// <param name="typeID">The typeID of devices to match, see \ref C_DEVICEID_page "Device serial numbers". </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identification.cpp
	/// <seealso cref="TLI_GetDeviceListSize()" />
	/// <seealso cref="TLI_BuildDeviceList()" />
	/// <seealso cref="TLI_GetDeviceList(SAFEARRAY** stringsReceiver)" />
	/// <seealso cref="TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length)" />
	/// <seealso cref="TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer)" />
	/// <seealso cref="TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID)" />
	/// <seealso cref="TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length)" />
	NANOTRAK_API short __cdecl TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);

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
	NANOTRAK_API short __cdecl TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);

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
	NANOTRAK_API short __cdecl TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer);

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
	NANOTRAK_API short __cdecl TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID);

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
	NANOTRAK_API short __cdecl TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length);

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
	NANOTRAK_API short _cdecl TLI_GetDeviceInfo(char const * serialNo, TLI_DeviceInfo *info);

	/// <summary> Open the device for communications. </summary>
	/// <param name="serialNo">	The serial no of the device to be connected. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_connectionN.cpp
	/// <seealso cref="NT_Close(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_Open(char const * serialNo);

	/// <summary> Disconnect and close the device. </summary>
	/// <param name="serialNo">	The serial no of the device to be disconnected. </param>
    /// 		  \include CodeSnippet_connectionN.cpp
	/// <seealso cref="NT_Open(char const * serialNo)" />
	NANOTRAK_API void __cdecl NT_Close(char const * serialNo);

	/// <summary> Sends a command to the device to make it identify iteself. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	NANOTRAK_API void __cdecl NT_Identify(char const * serialNo);

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
	NANOTRAK_API short __cdecl NT_GetHardwareInfo(char const * serialNo, char * modelNo, DWORD sizeOfModelNo, WORD * type, WORD * numChannels, 
													char * notes, DWORD sizeOfNotes, DWORD * firmwareVersion, WORD * hardwareVersion, WORD * modificationState);

	/// <summary> Gets the hardware information in a block. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="hardwareInfo"> Address of a TLI_HardwareInformation structure to receive the hardware information. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	NANOTRAK_API short __cdecl NT_GetHardwareInfoBlock(char const *serialNo, TLI_HardwareInformation *hardwareInfo);

	/// <summary> Gets version number of the device firmware. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device firmware version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	NANOTRAK_API DWORD __cdecl NT_GetFirmwareVersion(char const * serialNo);

	/// <summary> Gets version number of the device software. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device software version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	NANOTRAK_API DWORD __cdecl NT_GetSoftwareVersion(char const * serialNo);

	/// <summary> Update device with stored settings. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
    /// 		  \include CodeSnippet_connection1.cpp
	NANOTRAK_API bool __cdecl NT_LoadSettings(char const * serialNo);

	/// <summary> Tells the device that it is being disconnected. </summary>
	/// <remarks> This does not disconnect the communications.<br />
	/// 		  To disconnect the communications, call the <see cref="NT_Close(char const * serialNo)" /> function. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	NANOTRAK_API short __cdecl NT_Disconnect(char const * serialNo);

	/// <summary> Gets the NanoTrak signal state. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> <list type=table>
	///				<item><term>Not tracking or poor signal</term><term>0</term></item>
	///				<item><term>Tracking and good signal</term><term>1</term></item>
	/// 		  </list> </returns>
	NANOTRAK_API NT_SignalState __cdecl NT_GetSignalState(char const * serialNo);

	/// <summary> Gets the nanoTrak operating mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The NanoTrak operating mode.
	/// 		  <list type=table>
	///				<item><term>Undefined</term><term>0</term></item>
	///				<item><term>Piezo Mode</term><term>1</term></item>
	///				<item><term>Latched output mode</term><term>2</term></item>
	///				<item><term>Tracking mode</term><term>3</term></item>
	///				<item><term>Horizontal tracking mode</term><term>4</term></item>
	///				<item><term>Vertical Tracking Mode</term><term>5</term></item>
	/// 		  </list> </returns>
	/// <seealso cref="NT_SetMode(char const * serialNo, NT_Mode mode)" />
	NANOTRAK_API NT_Mode __cdecl NT_GetMode(char const * serialNo);

	/// <summary> Setsthe nanoTrak operating mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="mode"> The NanoTrak operating mode.
	/// 		<list type=table>
	///				<item><term>Undefined</term><term>0</term></item>
	///				<item><term>Piezo Mode</term><term>1</term></item>
	///				<item><term>Latched output mode</term><term>2</term></item>
	///				<item><term>Tracking mode</term><term>3</term></item>
	///				<item><term>Horizontal tracking mode</term><term>4</term></item>
	///				<item><term>Vertical Tracking Mode</term><term>5</term></item>
	/// 		  </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetMode(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_SetMode(char const * serialNo, NT_Mode mode);

	/// <summary> Gets the tracking threshold signal. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> Tracking threshold signal in amps from 1.0e-9 to 1.0e-3. </returns>
	/// <seealso cref="NT_SetTrackingThresholdSignal(char const * serialNo, float threshold)" />
	NANOTRAK_API float __cdecl NT_GetTrackingThresholdSignal(char const * serialNo);

	/// <summary> Sets the tracking threshold signal. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="threshold"> Tracking threshold signal in amps from 1.0e-9 to 1.0e-3. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetTrackingThresholdSignal(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_SetTrackingThresholdSignal(char const * serialNo, float threshold);

	/// <summary> Gets the home position of the scan circle. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="position"> Address of the NT_HVComponent to receive the position as an XY pair,<br />
	/// 				   range 0 to 65535 equivalent to 0 to 10 NT Units. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetCircleHomePosition(char const * serialNo, NT_HVComponent *position)" />
	/// <seealso cref="NT_HomeCircle(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_GetCircleHomePosition(char const * serialNo, NT_HVComponent *position);

	/// <summary> Sets the home position of the scan circle. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="position"> Address of the NT_HVComponent containing the new position as an XY pair,<br />
	/// 				   range 0 to 65535 equivalent to 0 to 10 NT Units. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetCircleHomePosition(char const * serialNo, NT_HVComponent *position)" />
	/// <seealso cref="NT_HomeCircle(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_SetCircleHomePosition(char const * serialNo, NT_HVComponent *position);

	/// <summary> Move the scan circle to the home position. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetCircleHomePosition(char const * serialNo, NT_HVComponent *position)" />
	/// <seealso cref="NT_GetCircleHomePosition(char const * serialNo, NT_HVComponent *position)" />
	NANOTRAK_API short __cdecl NT_HomeCircle(char const * serialNo);

	/// <summary>Requests the current scan circle centre position. </summary>
	/// <remarks> This command will request an update from the device.<br />
	/// 		  NOTE this is nor required if the device is being continously polled using <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_GetCirclePosition(char const * serialNo, NT_HVComponent *position)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_RequestCirclePosition(char const * serialNo);

	/// <summary> Gets the current scan circle centre position. </summary>
	/// <remarks> This command will return the latest value received from the device.<br />
	///			  To get an updated value use <see cref="NT_RequestCirclePosition(char const * serialNo)" /> or <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" /></remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="position"> Address of the NT_HVComponent to receive the position as an XY pair,<br />
	/// 				   range 0 to 65535 equivalent to 0 to 10 NT Units. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_RequestCirclePosition(char const * serialNo)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_GetCirclePosition(char const * serialNo, NT_HVComponent *position);

	/// <summary> Gets the scanning circle parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_CircleParameters to receive the parameters defining the scan circle. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameter(char const * serialNo)" />
	/// <seealso cref="NT_SetCircleDiameter(char const * serialNo, WORD diameter)" />
	/// <seealso cref="NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	NANOTRAK_API short __cdecl NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params);

	/// <summary> Sets the scanning circle parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_CircleParameters containing the new parameters defining the scan circle. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameter(char const * serialNo)" />
	/// <seealso cref="NT_SetCircleDiameter(char const * serialNo, WORD diameter)" />
	/// <seealso cref="NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	NANOTRAK_API short __cdecl NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params);

	/// <summary> Gets the scan circle diameter. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The scanning circle diameter, range 0 to 65535 (0 to 10 NT Units). </returns>
	/// <seealso cref="NT_SetCircleDiameter(char const * serialNo, WORD diameter)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	NANOTRAK_API WORD __cdecl NT_GetCircleDiameter(char const * serialNo);

	/// <summary> Sets the scan circle diameter. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="diameter"> The scanning circle diameter, range 0 to 65535 (0 to 10 NT Units). </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetCircleDiameter(char const * serialNo)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	NANOTRAK_API short __cdecl NT_SetCircleDiameter(char const * serialNo, WORD diameter);

	/// <summary> Gets the scan circle diameter Lookup Table (LUT). </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="LUT"> Address of the NT_CircleDiameterLUT to receive the  scan circle diameter lookup table. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameter(char const * serialNo)" />
	/// <seealso cref="NT_SetCircleDiameter(char const * serialNo, WORD diameter)" />
	NANOTRAK_API short __cdecl NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT);

	/// <summary> Sets the scan circle diameter Lookup Table (LUT). </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="LUT"> Address of the NT_CircleParameters containing the new scan circle diameter lookup table. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleDiameter(char const * serialNo)" />
	/// <seealso cref="NT_SetCircleDiameter(char const * serialNo, WORD diameter)" />
	NANOTRAK_API short __cdecl NT_SetCircleDiameterLUT(char const * serialNo, NT_CircleDiameterLUT *LUT);

	/// <summary> Gets the phase compensation parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_HVComponent to receive the  phase compensation,<br />
	/// 				   range 0 to 65535, value = phase(degrees) * OscillatingFrequency / 360. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetPhaseCompensationParams(char const * serialNo, NT_HVComponent *params)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	NANOTRAK_API short __cdecl NT_GetPhaseCompensationParams(char const * serialNo, NT_HVComponent *params);

	/// <summary> Sets the phase compensation parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_HVComponent containing the new phase compensation<br />
	/// 				   range 0 to 65535, value = phase(degrees) * OscillatingFrequency / 360. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetPhaseCompensationParams(char const * serialNo, NT_HVComponent *params)" />
	/// <seealso cref="NT_SetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	/// <seealso cref="NT_GetCircleParams(char const * serialNo, NT_CircleParameters *params)" />
	NANOTRAK_API short __cdecl NT_SetPhaseCompensationParams(char const * serialNo, NT_HVComponent *params);

	/// <summary> Gets the TIA range parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_TIARangeParameters to receive the  TIA range parameters. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_SetRangeMode(char const * serialNo, NT_TIARangeMode mode, NT_OddOrEven oddOrEven)" />
	/// <seealso cref="NT_SetTIARange(char const * serialNo, NT_TIARange range)" />
	NANOTRAK_API short __cdecl NT_GetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params);

	/// <summary> Sets the TIA range parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_TIARangeParameters containing the new TIA range parameters. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_SetRangeMode(char const * serialNo, NT_TIARangeMode mode, NT_OddOrEven oddOrEven)" />
	/// <seealso cref="NT_SetTIARange(char const * serialNo, NT_TIARange range)" />
	NANOTRAK_API short __cdecl NT_SetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params);

	/// <summary> Set the TIA Range Mode and OddEven mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="mode">	The TIA range mode.
	/// 					<list type=table>
	///							<item><term>Automatic Ranging at current range.</term><term>1</term></item>
	///							<item><term>Manual Ranging at current range.</term><term>2</term></item>
	///							<item><term>Manual Ranging using supplied range</term><term>3</term></item>
	///							<item><term>Automatic ranging using supplied range</term><term>4</term></item>
	/// 					</list> </param>
	/// <param name="oddOrEven"> The TIA odd or even state.
	/// 					<list type=table>
	///							<item><term>All TIA Ranges.</term><term>1</term></item>
	///							<item><term>Only use Odd TIA ranges.</term><term>2</term></item>
	///							<item><term>Only use even TIA ranges</term><term>3</term></item>
	/// 					</list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_GetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_SetTIARange(char const * serialNo, NT_TIARange range)" />
	NANOTRAK_API short __cdecl NT_SetRangeMode(char const * serialNo, NT_TIARangeMode mode, NT_OddOrEven oddOrEven);
	
	/// <summary> Sets TIA range. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="range">	The TIA range.
	/// 					<list type=table>
	///								<item><term>0-3nA range.</term><term>3</term></item>
	///								<item><term>0-10nA range.</term><term>4</term></item>
	///								<item><term>0-30nA range.</term><term>5</term></item>
	///								<item><term>0-100nA range.</term><term>6</term></item>
	///								<item><term>0-300nA range.</term><term>7</term></item>
	///								<item><term>0-1uA range.</term><term>8</term></item>
	///								<item><term>0-3uA range.</term><term>9</term></item>
	///								<item><term>0-10uA range.</term><term>10</term></item>
	///								<item><term>0-30uA range.</term><term>11</term></item>
	///								<item><term>0-100uA range.</term><term>12</term></item>
	///								<item><term>0-300uA range.</term><term>13</term></item>
	///								<item><term>0-1mA range.</term><term>14</term></item>
	///								<item><term>0-3mA range.</term><term>15</term></item>
	///								<item><term>0-10mA range.</term><term>16</term></item>
	///							  </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_GetTIArangeParams(char const * serialNo, NT_TIARangeParameters *params)" />
	/// <seealso cref="NT_SetTIARangeMode(char const * serialNo, NT_TIARangeMode mode, NT_OddOrEven oddOrEven)" />
	NANOTRAK_API short __cdecl NT_SetTIARange(char const * serialNo, NT_TIARange range);

	/// <summary> Gets the control loop gain. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The gain value, range 100 to 10000. Typically ~600. </returns>
	/// <seealso cref="NT_SetGain(char const * serialNo, short gain)" />
	NANOTRAK_API short __cdecl NT_GetGain(char const * serialNo);

	/// <summary> Sets the control loop gain. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="gain"> The new Gain value, range 100 to 10000. Typically ~600.. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetGain(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_SetGain(char const * serialNo, short gain);

	/// <summary> Gets the TIA long pass filter parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_LowPassFilterParameters to receive the TIA long pass filter. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetTIALPFilterParams(char const * serialNo, NT_LowPassFilterParameters *params)" />
	NANOTRAK_API short __cdecl NT_GetTIALPFilterParams(char const * serialNo, NT_LowPassFilterParameters *params);

	/// <summary> Sets the TIA long pass filter parameters. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="params"> Address of the NT_LowPassFilterParameters containing the new TIA long pass filter. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetTIALPFilterParams(char const * serialNo, NT_LowPassFilterParameters *params)" />
	NANOTRAK_API short __cdecl NT_SetTIALPFilterParams(char const * serialNo, NT_LowPassFilterParameters *params);

	/// <summary> Gets the NanoTrak feedback source. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device feedback source
	/// 					<list type=table>
	///							<item><term>TIA value</term><term>1</term></item>
	///							<item><term>BNC 1v range</term><term>2</term></item>
	///							<item><term>BNC 2v range</term><term>3</term></item>
	///							<item><term>BNC 5v range</term><term>4</term></item>
	///							<item><term>BNC 10v range</term><term>5</term></item>
	/// 					 </list> </returns>
	/// <seealso cref="NT_SetFeedbackSource(char const * serialNo, NT_FeedbackSource input)" />
	NANOTRAK_API NT_FeedbackSource __cdecl NT_GetFeedbackSource(char const * serialNo);

	/// <summary> Sets the NanoTrak feedback source. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="input"> The device feedback source.
	/// 					 <list type=table>
	///							<item><term>TIA value</term><term>1</term></item>
	///							<item><term>BNC 1v range</term><term>2</term></item>
	///							<item><term>BNC 2v range</term><term>3</term></item>
	///							<item><term>BNC 5v range</term><term>4</term></item>
	///							<item><term>BNC 10v range</term><term>5</term></item>
	/// 					 </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetFeedbackSource(char const * serialNo)" />
	NANOTRAK_API short __cdecl NT_SetFeedbackSource(char const * serialNo, NT_FeedbackSource input);
	
	/// <summary> clears the message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <seealso cref="NT_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="NT_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="NT_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	/// <seealso cref="NT_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	NANOTRAK_API void __cdecl NT_ClearMessageQueue(char const * serialNo);

	/// <summary> Registers a callback on the message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="functionPointer"> A function pointer to be called whenever messages are received. </param>
	/// <seealso cref="NT_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="NT_ClearMessageQueue(char const * serialNo)" />
	/// <seealso cref="NT_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	/// <seealso cref="NT_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	NANOTRAK_API void __cdecl NT_RegisterMessageCallback(char const * serialNo, void (* functionPointer)());

	/// <summary> Gets the MessageQueue size. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> number of messages in the queue. </returns>
	/// <seealso cref="NT_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="NT_ClearMessageQueue(char const * serialNo)" />
	/// <seealso cref="NT_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	/// <seealso cref="NT_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	NANOTRAK_API int __cdecl NT_MessageQueueSize(char const * serialNo);

	/// <summary> Get the next MessageQueue item. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> The address of the parameter to receive the message Type. </param>
	/// <param name="messageID">   The address of the parameter to receive the message id. </param>
	/// <param name="messageData"> The address of the parameter to receive the message data. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="NT_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="NT_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="NT_ClearMessageQueue(char const * serialNo)" />
	/// <seealso cref="NT_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	NANOTRAK_API bool __cdecl NT_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary> Wait for next MessageQueue item. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> The address of the parameter to receive the message Type. </param>
	/// <param name="messageID">   The address of the parameter to receive the message id. </param>
	/// <param name="messageData"> The address of the parameter to receive the message data. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="NT_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="NT_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="NT_ClearMessageQueue(char const * serialNo)" />
	/// <seealso cref="NT_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	NANOTRAK_API bool __cdecl NT_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary>	Requests a TIA reading. </summary>
	/// <remarks> This needs to be called to get the device to send it's current reading.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_GetReading(const char * serialNo, NT_TIAReading *reading)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_RequestReading(char const * serialNo);

	/// <summary> Gets a reading. </summary>
	/// <remarks> This returns the latest reading received from the device.<br />
	/// 		  To get new readings, use <see cref="NT_RequestReading(char const * serialNo)" />
	///			  or use <see cref="NT_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />.  </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="reading"> Address of the NT_TIAReading to recieve the TIA data. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_RequestReading(char const * serialNo)" />
	/// <seealso cref="NT_RequestStatus(char const * serialNo)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_GetReading(const char * serialNo, NT_TIAReading *reading);

	/// <summary> Requests the status bits and reading. </summary>
	/// <remarks> This needs to be called to get the device to send it's current status.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_RequestReading(char const * serialNo)" />
	/// <seealso cref="NT_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="NT_GetReading(const char * serialNo, NT_TIAReading *reading)" />
	/// <seealso cref="NT_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_RequestStatus(char const * serialNo);

	/// <summary> Request the status bits which identify the current device state. </summary>
	/// <remarks> This needs to be called to get the device to send it's current status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API short __cdecl NT_RequestStatusBits(char const * serialNo);

	/// <summary>Get the current status bits. </summary>
	/// <remarks> This returns the latest status bits received from the device.<br />
	/// 		  To get new status bits, use <see cref="NT_RequestStatusBits(char const * serialNo)" />
	///			  or use <see cref="NT_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="NT_StartPolling(char const * serialNo, int milliseconds)" />.  </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns>	The status bits from the device <list type=table>
	///				<item><term>0x00000001</term><term>Tracking (1=Tracking, 0=Latched).</term></item>
	///				<item><term>0x00000002</term><term>Tracking with Signal (1=With Signal, 0=No Signal).</term></item>
	///				<item><term>0x00000004</term><term>Tracking Channel A (1=Chan A Only, 0=Both Channels).</term></item>
	///				<item><term>0x00000008</term><term>Tracking Channel B (1=Chan B Only, 0=Both Channels).</term></item>
	///				<item><term>0x00000010</term><term>Auto-ranging (1=Auto Ranging, 0=Manual Ranging).</term></item>
	///				<item><term>0x00000020</term><term>Under Read (1=Under Reading, 0=Reading Within Range).</term></item>
	///				<item><term>0x00000040</term><term>Over Read (1=Over Reading, 0=Reading Within Range).</term></item>
	///				<item><term>0x00000100</term><term>For Future Use.</term></item>
	///				<item><term>0x00000200</term><term></term></item>
	///				<item><term>...</term><term></term></item>
	///				<item><term>0x00008000</term><term></term></item>
	///				<item><term>0x00010000</term><term>Channel A Connected (1=Connected, 0=Not Connected).</term></item>
	///				<item><term>0x00020000</term><term>Channel B Connected (1=Connected, 0=Not Connected).</term></item>
	///				<item><term>0x00040000</term><term>Channel A Enabled (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00080000</term><term>Channel B Enabled (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00100000</term><term>Channel A Control Mode (1=Closed Loop, 0=Open Loop) or Digital input 1 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00200000</term><term>Channel B Control Mode (1=Closed Loop, 0=Open Loop) or Digital input 2 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00400000</term><term>Digital input 3 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x00800000</term><term>Digital input 4 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x01000000</term><term>Digital input 5 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x02000000</term><term>Digital input 6 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x04000000</term><term>Digital input 7 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x08000000</term><term>Digital input 8 state (1=Logic High, 0=Logic Low).</term></item>
	///				<item><term>0x10000000</term><term>Power OK (1=Indicates OK, 0=Not OK).</term></item>
	///				<item><term>0x20000000</term><term>Active (1=Indicates Unit Is Active, 0=Not Active).</term></item>
	///				<item><term>0x40000000</term><term>Error (1=Indicates Error, 0=No Error).</term></item>
	///				<item><term>0x80000000</term><term>For Future Use.</term></item>
	/// 		  </list> <remarks> Bits 21 to 28 (Digital Input States) are only applicable if the associated digital input is fitted to your controller - see the relevant handbook for more details. </remarks> </returns>
	/// <seealso cref="NT_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="NT_RequestStatus(char const * serialNo)" />
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	NANOTRAK_API DWORD __cdecl NT_GetStatusBits(char const * serialNo);

	/// <summary> Starts the internal polling loop which continuously requests position and status. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="milliseconds">The milliseconds polling rate. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="NT_StopPolling(char const * serialNo)" />
	/// <seealso cref="NT_PollingDuration(char const * serialNo)" />
	/// <seealso cref="NT_RequestStatusBits(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	NANOTRAK_API bool __cdecl NT_StartPolling(char const * serialNo, int milliseconds);

	/// <summary> Gets the polling loop duration. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The time between polls in milliseconds or 0 if polling is not active. </returns>
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="NT_StopPolling(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	NANOTRAK_API long __cdecl NT_PollingDuration(char const * serialNo);

	/// <summary> Stops the internal polling loop. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <seealso cref="NT_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="NT_PollingDuration(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	NANOTRAK_API void __cdecl NT_StopPolling(char const * serialNo);

	/// <summary> Requests that all settings are download from device. </summary>
	/// <remarks> This function requests that the device upload all it's settings to the DLL.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	NANOTRAK_API short __cdecl NT_RequestSettings(char const * serialNo);

	/// <summary> Request the channel states from the device. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_ChannelEnable(char const * serialNo, WORD channel, bool enable)" />
	/// <seealso cref="NT_IsChannelEnabled(char const * serialNo, WORD channel)" />
	NANOTRAK_API short __cdecl NT_RequestChannelStates(char const * serialNo);

	/// <summary> Enable / Disable the specified channel. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel"> The channel to enable / disable. </param>
	/// <param name="enable"> The new channel enabled state, <c>true</c> to enable,  <c>false</c> to disable. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_RequestChannelStates(char const * serialNo)" />
	/// <seealso cref="NT_IsChannelEnabled(char const * serialNo, WORD channel)" />
	NANOTRAK_API short __cdecl NT_ChannelEnable(char const * serialNo, WORD channel, bool enable);

	/// <summary> Get the channel enabled state. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel"> The channel to be queried. </param>
	/// <returns> The channel state, <c>true</c> if enabled,  <c>false</c> if disabled. </returns>
	/// <seealso cref="NT_ChannelEnable(char const * serialNo, WORD channel, bool enable)" />
	/// <seealso cref="NT_RequestChannelStates(char const * serialNo)" />
	NANOTRAK_API bool __cdecl NT_IsChannelEnabled(char const * serialNo, WORD channel);

	/// <summary> Request the NanoTrak control mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_SetControlMode(char const * serialNo, WORD channel, NT_ControlMode mode)" />
	/// <seealso cref="NT_GetControlMode(char const * serialNo, WORD channel)" />
	NANOTRAK_API short __cdecl NT_RequestControlMode(char const * serialNo);

	/// <summary> Set the NanoTrak control mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel 1 or 2. </param>
	/// <param name="mode">	    The operating mode.
	/// 					<list type=table>
	///							<item><term>Open loop mode</term><term>1</term></item>
	///							<item><term>Closed loop mode</term><term>2</term></item>
	///							<item><term>Open loop (Smoothed) mode</term><term>3</term></item>
	///							<item><term>Closed loop (Smoothed) mode</term><term>4</term></item>
	/// 					 </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_RequestControlMode(char const * serialNo)"/>
	/// <seealso cref="NT_GetControlMode(char const * serialNo, WORD channel)"/>
	NANOTRAK_API short __cdecl NT_SetControlMode(char const * serialNo, WORD channel, NT_ControlMode mode);

	/// <summary> Get the NanoTrak control mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The NanoTrak control mode.
	/// 					 <list type=table>
	///							<item><term>Open loop mode</term><term>1</term></item>
	///							<item><term>Closed loop mode</term><term>2</term></item>
	///							<item><term>Open loop (Smoothed) mode</term><term>3</term></item>
	///							<item><term>Closed loop (Smoothed) mode</term><term>4</term></item>
	/// 					 </list> </returns>
	/// <seealso cref="NT_RequestControlMode(char const * serialNo)"/>
	/// <seealso cref="NT_SetControlMode(char const * serialNo, WORD channel, NT_ControlMode mode)"/>
	NANOTRAK_API NT_ControlMode __cdecl NT_GetControlMode(char const * serialNo, WORD channel);

	/// <summary> Sets the NanoTrak channels to (usually) piezos. </summary>
	/// <param name="chanA"> The number of channel A. </param>
	/// <param name="chanB"> The number of channel B. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_RequestNTChannels()" />
	/// <seealso cref="NT_GetNTChannels(short &chanA, short &chanB)" />
	NANOTRAK_API short __cdecl  NT_SetNTChannels(short chanA, short chanB);

	/// <summary> Request the device updates the NanoTrak channel numbers. </summary>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="NT_SetNTChannels(short chanA, short chanB)" />
	/// <seealso cref="NT_GetNTChannels(short &chanA, short &chanB)" />
	NANOTRAK_API short __cdecl NT_RequestNTChannels();

	/// <summary> Gets the NanoTrak channels to (usually) piezos. </summary>
	/// <param name="chanA"> The number of channel A. </param>
	/// <param name="chanB"> The number of channel B. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetNTChannels(short chanA, short chanB)" />
	/// <seealso cref="NT_RequestNTChannels()" />
	NANOTRAK_API short __cdecl NT_GetNTChannels(short &chanA, short &chanB);

	/// <summary> Gets the MaxTravel for the Piezos in um. </summary>
	/// <param name="chanA"> The MaxTravel of channel A in um. </param>
	/// <param name="chanB"> The MaxTravel of channel B in um. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	NANOTRAK_API short __cdecl NT_GetMaxTravel(double &chanA, double &chanB);

	/// <summary> Gets the input/output settings in a block. </summary>
	/// <param name="serialNo">   The device serial no. </param>
	/// <param name="IOsettings"> Address of the BNT_IO_Settings to receive the input/output settings. </param>
	/// <param name="channel">    The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_SetIOsettingsBlock(const char * serialNo, BNT_IO_Settings *IOsettings)"/>
	NANOTRAK_API short __cdecl NT_GetIOsettingsBlock(const char * serialNo, BNT_IO_Settings *IOsettings, WORD channel);

	/// <summary> Sets the input/output options in a block. </summary>
	/// <param name="serialNo">   The device serial no. </param>
	/// <param name="IOsettings"> Address of the BNT_IO_Settings containing the new input/output
	/// 						  settings.. </param>
	/// <param name="channel">    The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="NT_GetIOsettingsBlock(const char * serialNo, BNT_IO_Settings *IOsettings)"/>
	NANOTRAK_API short __cdecl NT_SetIOsettingsBlock(const char * serialNo, BNT_IO_Settings *IOsettings, WORD channel);
}
/** @} */ // BenchtopNanoTrak
