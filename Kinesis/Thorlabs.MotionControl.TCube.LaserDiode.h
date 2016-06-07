// summary:	Declares the functions class
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the TLASERDIODE_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// TLASERDIODE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef TLASERDIODEDLL_EXPORTS
#define TLASERDIODE_API __declspec(dllexport)
#else
#define TLASERDIODE_API __declspec(dllimport)
#endif

#include <OaIdl.h>

#pragma once

/** @defgroup TCubeLaserDiode TCube LaserDiode
 *  This section details the Structures and Functions relavent to the  @ref TLD001_page "TCube Laser Diode"<br />
 *  For an example of how to connect to the device and perform simple operations use the following links:
 *  <list type=bullet>
 *    <item> \ref namespaces_tls_ex_1 "Example of using the Thorlabs.MotionControl.TCube.LaserDiode.DLL from a C or C++ project."<br />
 *									  This requires the DLL to be dynamical linked. </item>
 *    <item> \ref namespaces_tls_ex_2 "Example of using the Thorlabs.MotionControl.TCube.LaserDiode.DLL from a C# project"<br />
 *									  This uses Marshalling to load and access the C DLL. </item>
 *  </list>
 *  The Thorlabs.MotionControl.TCube.LaserDiode.DLL requires the following DLLs
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
	#pragma pack()

	/// <summary> Values that represent Laser Input Source Flags. </summary>
	enum LD_InputSourceFlags : unsigned short
	{
		LD_SoftwareOnly = 0x01,///< Input is read from software only
		LD_ExternalSignal = 0x02,///<Input is read from software or External signal
		LD_Potentiometer = 0x04,///<Input is read from software or Potentiometer
	};

	/// <summary> Values that represent Laser Display Units. </summary>
	enum LD_DisplayUnits : unsigned short
	{
		LD_ILim = 0x01,///<Dislay shows the Max Laser Diode Current in mAmps
		LD_ILD = 0x02,///<Dislay shows the Laser Diode Current in mAmps
		LD_IPD = 0x03,///<Dislay shows the Photodiode current in mA
		LD_PLD = 0x04,///<Dislay shows the Ouput Power in uW
	};

	/// <summary> Values that represent LD_TIA_RANGES. </summary>
	enum LD_TIA_RANGES
	{
		LD_TIA_10uA = 1,///<The device TIA range is set to 10uA
		LD_TIA_100uA = 2,///<The device TIA range is set to 100uA
		LD_TIA_1mA = 4,///<The device TIA range is set to 1mA
		LD_TIA_10mA = 8,///<The device TIA range is set to 10mA
	};

	/// <summary> Values that represent LD_POLARITY. </summary>
	enum LD_POLARITY
	{
		LD_CathodeGrounded = 1,///<Configure the device for Cathode Grounded
		LD_AnodeGrounded = 2,///<Configure the device for Anode Grounded
	};

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
	TLASERDIODE_API short __cdecl TLI_BuildDeviceList(void);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListSize();

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceList(SAFEARRAY** stringsReceiver);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListExt(char *receiveBuffer, DWORD sizeOfBuffer);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListByTypeExt(char *receiveBuffer, DWORD sizeOfBuffer, int typeID);

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
	TLASERDIODE_API short __cdecl TLI_GetDeviceListByTypesExt(char *receiveBuffer, DWORD sizeOfBuffer, int * typeIDs, int length);

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
	TLASERDIODE_API short _cdecl TLI_GetDeviceInfo(char const * serialNo, TLI_DeviceInfo *info);

	/// <summary> Open the device for communications. </summary>
	/// <param name="serialNo">	The serial no of the device to be connected. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_connection1.cpp
	/// <seealso cref="LD_Close(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_Open(char const * serialNo);

	/// <summary> Disconnect and close the device. </summary>
	/// <param name="serialNo">	The serial no of the device to be disconnected. </param>
    /// 		  \include CodeSnippet_connection1.cpp
	/// <seealso cref="LD_Open(char const * serialNo)" />
	TLASERDIODE_API void __cdecl LD_Close(char const * serialNo);

	/// <summary> Sends a command to the device to make it identify iteself. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	TLASERDIODE_API void __cdecl LD_Identify(char const * serialNo);

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
	TLASERDIODE_API short __cdecl LD_GetHardwareInfo(char const * serialNo, char * modelNo, DWORD sizeOfModelNo, WORD * type, WORD * numChannels, 
														char * notes, DWORD sizeOfNotes, DWORD * firmwareVersion, WORD * hardwareVersion, WORD * modificationState);

	/// <summary> Gets the hardware information in a block. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="hardwareInfo"> Address of a TLI_HardwareInformation structure to receive the hardware information. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	TLASERDIODE_API short __cdecl LD_GetHardwareInfoBlock(char const *serialNo, TLI_HardwareInformation *hardwareInfo);

	/// <summary> Gets version number of the device firmware. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device firmware version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	TLASERDIODE_API DWORD __cdecl LD_GetFirmwareVersion(char const * serialNo);

	/// <summary> Gets version number of the device software. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The device software version number made up of 4 byte parts. </returns>
    /// 		  \include CodeSnippet_identify.cpp
	TLASERDIODE_API DWORD __cdecl LD_GetSoftwareVersion(char const * serialNo);

	/// <summary> Update device with stored settings. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
    /// 		  \include CodeSnippet_connection1.cpp
	TLASERDIODE_API bool __cdecl LD_LoadSettings(char const * serialNo);

	/// <summary> persist the devices current settings. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	TLASERDIODE_API bool __cdecl LD_PersistSettings(char const * serialNo);

	/// <summary> Disable laser. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_Enable(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_Disable(char const * serialNo);

	/// <summary> Enable laser for computer control. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_Disable(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_Enable(char const * serialNo);

	/// <summary> Clears the device message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	TLASERDIODE_API void __cdecl LD_ClearMessageQueue(char const * serialNo);

	/// <summary> Registers a callback on the message queue. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="functionPointer"> A function pointer to be called whenever messages are received. </param>
	/// <seealso cref="LD_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="LD_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	/// <seealso cref="LD_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	TLASERDIODE_API void __cdecl LD_RegisterMessageCallback(char const * serialNo, void (* functionPointer)());

	/// <summary> Gets the MessageQueue size. </summary>
	/// <remarks> see \ref C_MESSAGES_page "Device Messages" for details on how to use messages. </remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> number of messages in the queue. </returns>
	/// <seealso cref="LD_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="LD_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	/// <seealso cref="LD_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	TLASERDIODE_API int __cdecl LD_MessageQueueSize(char const * serialNo);

	/// <summary> Get the next MessageQueue item. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> The address of the parameter to receive the message Type. </param>
	/// <param name="messageID">   The address of the parameter to receive the message id. </param>
	/// <param name="messageData"> The address of the parameter to receive the message data. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="LD_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="LD_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="LD_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	TLASERDIODE_API bool __cdecl LD_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary> Wait for next MessageQueue item. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="messageType"> The address of the parameter to receive the message Type. </param>
	/// <param name="messageID">   The address of the parameter to receive the message id. </param>
	/// <param name="messageData"> The address of the parameter to receive the message data. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="LD_RegisterMessageCallback(char const * serialNo, void (* functionPointer)())" />
	/// <seealso cref="LD_MessageQueueSize(char const * serialNo)" />
	/// <seealso cref="LD_GetNextMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData)" />
	TLASERDIODE_API bool __cdecl LD_WaitForMessage(char const * serialNo, WORD * messageType, WORD * messageID, DWORD *messageData);

	/// <summary> Set open loop mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_SetClosedLoopMode(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetOpenLoopMode(char const * serialNo);

	/// <summary> Set closed loop mode. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_SetOpenLoopMode(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetClosedLoopMode(char const * serialNo);

	/// <summary> Enables the maximum current adjust mode. </summary>
	/// <param name="serialNo">   The serial no. </param>
	/// <param name="enableAdjust"> true to enable, false to disable the maximum current adjust mode. </param>
	/// <param name="enableDiode">  true to enable, false to disable the laser diode during adjustment. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_SetMaxCurrentDigPot(char const * serialNo, WORD maxCurrent)" />
	/// <seealso cref="LD_GetMaxCurrentDigPot(char const * serialNo)" />
	/// <seealso cref="LD_GetLaserDiodeMaxCurrentLimit(const char *serialNo)" />
	TLASERDIODE_API short __cdecl LD_EnableMaxCurrentAdjust(char const * serialNo, bool enableAdjust, bool enableDiode);

	/// <summary> Sets the maximum current digital pot position. </summary>
	/// <param name="serialNo">   The serial no. </param>
	/// <param name="maxCurrent"> The maximum current, range 20 to 255 equivalent to 17.3 to 220mA. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_EnableMaxCurrentAdjust(char const * serialNo, bool enableAdjust, bool enableDiode)" />
	/// <seealso cref="LD_GetMaxCurrentDigPot(char const * serialNo)" />
	/// <seealso cref="LD_GetLaserDiodeMaxCurrentLimit(const char *serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetMaxCurrentDigPot(char const * serialNo, WORD maxCurrent);

	/// <summary> Gets the maximum current dig pot position. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <returns> The maximum current, range 20 to 255 equivalent to 17.3 to 220mA. </returns>
	/// <seealso cref="LD_EnableMaxCurrentAdjust(char const * serialNo, bool enableAdjust, bool enableDiode)" />
	/// <seealso cref="LD_SetMaxCurrentDigPot(char const * serialNo, WORD maxCurrent)" />
	/// <seealso cref="LD_GetLaserDiodeMaxCurrentLimit(const char *serialNo)" />
	TLASERDIODE_API WORD __cdecl LD_GetMaxCurrentDigPot(char const * serialNo);

	/// <summary> Performs the FindTIAGain calibration. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_EnableTIAGainAdjust(char const * serialNo, bool enable)" />
	TLASERDIODE_API short __cdecl LD_FindTIAGain(char const * serialNo);

	/// <summary> Enables the tia gain adjustment mode. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <param name="enable"> true to enable, false to disable. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_FindTIAGain(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_EnableTIAGainAdjust(char const * serialNo, bool enable);

	/// <summary> Switch laser off. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_EnableOutput(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_DisableOutput(char const * serialNo);

	/// <summary> Switch laser on. </summary>
	/// <remarks> Laser will be enabled only if interlock is in place AND keyswitch is at ON position. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_DisableOutput(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_EnableOutput(char const * serialNo);

	/// <summary> Gets the control input source. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The control input source.
	/// 		  <list type=table>
	///				<item><term>Software only</term><term></term>1</item>
	///				<item><term>External signal</term><term></term>2</item>
	///				<item><term>Potentiometer</term><term></term>4</item>
	/// 		  </list> <remarks> These can be combined with bitwise OR. </remarks> </returns>
	/// <seealso cref="LD_SetControlSource(char const * serialNo, LD_InputSourceFlags source)" />
	TLASERDIODE_API LD_InputSourceFlags __cdecl LD_GetControlSource(char const * serialNo);

	/// <summary> Sets the control input source. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="source"> The control input source.
	/// 		  <list type=table>
	///				<item><term>Software only</term><term></term>1</item>
	///				<item><term>External signal</term><term></term>2</item>
	///				<item><term>Potentiometer</term><term></term>4</item>
	/// 		  </list> <remarks> These can be combined with bitwise OR. </remarks> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetControlSource(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetControlSource(char const * serialNo, LD_InputSourceFlags source);

	/// <summary> Gets the Interlock State. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The interlock state.
	/// 		  <list type=table>
	///				<item><term>Enabled</term><term></term>1</item>
	///				<item><term>Disabled</term><term></term>2</item>
	/// 		  </list> </returns>
	TLASERDIODE_API BYTE __cdecl LD_GetInterlockState(char const * serialNo);

	/// <summary> Gets the hardware display units. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The hardware display units.
	/// 		  <list type=table>
	///				<item><term>Max Current(ILim)</term><term></term>1</item>
	///				<item><term>Laser Diode Current (ILD)</term><term></term>2</item>
	///				<item><term>Photodiode Current (IPD)</term><term></term>3</item>
	///				<item><term>Output Power (PLD)</term><term></term>4</item>
	/// 		  </list> </returns>
	/// <seealso cref="LD_SetDisplayUnits(char const * serialNo, LD_DisplayUnits units)" />
	TLASERDIODE_API LD_DisplayUnits __cdecl LD_GetDisplayUnits(char const * serialNo);

	/// <summary> Sets the hardware display units. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="units"> The hardware display units.
	/// 		  <list type=table>
	///				<item><term>Max Current(ILim)</term><term></term>1</item>
	///				<item><term>Laser Diode Current (ILD)</term><term></term>2</item>
	///				<item><term>Photodiode Current (IPD)</term><term></term>3</item>
	///				<item><term>Output Power (PLD)</term><term></term>4</item>
	/// 		  </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetDisplayUnits(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetDisplayUnits(char const * serialNo, LD_DisplayUnits units);

	/// <summary> Gets the LED brightness. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> Intensity from 0 (off) to 255. </returns>
	/// <seealso cref="LD_SetLEDBrightness(char const * serialNo, short brightness)" />
	TLASERDIODE_API WORD __cdecl LD_GetLEDBrightness(char const * serialNo);

	/// <summary> Sets the LED brightness. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="brightness"> Intensity from 0 (off) to 255. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetLEDBrightness(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetLEDBrightness(char const * serialNo, short brightness);

	/// <summary> Gets the Laser Diode Current currently set. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The Laser Diode Current, range 0 to 32767 representing 0 to 220mA or 0 to TIA_Range_max depending on mode.</returns>
	/// <seealso cref="LD_SetLaserSetPoint(char const * serialNo, WORD laserDiodeCurrent)" />
	TLASERDIODE_API WORD __cdecl LD_GetLaserSetPoint(char const * serialNo);

	/// <summary> Sets the Laser Diode Current. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="laserDiodeCurrent"> range 0 to 32767 representing 0 to 220mA or 0 to TIA_Range_max depending on mode. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetLaserSetPoint(char const * serialNo)" />
	TLASERDIODE_API short __cdecl LD_SetLaserSetPoint(char const * serialNo, WORD laserDiodeCurrent);

	/// <summary> Requests the state quantities (actual power, current and status). </summary>
	/// <remarks> This needs to be called to get the device to send it's current status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_GetPhotoCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_GetLaserDiodeCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API short __cdecl LD_RequestStatus(char const * serialNo);

	/// <summary> Request power and current readings. </summary>
	/// <remarks> This needs to be called to get the device to send it's current reading.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="LD_GetPhotoCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_GetLaserDiodeCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatus(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API short __cdecl LD_RequestReadings(char const * serialNo);

	/// <summary> Request the status bits which identify the current device state. </summary>
	/// <remarks> This needs to be called to get the device to send it's current status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="LD_GetStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatus(char const * serialNo)" />
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API short __cdecl LD_RequestStatusBits(char const * serialNo);

	/// <summary> Gets current Photo Current reading. </summary>
	/// <remarks> This returns the latest photodiode current received from the device.<br />
	/// 		  To get new photodiode current, use <see cref="LD_RequestReadings(char const * serialNo)" />
	///			  or use <see cref="LD_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The current Photodiode Current, range 0 to 32767 representing 0 to TIA range in mA.</returns>
	/// <seealso cref="LD_GetLaserDiodeCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_GetVoltageReading(char const * serialNo)" />
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API WORD __cdecl LD_GetPhotoCurrentReading(char const * serialNo);

	/// <summary> Gets current Voltage reading. </summary>
	/// <remarks> This returns the latest voltage received from the device.<br />
	/// 		  To get new voltage, use <see cref="LD_RequestReadings(char const * serialNo)" />
	///			  or use <see cref="LD_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The current Voltage, range -10000 to 10000 representing +/- 10.0mV.</returns>
	/// <seealso cref="LD_GetPhotoCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_GetLaserDiodeCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API WORD __cdecl LD_GetVoltageReading(char const * serialNo);

	/// <summary> Gets current Current reading. </summary>
	/// <remarks> This returns the latest current received from the device.<br />
	/// 		  To get new current, use <see cref="LD_RequestReadings(char const * serialNo)" />
	///			  or use <see cref="LD_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The current Current, range -32768 to 32767 representing -220 to +220mA.</returns>
	/// <seealso cref="LD_GetPhotoCurrentReading(char const * serialNo)" />
	/// <seealso cref="LD_GetVoltageReading(char const * serialNo)" />
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API WORD __cdecl LD_GetLaserDiodeCurrentReading(char const * serialNo);

	/// <summary> Gets the Laser Diode max current limit. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <returns> The maximum current limit as set by adjusting the max current digital pot, 0 to 32767 representing 0 to 220mA. </returns>
	/// <seealso cref="LD_EnableMaxCurrentAdjust(char const * serialNo, bool enableAdjust, bool enableDiode)" />
	/// <seealso cref="LD_SetMaxCurrentDigPot(char const * serialNo, WORD maxCurrent)" />
	/// <seealso cref="LD_GetMaxCurrentDigPot(char const * serialNo)" />
	TLASERDIODE_API WORD __cdecl LD_GetLaserDiodeMaxCurrentLimit(const char *serialNo);

	/// <summary> Gets the W/A calibibration factor. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <returns> The amps to watts calibibration factor. </returns>
	/// <seealso cref="LD_SetWACalibFactor(char const * serialNo, float calibFactor)"/>
	TLASERDIODE_API float __cdecl LD_GetWACalibFactor(char const * serialNo);

	/// <summary> Sets the W/A calibibration factor. </summary>
	/// <param name="serialNo">    The serial no. </param>
	/// <param name="calibFactor"> The amps to watts calibibration factor. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetWACalibFactor(char const * serialNo)"/>
	TLASERDIODE_API short __cdecl LD_SetWACalibFactor(char const * serialNo, float calibFactor);

	/// <summary> Gets the laser polarity. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <returns> The diode polarity.
	/// 		  <list type=table>
	///				<item><term>Cathode Grounded</term><term></term>1</item>
	///				<item><term>Anode Grounded</term><term></term>2</item>
	/// 		  </list> </returns>
	/// <seealso cref="LD_SetLaserPolarity(char const * serialNo, LD_POLARITY polarity)"/>
	TLASERDIODE_API LD_POLARITY __cdecl LD_GetLaserPolarity(char const * serialNo);

	/// <summary> Sets the laser polarity. </summary>
	/// <param name="serialNo"> The serial no. </param>
	/// <param name="polarity"> The diode polarity.
	/// 		  <list type=table>
	///				<item><term>Cathode Grounded</term><term></term>1</item>
	///				<item><term>Anode Grounded</term><term></term>2</item>
	/// 		  </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="LD_GetLaserPolarity(char const * serialNo)"/>
	TLASERDIODE_API short __cdecl LD_SetLaserPolarity(char const * serialNo, LD_POLARITY polarity);

	/// <summary>Get the current status bits. </summary>
	/// <remarks> This returns the latest status bits received from the device.<br />
	/// 		  To get new status bits, use <see cref="LD_RequestStatusBits(char const * serialNo)" />
	///			  or use <see cref="LD_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="LD_StartPolling(char const * serialNo, int milliseconds)" />.  </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns>	The status bits from the device <list type=table>
	///				<item><term>0x00000001</term><term>Laser output enabled state (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00000002</term><term>Key switch enabled state (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00000004</term><term>Laser control mode (1=Constant P (Closed Loop), 0=Constant I (Open Loop)).</term></item>
	///				<item><term>0x00000008</term><term>Safety interlock enabled state (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00000010</term><term>Transimpedance Amplifier range (1=10&micro;A, 0=Else).</term></item>
	///				<item><term>0x00000020</term><term>Transimpedance Amplifier range (1=100&micro;A, 0=Else).</term></item>
	///				<item><term>0x00000040</term><term>Transimpedance Amplifier range (1=1mA, 0=Else).</term></item>
	///				<item><term>0x00000080</term><term>Transimpedance Amplifier range (1=10mA, 0=Else).</term></item>
	///				<item><term>0x00000100</term><term>Laser Diode Polarity (1=Cathode Grounded, 0=Anode Grounded)</term></item>
	///				<item><term>0x00000200</term><term>External SMA Input enabled (1=Enabled, 0=Disabled).</term></item>
	///				<item><term>0x00000400</term><term>Laser Diode Current Limit Reached (1=Reached, 0=Not Reached).</term></item>
	///				<item><term>0x00000800</term><term>Laser Diode Open Circuit (1=O/C, 0=Else).</term></item>
	///				<item><term>0x00001000</term><term>All PSU Voltages are OK ) (1=OK, 0=Not OK).</term></item>
	///				<item><term>0x00002000</term><term>TIA Range Over Limit</term></item>
	///				<item><term>0x00004000</term><term>TIA Range Under Limit</term></item>
	///				<item><term>0x00008000</term><term>For Future Use</term></item>
	///				<item><term>0x00010000</term><term> </term></item>
	///				<item><term>...</term><term> </term></item>
	///				<item><term>0x80000000</term><term> </term></item>
	/// 		  </list>. </returns>
	/// <seealso cref="LD_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatus(char const * serialNo)" />
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	TLASERDIODE_API DWORD __cdecl LD_GetStatusBits(char const * serialNo);

	/// <summary> Starts the internal polling loop which continuously requests position and status. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="milliseconds">The milliseconds polling rate. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="LD_StopPolling(char const * serialNo)" />
	/// <seealso cref="LD_PollingDuration(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatusBits(char const * serialNo)" />
	/// <seealso cref="LD_RequestStatus(char const * serialNo)" />
	/// <seealso cref="LD_RequestReadings(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	TLASERDIODE_API bool __cdecl LD_StartPolling(char const * serialNo, int milliseconds);

	/// <summary> Gets the polling loop duration. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <returns> The time between polls in milliseconds or 0 if polling is not active. </returns>
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="LD_StopPolling(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	TLASERDIODE_API long __cdecl LD_PollingDuration(char const * serialNo);

	/// <summary> Stops the internal polling loop. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <seealso cref="LD_StartPolling(char const * serialNo, int milliseconds)" />
	/// <seealso cref="LD_PollingDuration(char const * serialNo)" />
	/// \include CodeSnippet_connection1.cpp
	TLASERDIODE_API void __cdecl LD_StopPolling(char const * serialNo);

	/// <summary> Requests that all settings are download from device. </summary>
	/// <remarks> This function requests that the device upload all it's settings to the DLL.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	TLASERDIODE_API short __cdecl LD_RequestSettings(char const * serialNo);
}
/** @} */ // TCubeLaserDiode
