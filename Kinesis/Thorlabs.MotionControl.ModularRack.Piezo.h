// summary:	Declares the brushless motor class
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the MODULARRACKDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// MODULARRACK_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef MODULARRACKDLL_EXPORTS
#define MODULARRACK_API __declspec(dllexport)
#else
#define MODULARRACK_API __declspec(dllimport)
#endif

#include <OaIdl.h>

#pragma once

#include "Thorlabs.MotionControl.ModularRack.h"

/** @defgroup ModularPiezo Modular Piezo
 *  This section details the Structures and Functions relavent to the  @ref MPZ601_page "Modular Piezo"<br />
 *  For an example of how to connect to the device and perform simple operations use the following links:
 *  <list type=bullet>
 *    <item> \ref namespaces_mpz_ex_1 "Example of using the Thorlabs.MotionControl.ModularRack.DLL for a Piezo from a C or C++ project."<br />
 *									  This requires the DLL to be dynamical linked. </item>
 *    <item> \ref namespaces_mpz_ex_2 "Example of using the Thorlabs.MotionControl.ModularRack.DLL for a Piezo from a C# project"<br />
 *									  This uses Marshalling to load and access the C DLL. </item>
 *  </list>
 *  The Thorlabs.MotionControl.ModularRack.DLL requires the following DLLs
 *  <list type=bullet>
 *    <item> Thorlabs.MotionControl.Benchtop.Piezo. </item>
 *    <item> Thorlabs.MotionControl.Benchtop.NanoTrak. </item>
 *    <item> Thorlabs.MotionControl.Benchtop.StepperMotor. </item>
 *    <item> Thorlabs.MotionControl.DeviceManager. </item>
 *  </list>
 *  @{
 */

extern "C"
{
	/// \cond NOT_MASTER

	/// <summary> The Piezo Control Modes. </summary>
	/// \ingroup Common
	enum PZ_ControlModeTypes : short
	{
		Undefined = 0, ///<Undefined
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

	/// \endcond

	#pragma pack(1)

	/// <summary> Structure containing feedback loop constants. </summary>
	struct PZ_FeedbackLoopConstants
	{
		/// <summary> The proportional term. </summary>
		short proportionalTerm;
		/// <summary> The integral term. </summary>
		short integralTerm;
	};

	/// <summary> Structure containing LUT output wave parameters. </summary>
	struct PZ_LUTWaveParameters
	{
		/// <summary> Specifies the LUT (waveform) output mode (continuous or fixed) </summary>
		PZ_OutputLUTModes mode;
		/// <summary> Specifies the number of LUT values to output for a single waveform cycle (0 to 7999) </summary>
		short cycleLength;
		/// <summary> Specifies the number of waveform cycles to output. </summary>
		unsigned int numCycles;
		/// <summary> Specifies the delay in milliseconds that the system waits after setting each LUT
		/// output value. </summary>
		unsigned int LUTValueDelay;
		/// <summary> The delay time before the system clocks out the LUT values. </summary>
		unsigned int preCycleDelay;
		/// <summary> The delay time after the system clocks out the LUT values. </summary>
		unsigned int postCycleDelay;
		/// <summary> Zero-based index at which the output trigger is fired.<br />
		/// 		  Requires Triggering is enabled. </summary>
		short outTriggerStart;
		/// <summary> Duration of the output trigger in ms.<br />
		/// 		  Requires Triggering is enabled. </summary>
		unsigned int outTriggerDuration;
		/// <summary> The output trigger repeat interval.<br />
		/// 		  Requires Repeat Triggering is enabled </summary>
		short numOutTriggerRepeat;
	};

	/// <summary> Structure containing slew rates. </summary>
	struct PZ_SlewRates
	{
		/// <summary> The open loop slew rate. </summary>
		short openModeRate;
		/// <summary> The closed loop slew rate. </summary>
		short closedModeRate;
	};

	#pragma pack()

	/// <summary> Requests the state quantities. </summary>
	/// <remarks> This needs to be called to get the device to send it's current position.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PBC_RequestStatusBits(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_GetPosition(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_GetStatusBits(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	MODULARRACK_API short __cdecl PBC_RequestStatus(char const * serialNo, short channel);

	/// <summary>Get the current status bits. </summary>
	/// <remarks> This returns the latest status bits received from the device.<br />
	/// 		  To get new status bits, use <see cref="PBC_RequestStatusBits(char const * serialNo, short channel)" /> <br />
	///			  or use <see cref="PBC_RequestStatus(char const * serialNo)" />
	/// 		  or use the polling functions, <see cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />. </remarks>
	/// <param name="channel">  The channel. </param>
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
	/// <seealso cref="PBC_RequestStatusBits(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_RequestStatus(char const * serialNo)" />
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	MODULARRACK_API DWORD __cdecl PBC_GetStatusBits(char const * serialNo, short channel);

	/// <summary> Request the status bits which identify the current device state. </summary>
	/// <remarks> This needs to be called to get the device to send it's current status bits.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PBC_GetStatusBits(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	MODULARRACK_API short __cdecl PBC_RequestStatusBits(char const * serialNo, short channel);

	/// <summary>	Requests the current output voltage or position depending on current mode. </summary>
	/// <remarks> This needs to be called to get the device to send it's current position.<br />
	/// 		  NOTE this is called automatically if Polling is enabled for the device using <see cref="PBC_StartPolling(char const * serialNo, int milliseconds)" />. </remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	/// <seealso cref="PBC_GetPosition(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	MODULARRACK_API short __cdecl PBC_RequestPosition(char const * serialNo, short channel);

	/// <summary> Starts the internal polling loop which continuously requests position and status. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="milliseconds">	    The milliseconds polling rate. </param>
	/// <returns> <c>true</c> if successful, false if not. </returns>
	/// <seealso cref="PBC_StopPolling(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_PollingDuration(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_RequestStatusBits(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_RequestPosition(char const * serialNo, short channel)" />
	/// \include CodeSnippet_connectionN.cpp
	MODULARRACK_API bool __cdecl PBC_StartPolling(char const * serialNo, short channel, int milliseconds);

	/// <summary> Gets the polling loop duration. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The time between polls in milliseconds or 0 if no polling. </returns>
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	/// <seealso cref="PBC_StopPolling(char const * serialNo, short channel)" />
	/// \include CodeSnippet_connectionN.cpp
	MODULARRACK_API long __cdecl PBC_PollingDuration(char const * serialNo, short channel);

	/// <summary> Stops the internal polling loop. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <seealso cref="PBC_StartPolling(char const * serialNo, short channel, int milliseconds)" />
	/// <seealso cref="PBC_PollingDuration(char const * serialNo, short channel)" />
	/// \include CodeSnippet_connectionN.cpp
	MODULARRACK_API void __cdecl PBC_StopPolling(char const * serialNo, short channel);

	/// <summary> Requests that all settings are download from device. </summary>
	/// <remarks> This function requests that the device upload all it's settings to the DLL.</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully requested. </returns>
	MODULARRACK_API short __cdecl PBC_RequestSettings(char const * serialNo, short channel);

	/// <summary>	Resets all parameters to power-up values. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_ResetParameters(char const * serialNo, short channel);

	/// <summary> Sets the voltage output to zero and defines the ensuing actuator position az zero. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_SetZero(char const * serialNo, short channel);

	/// <summary> Gets the Position Control Mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The control mode <list type=table>
	///				<item><term>Open Loop</term><term>1</term></item>
	///				<item><term>Closed Loop</term><term>2</term></item>
	///				<item><term>Open Loop smoothed</term><term>3</term></item>
	///				<item><term>Closed Loop smoothed</term><term>4</term></item>
	/// 		  </list> </returns>
	/// <seealso cref="PBC_SetPositionControlMode(char const * serialNo, short channel, PZ_ControlModeTypes mode)" />
	MODULARRACK_API PZ_ControlModeTypes __cdecl PBC_GetPositionControlMode(char const * serialNo, short channel);

	/// <summary> Sets the Position Control Mode. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="mode"> The control mode <list type=table>
	///				<item><term>Open Loop</term><term>1</term></item>
	///				<item><term>Closed Loop</term><term>2</term></item>
	///				<item><term>Open Loop smoothed</term><term>3</term></item>
	///				<item><term>Closed Loop smoothed</term><term>4</term></item>
	/// 		  </list>. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetPositionControlMode(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_SetPositionControlMode(char const * serialNo, short channel, PZ_ControlModeTypes mode);

	// Voltage Functions
	
	/// <summary> Gets the maximum output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The maximum output voltage, 750, 1000 or 1500 (75.0, 100.0, 150.0). </returns>
	/// <seealso cref="PBC_SetMaxOutputVoltage(char const * serialNo, short channel, short eVoltage)" />
	MODULARRACK_API short __cdecl PBC_GetMaxOutputVoltage(char const * serialNo, short channel);

	/// <summary> Sets the maximum output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="eVoltage">	The maximum output voltage, 750, 1000 or 1500 (75.0, 100.0, 150.0). </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetMaxOutputVoltage(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_SetMaxOutputVoltage(char const * serialNo, short channel, short eVoltage);

	/// <summary> Gets the set Output Voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The voltage as a percentage of MaxOutputVoltage,<br />
	/// 		  range -32767 to 32767 equivalent to -100% to 100%. </returns>
	/// <seealso cref="PBC_SetOutputVoltage(char const * serialNo, short channel, short volts)" />
	/// <seealso cref="PBC_SetMaxOutputVoltage(char const * serialNo, short channel, short eVoltage)" />
	/// <seealso cref="PBC_GetMaxOutputVoltage(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_GetOutputVoltage(char const * serialNo, short channel);

	/// <summary> Sets the output voltage. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="volts"> The voltage as a percentage of MaxOutputVoltage,<br />
	/// 		  range -32767 to 32767 equivalent to -100% to 100%. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetOutputVoltage(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_SetMaxOutputVoltage(char const * serialNo, short channel, short eVoltage)" />
	/// <seealso cref="PBC_GetMaxOutputVoltage(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_SetOutputVoltage(char const * serialNo, short channel, short volts);

	/// <summary> Gets the control voltage source. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The voltage source. <list type=table>
	///				<item><term>Software Only</term><term>0</term></item>
	///				<item><term>Software and External</term><term>1</term></item>
	///				<item><term>Software and Potentiometer</term><term>2</term></item>
	///				<item><term>Software, External and Potentiometer</term><term>3</term></item>
	/// 		  </list> </returns>
	/// <seealso cref="PBC_SetVoltageSource(char const * serialNo, short channel, PZ_InputSourceFlags source)" />
	MODULARRACK_API PZ_InputSourceFlags __cdecl PBC_GetVoltageSource(char const * serialNo, short channel);

	/// <summary> Sets the control voltage source. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="source"> The voltage source <list type=table>
	///				<item><term>Software Only</term><term>0</term></item>
	///				<item><term>Software and External</term><term>1</term></item>
	///				<item><term>Software and Potentiometer</term><term>2</term></item>
	///				<item><term>Software, External and Potentiometer</term><term>3</term></item>
	/// 		  </list> </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetVoltageSource(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_SetVoltageSource(char const * serialNo, short channel, PZ_InputSourceFlags source);


	/// <summary> Gets the maximum travel of the device. </summary>
	/// <remarks> This requires an actuator with built in position sensing</remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns>	The distance in steps of 100nm,<br />
	/// 			range 0 to 65535 (10000 is equivalent to 1mm). </returns>
	MODULARRACK_API WORD __cdecl PBC_GetMaximumTravel(char const * serialNo, short channel);

	/// <summary> Gets the position when in closed loop mode. </summary>
	/// <remarks> The result is undefined if not in closed loop mode</remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The position as a percentage of maximum travel,<br />
	/// 		  range -32767 to 32767, equivalent to -100 to 100%. </returns>
	/// <seealso cref="PBC_SetPosition(char const * serialNo, short channel, short position)" />
	/// <seealso cref="PBC_SetPositionControlMode(char const * serialNo, short channel, PZ_ControlModeTypes mode)" />
	/// <seealso cref="PBC_GetPositionControlMode(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_GetPosition(char const * serialNo, short channel);

	/// <summary> Sets the position when in closed loop mode. </summary>
	/// <remarks> The command is ignored if not in closed loop mode</remarks>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="position"> The position as a percentage of maximum travel,<br />
	/// 		  range 0 to 32767, equivalent to 0 to 100%. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetPosition(char const * serialNo, short channel)" />
	/// <seealso cref="PBC_SetPositionControlMode(char const * serialNo, short channel, PZ_ControlModeTypes mode)" />
	/// <seealso cref="PBC_GetPositionControlMode(char const * serialNo, short channel)" />
	MODULARRACK_API short __cdecl PBC_SetPosition(char const * serialNo, short channel, short position);

	/// <summary> Gets the feedback loop constants. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="proportionalTerm"> The address of the parameter to receive the proportional parameter. </param>
	/// <param name="integralTerm"> The address of the parameter to receive the integral parameter. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_SetFeedbackLoopPIconsts(char const * serialNo, short channel, short proportionalTerm, short integralTerm)" />
	/// <seealso cref="PBC_GetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants)" />
	/// <seealso cref="PBC_SetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants)" />
	MODULARRACK_API short __cdecl PBC_GetFeedbackLoopPIconsts(char const * serialNo, short channel, short * proportionalTerm, short * integralTerm);

	/// <summary> Sets the feedback loop parameters. </summary>
	/// <param name="serialNo">			 The device serial no. </param>
	/// <param name="channel">			 The channel. </param>
	/// <param name="proportionalTerm"> The proportional gain term from 0 to 255. </param>
	/// <param name="integralTerm">	 The integral gain term from 0 to 255. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetFeedbackLoopPIconsts(char const * serialNo, short channel, short * proportionalTerm, short * integralTerm)"/>
	/// <seealso cref="PBC_SetFeedbackLoopPIconsts(char const * serialNo, short channel, short proportionalTerm, short integralTerm)"/>
	/// <seealso cref="PBC_SetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants)"/>
	MODULARRACK_API short __cdecl PBC_SetFeedbackLoopPIconsts(char const * serialNo, short channel, short proportionalTerm, short integralTerm);

	/// <summary> Gets the feedback loop constants in a block. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="proportionalAndIntegralConstants"> The address of the PZ_FeedbackLoopConstants to receive the feedback loop constants. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetFeedbackLoopPIconsts(char const * serialNo, short channel, short * proportionalTerm, short * integralTerm)" />
	/// <seealso cref="PBC_SetFeedbackLoopPIconsts(char const * serialNo, short channel, short proportionalTerm, short integralTerm)" />
	/// <seealso cref="PBC_SetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants)" />
	MODULARRACK_API short __cdecl PBC_GetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants);

	/// <summary> Sets the feedback loop constants in a block. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="proportionalAndIntegralConstants"> The address of the PZ_FeedbackLoopConstants containing the new feedback loop constants. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	/// <seealso cref="PBC_GetFeedbackLoopPIconsts(char const * serialNo, short channel, short * proportionalTerm, short * integralTerm)" />
	/// <seealso cref="PBC_SetFeedbackLoopPIconsts(char const * serialNo, short channel, short proportionalTerm, short integralTerm)" />
	/// <seealso cref="PBC_GetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants)" />
	MODULARRACK_API short __cdecl PBC_SetFeedbackLoopPIconstsBlock(const char * serialNo, short channel, PZ_FeedbackLoopConstants *proportionalAndIntegralConstants);


	/// <summary> Sets the LUT output wave parameters. </summary>
	/// <remarks> NOTE, the get function has never been implemented in firmware, so new parameters need to be generated each time.</remarks>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="LUTwaveParams"> The address of the PZ_LUTWaveParameters containing the new LUT output wave parameters. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_SetLUTwaveParams(const char * serialNo, short channel, PZ_LUTWaveParameters *LUTwaveParams);

	/// <summary> Sets a waveform sample. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <param name="index"> The LUT waveform index where the value is to be set. </param>
	/// <param name="value"> The voltage or position as a percentage of full scale.<br />
	/// 					  range -32767 to 32767, equivalemnt to -100% to 100% of maximum voltage / position. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_SetLUTwaveSample(char const * serialNo, short channel, short index, WORD value);

	/// <summary> Starts the LUT waveform output. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successfully started. </returns>
	MODULARRACK_API short __cdecl PBC_StartLUTwave(char const * serialNo, short channel);

	/// <summary> Stops the LUT waveform output. </summary>
	/// <param name="serialNo">	The device serial no. </param>
	/// <param name="channel">  The channel. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_StopLUTwave(char const * serialNo, short channel);

	/// <summary> Gets the slew rates. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">    The channel. </param>
	/// <param name="slewRates"> The open mode and closed mode rates. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_GetSlewRates(const char * serialNo, short channel, PZ_SlewRates *slewRates);

	/// <summary> Sets the slew rates. </summary>
	/// <param name="serialNo"> The device serial no. </param>
	/// <param name="channel">    The channel. </param>
	/// <param name="slewRates"> The open mode and closed mode rates. </param>
	/// <returns> The error code (see \ref C_DLL_ERRORCODES_page "Error Codes") or zero if successful. </returns>
	MODULARRACK_API short __cdecl PBC_SetSlewRates(const char * serialNo, short channel, PZ_SlewRates *slewRates);
}
/** @} */ // ModularPiezo
