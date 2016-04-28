/////////////////////////////////////////////////////////////////////////////
// This is a part of the PI-Software Sources
// Copyright (C) 1995-2006 Physik Instrumente (PI) GmbH & Co. KG
// All rights reserved.
//
// This source code belongs to the Dll for the E-816 system
//

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32
#undef E816_FUNC_DECLL
#ifdef E816_DLL_EXPORTS
#define E816_FUNC_DECL __declspec(dllexport) WINAPI
#else
#define E816_FUNC_DECL __declspec(dllimport) WINAPI
#endif
		#ifndef pi_uint64
			#define	pi_uint64	unsigned _int64
		#endif
		
		#ifndef pi_uint32
			#define	pi_uint32	unsigned _int32
		#endif

		#ifndef pi_uint16
			#define	pi_uint16	unsigned _int16
		#endif

		#ifndef pi_uint8
			#define	pi_uint8	unsigned _int8
		#endif
		
		#ifndef pi_int64
			#define	pi_int64	_int64
		#endif
		
		#ifndef pi_int32
			#define	pi_int32	_int32
		#endif
		
		#ifndef pi_int16
			#define	pi_int16	_int16
		#endif

		#ifndef pi_int8
			#define	pi_int8	_int8
		#endif
#else
	#ifndef BOOL
    	#define BOOL int
	#endif
	
#define E816_FUNC_DECL
#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
#ifndef pi_uint64
	#define	pi_uint64	u_int64_t
#endif

#ifndef pi_uint8
	#define	pi_uint8	u_int8_t
#endif

#ifndef pi_uint16
	#define	pi_uint16	u_int16_t
#endif

#ifndef pi_uint32
	#define	pi_uint32	u_int32_t
#endif

#ifndef pi_int64
	#define	pi_int64	int64_t
#endif

#ifndef pi_int32
	#define	pi_int32	int32_t
#endif

#ifndef _int32
	#define _int32 int32_t
#endif

#ifndef _int64
	#define _int64 int64_t
#endif

#endif

/////////////////////////////////////////////////////////////////////////////
// DLL initialization and comm functions
//

long E816_FUNC_DECL E816_InterfaceSetupDlg(const char* szRegKeyName);
long E816_FUNC_DECL E816_ConnectRS232(int nPortNr, long nBaudRate);
long E816_FUNC_DECL E816_FindOnRS(int* pnStartPort, int* pnStartBaud);
BOOL E816_FUNC_DECL E816_IsConnected(long ID);
void E816_FUNC_DECL E816_CloseConnection(long ID);
int E816_FUNC_DECL E816_GetError(long ID);
BOOL E816_FUNC_DECL E816_TranslateError(int errNr, char* szBuffer, int maxlen);
int	E816_FUNC_DECL E816_SetTimeout(long ID, int timeout);

long E816_FUNC_DECL E816_ConnectTCPIP(const char* szHostname, long port);
long E816_FUNC_DECL E816_EnumerateTCPIPDevices(char* szBuffer, long iBufferSize);
long E816_FUNC_DECL E816_ConnectTCPIPByDescription(const char* szDescription);

long E816_FUNC_DECL E816_EnumerateUSB(char* szBuffer, long iBufferSize, const char* szFilter);
long E816_FUNC_DECL E816_ConnectUSB(const char* szDescription);


/////////////////////////////////////////////////////////////////////////////


BOOL E816_FUNC_DECL E816_qIDN(long ID, char* szBuffer, int maxlen);
BOOL E816_FUNC_DECL E816_qERR(long ID, int* pnError);
BOOL E816_FUNC_DECL E816_qHLP(long ID, char* szBuffer, int maxlen);

BOOL E816_FUNC_DECL E816_qPOS(long ID, const char* axes, double* pdValarray);

BOOL E816_FUNC_DECL E816_qONT(long ID, const char* axes, BOOL* pbOnTarget);

BOOL E816_FUNC_DECL E816_MOV(long ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qMOV(long ID, const char* axes, double* pdValarray);
BOOL E816_FUNC_DECL E816_MVR(long ID, const char* axes, const double* pdValarray);

BOOL E816_FUNC_DECL E816_SVO(long ID, const char* szAxes, const BOOL* pbValarray);
BOOL E816_FUNC_DECL E816_qSVO(long ID, const char* szAxes, BOOL* pbValarray);

BOOL E816_FUNC_DECL E816_DCO(long ID, const char* szAxes, const BOOL* pbValarray);
BOOL E816_FUNC_DECL E816_qDCO(long ID, const char* szAxes, BOOL* pbValarray);

BOOL E816_FUNC_DECL E816_SVA(long ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qSVA(long ID, const char* axes, double* pdValarray);
BOOL E816_FUNC_DECL E816_SVR(long ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qVOL(long ID, const char* axes, double* pdValarray);

BOOL E816_FUNC_DECL E816_qOVF(long ID, const char* axes, BOOL* pbOverflow);

BOOL E816_FUNC_DECL E816_AVG(long ID, int nAverage);
BOOL E816_FUNC_DECL E816_qAVG(long ID, int* pnAverage);

BOOL E816_FUNC_DECL E816_SPA(long ID, const char* szAxes, const int* iCmdarray, const double* dValarray);
BOOL E816_FUNC_DECL E816_qSPA(long ID, const char* szAxes, const int* iCmdarray, double* dValarray);

BOOL E816_FUNC_DECL E816_WPA(long ID, const char* swPassword);

BOOL E816_FUNC_DECL E816_qSAI(long ID, char* axes, int maxlen);

BOOL E816_FUNC_DECL E816_qSSN(long ID, const char* szAxes, int* piValarray);

BOOL E816_FUNC_DECL E816_qSCH(long ID, char* pcChannelName);
BOOL E816_FUNC_DECL E816_SCH(long ID, char cChannelName);

BOOL E816_FUNC_DECL E816_RST(long ID);

BOOL E816_FUNC_DECL E816_BDR(long ID, int nBaudRate);
BOOL E816_FUNC_DECL E816_qBDR(long ID, int* pnBaudRate);

BOOL E816_FUNC_DECL E816_qI2C(long ID, int* pnErrorCode, char* pcChannel);

BOOL E816_FUNC_DECL E816_WTO(long ID, char cAxis, int nNumber);
BOOL E816_FUNC_DECL E816_WTOTimer(long ID, char cAxis, int nNumber, int timer);
BOOL E816_FUNC_DECL E816_SWT(long ID, char cAxis, int nIndex, double dValue);
BOOL E816_FUNC_DECL E816_qSWT(long ID, char cAxis, int nIndex, double* pdValue);

BOOL E816_FUNC_DECL E816_MVT(long ID, const char* szAxes, const BOOL* pbValarray);
BOOL E816_FUNC_DECL E816_qMVT(long ID, const char* szAxes, BOOL* pbValarray);

BOOL E816_FUNC_DECL E816_qDIP(long ID, const char* szAxes, BOOL* pbValarray);


/////////////////////////////////////////////////////////////////////////////

BOOL E816_FUNC_DECL E816_GcsCommandset(long ID, const char* szCommand);
BOOL E816_FUNC_DECL E816_GcsGetAnswer(long ID, char* szAnswer, int bufsize);
BOOL E816_FUNC_DECL E816_GcsGetAnswerSize(long ID, int* iAnswerSize);

BOOL E816_FUNC_DECL E816_ConfigPStage(long ID, char cAxis, double dPos10V, double dPos0V, BOOL bUseCurrentParams);
BOOL E816_FUNC_DECL E816_ConfigPZTVAmplifier(long ID, char cAxis, unsigned char ucAmpType, BOOL bUseCurrentParams);

BOOL E816_FUNC_DECL E816_IsRecordingMacro(int ID, BOOL* pbRecordingMacro);
BOOL E816_FUNC_DECL E816_IsRunningMacro(int ID, BOOL* pbRunningMacro);

BOOL E816_FUNC_DECL E816_MAC_DEL(int ID, const char* szName);

BOOL E816_FUNC_DECL E816_MAC_START(int ID, const char* szName);
BOOL E816_FUNC_DECL E816_MAC_NSTART(int ID, const char* szName, int nrRuns);
BOOL E816_FUNC_DECL E816_qMAC(int ID, const char* szName, char* szBuffer, int maxlen);

BOOL E816_FUNC_DECL E816_MAC_BEG(int ID, const char* szName);
BOOL E816_FUNC_DECL E816_MAC_END(int ID);

BOOL E816_FUNC_DECL E816_MAC_qFREE(int ID, int* pNumberChars);
BOOL E816_FUNC_DECL E816_MAC_DEF(int ID, const char* szName);
BOOL E816_FUNC_DECL E816_MAC_qDEF(int ID, char* szBuffer, int maxlen);

BOOL E816_FUNC_DECL E816_SaveMacroToFile(int ID, const char* szFileName, const char* szMacroName);
BOOL E816_FUNC_DECL E816_LoadMacroFromFile(int ID, const char* szFileName, const char* szMacroName);

#ifdef __cplusplus
}
#endif
