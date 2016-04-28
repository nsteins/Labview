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

#undef E816_FUNC_DECL
#ifdef E816_DLL_EXPORTS
#define E816_FUNC_DECL __declspec(dllexport) WINAPI
#else
#define E816_FUNC_DECL __declspec(dllimport) WINAPI
#endif

/////////////////////////////////////////////////////////////////////////////
// DLL initialization and comm functions
//
#ifndef _GENERIC_I_NO_GUI
int E816_FUNC_DECL E816_InterfaceSetupDlg(const char* szRegKeyName);
#endif
int E816_FUNC_DECL E816_ConnectRS232(int nPortNr, long nBaudRate);
int E816_FUNC_DECL E816_FindOnRS(int* pnStartPort, int* pnStartBaud);
BOOL E816_FUNC_DECL E816_IsConnected(int ID);
void E816_FUNC_DECL E816_CloseConnection(int ID);
int E816_FUNC_DECL E816_GetError(int ID);
BOOL E816_FUNC_DECL E816_TranslateError(int errNr, char* szBuffer, int maxlen);

/////////////////////////////////////////////////////////////////////////////


BOOL E816_FUNC_DECL E816_qIDN(int ID, char* szBuffer, int maxlen);
BOOL E816_FUNC_DECL E816_qERR(int ID, int* pnError);

BOOL E816_FUNC_DECL E816_qPOS(int ID, const char* axes, double* pdValarray);

BOOL E816_FUNC_DECL E816_qONT(int ID, const char* axes, BOOL* pbOnTarget);

BOOL E816_FUNC_DECL E816_MOV(int ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qMOV(int ID, const char* axes, double* pdValarray);
BOOL E816_FUNC_DECL E816_MVR(int ID, const char* axes, const double* pdValarray);

BOOL E816_FUNC_DECL E816_SVO(int ID, const char* szAxes, const BOOL* pbValarray);
BOOL E816_FUNC_DECL E816_qSVO(int ID, const char* szAxes, BOOL* pbValarray);

BOOL E816_FUNC_DECL E816_DCO(int ID, const char* szAxes, const BOOL* pbValarray);
BOOL E816_FUNC_DECL E816_qDCO(int ID, const char* szAxes, BOOL* pbValarray);

BOOL E816_FUNC_DECL E816_SVA(int ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qSVA(int ID, const char* axes, double* pdValarray);
BOOL E816_FUNC_DECL E816_SVR(int ID, const char* axes, const double* pdValarray);
BOOL E816_FUNC_DECL E816_qVOL(int ID, const char* axes, double* pdValarray);

BOOL E816_FUNC_DECL E816_qOVF(int ID, const char* axes, BOOL* pbOverflow);

BOOL E816_FUNC_DECL E816_AVG(int ID, int nAverage);
BOOL E816_FUNC_DECL E816_qAVG(int ID, int* pnAverage);

BOOL E816_FUNC_DECL E816_SPA(int ID, const char* szAxes, const int* iCmdarray, const double* dValarray);
BOOL E816_FUNC_DECL E816_qSPA(int ID, const char* szAxes, const int* iCmdarray, double* dValarray);

BOOL E816_FUNC_DECL E816_WPA(int ID, const char* swPassword);

BOOL E816_FUNC_DECL E816_qSAI(int ID, char* axes, int maxlen);

BOOL E816_FUNC_DECL E816_qSSN(int ID, const char* szAxes, int* piValarray);

BOOL E816_FUNC_DECL E816_qSCH(int ID, char* pcChannelName);
BOOL E816_FUNC_DECL E816_SCH(int ID, char cChannelName);

BOOL E816_FUNC_DECL E816_RST(int ID);

BOOL E816_FUNC_DECL E816_BDR(int ID, int nBaudRate);
BOOL E816_FUNC_DECL E816_qBDR(int ID, int* pnBaudRate);

BOOL E816_FUNC_DECL E816_qI2C(int ID, int* pnErrorCode, char* pcChannel);

BOOL E816_FUNC_DECL E816_WTO(int ID, char cAxis, int nNumber);
BOOL E816_FUNC_DECL E816_WTOTimer(int ID, char cAxis, int nNumber, int timer);
BOOL E816_FUNC_DECL E816_SWT(int ID, char cAxis, int nIndex, double dValue);
BOOL E816_FUNC_DECL E816_qSWT(int ID, char cAxis, int nIndex, double* pdValue);

/////////////////////////////////////////////////////////////////////////////

BOOL E816_FUNC_DECL E816_GcsCommandset(int ID, const char* szCommand);
BOOL E816_FUNC_DECL E816_GcsGetAnswer(int ID, char* szAnswer, int bufsize);
BOOL E816_FUNC_DECL E816_GcsGetAnswerSize(int ID, int* iAnswerSize);

BOOL E816_FUNC_DECL E816_ConfigPStage(int ID, char cAxis, double dPos10V, double dPos0V, BOOL bUseCurrentParams);
BOOL E816_FUNC_DECL E816_ConfigPZTVAmplifier(int ID, char cAxis, unsigned char ucAmpType, BOOL bUseCurrentParams);

#ifdef __cplusplus
}
#endif