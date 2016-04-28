Attribute VB_Name = "ME816"

Public Declare Function E816_InterfaceSetupDlg Lib "E816_DLL.dll" (ByVal szRegKeyName As String) As Long
Public Declare Function E816_ConnectRS232 Lib "E816_DLL.dll" (ByVal nPortNr As Long, ByVal nBaudRate As Long) As Long

Public Declare Function E816_EnumerateUSB Lib "E816_DLL.dll" (ByVal szBuffer As String, ByVal iBufferSize As Long, ByVal szFilter As String) As Long
Public Declare Function E816_ConnectUSB Lib "E816_DLL.dll" (ByVal szDescription As String) As Long

Public Declare Function E816_IsConnected Lib "E816_DLL.dll" (ByVal ID As Long) As Long
Public Declare Sub E816_CloseConnection Lib "E816_DLL.dll" (ByVal ID As Long)
Public Declare Function E816_GetError Lib "E816_DLL.dll" (ByVal ID As Long) As Long
Public Declare Function E816_SetErrorCheck Lib "E816_DLL.dll" (ByVal ID As Long, ByVal bErrorCheck As Long) As Long
Public Declare Function E816_TranslateError Lib "E816_DLL.dll" (ByVal errNr As Long, ByVal szBuffer As String, ByVal iBufferSize As Long) As Long
Public Declare Function E816_SetTimeout(ByVal ID As Long, ByVal timeout As Long) As Long




'/////////////////////////////////////////////////////////////////////////////
'// general
Public Declare Function E816_qERR Lib "E816_DLL.dll" (ByVal ID As Long, ByRef pnError As Long) As Long
Public Declare Function E816_qIDN Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szBuffer As String, ByVal iBufferSize As Long) As Long
Public Declare Function E816_qHLP Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szBuffer As String, ByVal iBufferSize As Long) As Long
Public Declare Function E816_BDR Lib "E816_DLL.dll" (ByVal ID As Long, ByVal iBaudRate As Long) As Long
Public Declare Function E816_qBDR Lib "E816_DLL.dll" (ByVal ID As Long, ByVal iBaudRate As Long) As Long

Public Declare Function E816_MOV Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_qMOV Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_MVR Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_qPOS Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_HLT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String) As Long
Public Declare Function E816_STP Lib "E816_DLL.dll" (ByVal ID As Long) As Long
Public Declare Function E816_qONT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long

Public Declare Function E816_SVA Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_qSVA Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_SVR Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pdValueArray As Double) As Long
Public Declare Function E816_qVOL Lib "E816_DLL.dll" (ByVal ID As Long, ByRef piPiezoChannelsArray As Long, ByRef pdValueArray As Double, ByVal iArraySize As Long) As Long

Public Declare Function E816_SVO Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long
Public Declare Function E816_qSVO Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long
Public Declare Function E816_DCO Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long
Public Declare Function E816_qDCO Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long

Public Declare Function E816_SPA Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef iParameterArray As Long, ByRef pdValueArray As Double, ByVal szStrings As String) As Long
Public Declare Function E816_qSPA Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef iParameterArray As Long, ByRef pdValueArray As Double, ByVal szStrings As String, ByVal iMaxNameSize As Long) As Long
Public Declare Function E816_WPA Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szPassWord As String, ByVal szAxes As String, ByRef iParameterArray As Long) As Long
Public Declare Function E816_SPA_String Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef iParameterArray As Long, ByVal szStrings As String) As Long
Public Declare Function E816_qSPA_String Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef iParameterArray As Long, ByVal szStrings As String, ByVal iMaxNameSize As Long) As Long

Public Declare Function E816_qSAI Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByVal iBufferSize As Long) As Long

Public Declare Function E816_AVG Lib "E816_DLL.dll" (ByVal ID As Long, ByVal iAverrageTime As Long) As Long
Public Declare Function E816_qAVG Lib "E816_DLL.dll" (ByVal ID As Long, ByVal iAverrageTime As Long) As Long
Public Declare Function E816_qOVF Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long

Public Declare Function E816_qSSN Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long

Public Declare Function E816_qSCH Lib "E816_DLL.dll" (ByVal ID As Long, ByRef pcChannelName As Char) As Long
Public Declare Function E816_SCH Lib "E816_DLL.dll" (ByVal ID As Long, ByVal cChannelName As Char) As Long

Public Declare Function E816_RST Lib "E816_DLL.dll" (ByVal ID As Long) As Long
Public Declare Function E816_WTO Lib "E816_DLL.dll" (ByVal ID As Long, ByVal cAxis As Char, ByVal nNumber As Long) As Long
Public Declare Function E816_WTOTimer Lib "E816_DLL.dll" (ByVal ID As Long, ByVal cAxis As Char, ByVal nNumber As Long, ByVal timer As Long) As Long
Public Declare Function E816_SWT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal cAxis As Char, ByVal nIndex As Long, ByRef pdValueArray As Double) As Long
Public Declare Function E816_qSWT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal cAxis As Char, ByVal nIndex As Long, ByRef pdValueArray As Double) As Long

Public Declare Function E816_MVT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long
Public Declare Function E816_qMVT Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long

Public Declare Function E816_qDIP Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAxes As String, ByRef pbValueArray As Long) As Long


'/////////////////////////////////////////////////////////////////////////////
'// Macro commande
Public Declare Function E816_IsRecordingMacro Lib "E816_DLL.dll" (ByVal ID As Long, ByRef pbRecordingMacro As Long) As Long
Public Declare Function E816_IsRunningMacro Lib "E816_DLL.dll" (ByVal ID As Long, ByRef pbRunningMacro As Long) As Long
Public Declare Function E816_MAC_BEG Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String) As Long
Public Declare Function E816_MAC_START Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String) As Long
Public Declare Function E816_MAC_NSTART Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String, ByVal nrRuns As Long) As Long
Public Declare Function E816_MAC_END Lib "E816_DLL.dll" (ByVal ID As Long) As Long
Public Declare Function E816_MAC_DEL Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String) As Long
Public Declare Function E816_MAC_DEF Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String) As Long
Public Declare Function E816_MAC_qDEF Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szBuffer As String, ByVal iBufferSize As Long) As Long
Public Declare Function E816_qMAC Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szMacroName As String, ByVal szBuffer As String, ByVal iBufferSize As Long) As Long

Public Declare Function E816_DEL Lib "E816_DLL.dll" (ByVal ID As Long, ByVal nMilliSeconds As Long) As Long

'/////////////////////////////////////////////////////////////////////////////
'// String commands.
Public Declare Function E816_GcsCommandset Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szCommand As String) As Long
Public Declare Function E816_GcsGetAnswer Lib "E816_DLL.dll" (ByVal ID As Long, ByVal szAnswer As String, ByVal iBufferSize As Long) As Long
Public Declare Function E816_GcsGetAnswerSize Lib "E816_DLL.dll" (ByVal ID As Long, ByRef iAnswerSize As Long) As Long




