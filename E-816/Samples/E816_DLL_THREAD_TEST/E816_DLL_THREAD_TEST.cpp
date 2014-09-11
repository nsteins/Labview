//////////////////////////////////////////////////////////////////////
// $HeaderUTC:E816_DLL_THREAD_TEST.cpp, 4, 2006-10-18 09:17:23Z, Steffen Rau$
// $LogUTC[5]:
//  4    E-816_GCS_DLLCOM1.3         2006-10-18 09:17:23Z     Steffen Rau    
//       added 64bit build for MOV-qMOV and thread sample
//  3    E-816_GCS_DLLCOM1.2         2004-04-16 12:32:04Z     Steffen Rau    
//       final test with 6 COM ports
//  2    E-816_GCS_DLLCOM1.1         2004-04-16 10:53:12Z     Steffen Rau    
//       test with 6 COM ports
//  1    E-816_GCS_DLLCOM1.0         2004-04-15 14:11:19Z     Steffen Rau     
// $
//
//////////////////////////////////////////////////////////////////////
// E816_DLL_THREAD_TEST.cpp : Defines the entry point for the console application.
//

#include <windows.h>
#include <stdio.h>

#include "..\..\source\E816_DLL\E816_DLL.h"

struct COMPARAM
{
	int port;
	int baud;
	HANDLE hThreadEndEvent;
};

FILE* pLogFile = NULL;

void LogPrintf(int threadID, const char *format, ...)
{
	va_list args;
	char mess[1024] = {'\0'};

	va_start (args, format);
	vsprintf (mess, format, args);
	va_end (args);

	char str[1200];
	sprintf(str, "**** Thread(%d): %s\n", threadID, mess);
	printf(str);
	if (pLogFile != NULL)
		fprintf(pLogFile, str);
#ifdef _DEBUG
	OutputDebugStr(str);
#endif
}



DWORD WINAPI ThreadFunc( LPVOID lpParam ) 
{
	int ID = -1;
	int tID = int(GetCurrentThreadId());
	LogPrintf(tID, "Enter thread");
	Sleep(50);
	COMPARAM* pParam = (COMPARAM*)lpParam;
	ID = E816_ConnectRS232(pParam->port, pParam->baud);
	if (ID<0)
	{
		LogPrintf(tID, "could not open interface COM%d, %d baud", pParam->port, pParam->baud);
		return 0;
	}

	LogPrintf(tID, "E-816 on COM%d, %d baud, ID=%d", pParam->port, pParam->baud, ID);
	Sleep(100);

	char szAxis[] = "A";
	int SSN;
	char IDN[1024];
	if (!E816_qSSN(ID, szAxis, &SSN)) goto stop;
	if (!E816_qIDN(ID, IDN, 1023)) goto stop;
	LogPrintf(tID, "*IDN?: \"%s\", SSN: %d", IDN, SSN);
	Sleep(100);

	for (int i=0; i<1000; i++)
	{
		double pos, vol;
		int tmpSSN;
		if (!E816_qPOS(ID, szAxis, &pos)) goto stop;
		if (!E816_qVOL(ID, szAxis, &vol)) goto stop;
		if (!E816_qSSN(ID, szAxis, &tmpSSN)) goto stop;
		LogPrintf(tID, "loop %d, COM%d, pos: %.4f, vol%.4f, SSN: %d", i, pParam->port, pos, vol, tmpSSN);
		if (tmpSSN != SSN)
		{
			LogPrintf(tID, "wrong serial number: %d != %d", SSN, tmpSSN);
			goto stop;
		}
		double sleeptime = double(rand())/RAND_MAX;
		Sleep(int(30*sleeptime));
	}

stop:
	int err = E816_GetError(ID);
	char errMsg[1024];
	E816_TranslateError(err, errMsg, 1023);
	LogPrintf(tID, "COM%d, Error %d: \"%s\"", pParam->port, err, errMsg);
	LogPrintf(tID, "COM%d, Closing connection E-816 ID=%d", pParam->port, ID);
	E816_CloseConnection(ID);

	return 0;
}

int main(int argc, char* argv[])
{
	const int nrThreads = 2;
	int ports[nrThreads] = { 1, 2};
	int baud[nrThreads] = { 115200, 115200};

	//const int nrThreads = 6;
	//int ports[nrThreads] = { 1, 2, 3, 4, 5, 6};
	//int baud[nrThreads] = { 115200, 115200, 115200, 115200, 115200, 115200 };

	//const int nrThreads = 1;
	//int ports[nrThreads] = { 3 };
	//int baud[nrThreads] = { 115200 };

	//const int nrThreads = 1;
	//int ports[nrThreads] = { 6 };
	//int baud[nrThreads] = { 115200 };

	pLogFile = fopen("E816_DLL_THREAD_TEST.log", "w");

	COMPARAM threadParams[nrThreads];
	HANDLE hThreads[nrThreads];
	DWORD dwThreadIDs[nrThreads];


	for (int i=0; i<nrThreads; i++)
	{
		threadParams[i].port = ports[i];
		threadParams[i].baud = baud[i];

		hThreads[i] = CreateThread( (LPSECURITY_ATTRIBUTES)NULL,
		                           0,
								   ThreadFunc,
							       (LPVOID)&threadParams[i],
							       0,
							       &dwThreadIDs[i] );

	}

	WaitForMultipleObjects(nrThreads, hThreads, TRUE, INFINITE);

	return 0;
}

