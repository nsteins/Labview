using System;
using System.Runtime.InteropServices;
using System.Text;


namespace PI
{

	public class E816
	{

		/////////////////////////////////////////////////////////////////////////////
		// DLL initialization and comm functions
		[DllImport("E816_DLL.dll", EntryPoint = "E816_ConnectRS232")]			public static extern int	ConnectRS232(int nPortNr, int nBaudRate);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_CloseConnection")]		public static extern void	CloseConnection(int iId);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_GetError")]				public static extern int	GetError(int iId);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_TranslateError")]			public static extern int	TranslateError(int errNr, StringBuilder sBuffer, int iMaxlen);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_IsConnected")]			public static extern int	IsConnected(int ID);
        [DllImport("E816_DLL.dll", EntryPoint = "E816_SetTimeout")]			    public static extern int	SetTimeout(int ID, int timeout);
        [DllImport("E816_DLL.dll", EntryPoint = "E816_EnumerateUSB")]			public static extern int	EnumerateUSB(StringBuilder sBuffer, int iBufferSize, string sFilter);
        [DllImport("E816_DLL.dll", EntryPoint = "E816_ConnectUSB")]		    	public static extern int	ConnectUSB(string sDescription);

		/////////////////////////////////////////////////////////////////////////////
		// general
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qERR")]	public static extern int	qERR(int iId, ref int nError);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qIDN")]	public static extern int	qIDN(int iId, StringBuilder buffer, int iMaxlen);
        [DllImport("E816_DLL.dll", EntryPoint = "qHLP")]		public static extern int	qHLP(int ID, StringBuilder sBuffer, int maxlen);

	
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qPOS")] public static extern int qPOS(int ID, string sAxes, double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qONT")] public static extern int qONT(int ID, string sAxes, int[] pbOnTarget);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_MOV")] public static extern int MOV(int ID, string sAxes,  double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qMOV")] public static extern int qMOV(int ID, string sAxes, double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_MVR")] public static extern int MVR(int ID, string sAxes,  double[] pdValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_SVO")] public static extern int SVO(int ID, string sAxes,  int[] pbValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSVO")] public static extern int qSVO(int ID, string sAxes, int[] pbValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_DCO")] public static extern int DCO(int ID, string sAxes,  int[] pbValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qDCO")] public static extern int qDCO(int ID, string sAxes, int[] pbValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_SVA")] public static extern int SVA(int ID, string sAxes,  double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSVA")] public static extern int qSVA(int ID, string sAxes, double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_SVR")] public static extern int SVR(int ID, string sAxes,  double[] pdValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qVOL")] public static extern int qVOL(int ID, string sAxes, double[] pdValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_qOVF")] public static extern int qOVF(int ID, string sAxes, int[] pbOverflow);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_AVG")] public static extern int AVG(int ID, int nAverage);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qAVG")] public static extern int qAVG(int ID, ref int pnAverage);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_SPA")] public static extern int SPA(int ID, string sAxes, int[] iCmdarray, double[] dValarray);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSPA")] public static extern int qSPA(int ID, string sAxes, int[] iCmdarray, double[] dValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_WPA")] public static extern int WPA(int ID,  string swPassword);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSAI")] public static extern int qSAI(int ID, StringBuilder axes, int maxlen);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSSN")] public static extern int qSSN(int ID, string sAxes, int[] piValarray);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSCH")] public static extern int qSCH(int ID,  ref char pcChannelName);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_SCH")] public static extern int SCH(int ID, char cChannelName);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_RST")] public static extern int RST(int ID);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_BDR")] public static extern int BDR(int ID, int nBaudRate);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qBDR")] public static extern int qBDR(int ID, ref int pnBaudRate);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_qI2C")] public static extern int qI2C(int ID, ref int errorCode, ref char pcChannel);

		[DllImport("E816_DLL.dll", EntryPoint = "E816_WTO")] public static extern int WTO(int ID, char cAxis, int nNumber);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_WTOTimer")] public static extern int WTOTimer(int ID, char cAxis, int nNumber, int timer);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_SWT")] public static extern int SWT(int ID, char cAxis, int nIndex, double dValue);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_qSWT")] public static extern int qSWT(int ID, char cAxis, int nIndex, ref double pdValue);

        [DllImport("E816_DLL.dll", EntryPoint = "MVT")]			public static extern int	MVT(int ID, string sAxes, int[] pbValarray);
        [DllImport("E816_DLL.dll", EntryPoint = "qMVT")]			public static extern int	qMVT(int ID, string sAxes, int[] pbValarray);

        [DllImport("E816_DLL.dll", EntryPoint = "qDIP")]			public static extern int	qDIP(int ID, string sAxes, int[] pbValarray);

  		/////////////////////////////////////////////////////////////////////////////

        [DllImport("E816_DLL.dll", EntryPoint = "IsRecordingMacro")]	public static extern int	IsRecordingMacro(int ID, ref int pbRecordingMacro);
        [DllImport("E816_DLL.dll", EntryPoint = "IsRunningMacro")]		public static extern int	IsRunningMacro(int ID, ref int pbRunningMacro);

        [DllImport("E816_DLL.dll", EntryPoint = "MAC_DEL")]			    public static extern int	MAC_DEL(int ID, string sName);

        [DllImport("E816_DLL.dll", EntryPoint = "MAC_START")]			public static extern int	MAC_START(int ID, string sName);
        [DllImport("E816_DLL.dll", EntryPoint = "MAC_NSTART")]			public static extern int	MAC_NSTART(int ID, string sName, int nrRuns);
        [DllImport("E816_DLL.dll", EntryPoint = "qMAC")]			    public static extern int	qMAC(int ID, string sName, StringBuilder sBuffer, int maxlen);

        [DllImport("E816_DLL.dll", EntryPoint = "MAC_BEG")]			    public static extern int	MAC_BEG(int ID, string sName);
        [DllImport("E816_DLL.dll", EntryPoint = "MAC_END")]			    public static extern int	MAC_END(int ID);

        [DllImport("E816_DLL.dll", EntryPoint = "MAC_qFREE")]			public static extern int	MAC_qFREE(int ID, ref int pNumberChars);
        [DllImport("E816_DLL.dll", EntryPoint = "MAC_DEF")]			    public static extern int	MAC_DEF(int ID, string sName);
        [DllImport("E816_DLL.dll", EntryPoint = "MAC_qDEF")]			public static extern int	MAC_qDEF(int ID, StringBuilder sBuffer, int maxlen);

        [DllImport("E816_DLL.dll", EntryPoint = "SaveMacroToFile")]		public static extern int	SaveMacroToFile(int ID, string sFileName, string sMacroName);
        [DllImport("E816_DLL.dll", EntryPoint = "LoadMacroFromFile")]	public static extern int	LoadMacroFromFile(int ID, string sFileName, string sMacroName);

        /////////////////////////////////////////////////////////////////////////////

		[DllImport("E816_DLL.dll", EntryPoint = "E816_GcsCommandset")] public static extern int GcsCommandset(int ID,  string sCommand);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_GcsGetAnswer")] public static extern int GcsGetAnswer(int ID, StringBuilder sAnswer, int bufsize);
		[DllImport("E816_DLL.dll", EntryPoint = "E816_GcsGetAnswerSize")] public static extern int GcsGetAnswerSize(int ID, ref int iAnswerSize);


	
	}
}
