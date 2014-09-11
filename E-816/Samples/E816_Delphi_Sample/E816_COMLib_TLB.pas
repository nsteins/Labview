unit E816_COMLib_TLB;

// ************************************************************************ //
// WARNUNG                                                                    
// -------                                                                    
// Die in dieser Datei deklarierten Typen wurden aus Daten einer Typbibliothek
// generiert. Wenn diese Typbibliothek explizit oder indirekt (über eine     
// andere Typbibliothek) reimportiert wird oder wenn die Anweisung            
// 'Aktualisieren' im Typbibliotheks-Editor während des Bearbeitens der     
// Typbibliothek aktiviert ist, wird der Inhalt dieser Datei neu generiert und 
// alle manuell vorgenommenen Änderungen gehen verloren.                           
// ************************************************************************ //

// PASTLWTR : 1.2
// Datei generiert am 16.12.04 16:11:59 aus der unten beschriebenen Typbibliothek.

// ************************************************************************  //
// Typbib: N:\Produkte E\E-816\Software\CD-Mirror__2004_12_06__1.1.1\E816_COM\E816_COM.tlb (1)
// LIBID: {3F8F5361-5810-11D6-9F1E-0004768D79CE}
// LCID: 0
// Hilfedatei: 
// Hilfe-String: E816_COM 1.0 Type Library
// DepndLst: 
//   (1) v2.0 stdole, (C:\WINNT\System32\STDOLE2.TLB)
// Fehler
//   Fehler beim Erzeugen von Palettenbitmap von (TE816) : Server N:\Produkte E\E-816\Software\CD-Mirror__2004_12_06__1.1.1\E816_COM\E816_COM.DLL enthält keine Symbole
// ************************************************************************ //
// *************************************************************************//              
// HINWEIS:                                                                                   
// Von $IFDEF_LIVE_SERVER_AT_DESIGN_TIME überwachte Einträge, werden von  
// Eigenschaften verwendet, die Objekte zurückgeben, die explizit mit einen Funktionsaufruf  
// vor dem Zugriff über die Eigenschaft erzeugt werden müssen. Diese Einträge wurden deaktiviert,  
// um deren unbeabsichtigte Benutzung im Objektinspektor zu verhindern. Sie können sie  
// aktivieren, indem Sie LIVE_SERVER_AT_DESIGN_TIME definieren oder sie selektiv  
// aus den $IFDEF-Blöcken entfernen. Solche Einträge müssen jedoch programmseitig 
// mit einer Methode der geeigneten CoClass vor der Verwendung  
// erzeugt werden.                                                                 
{$TYPEDADDRESS OFF} // Unit muß ohne Typüberprüfung für Zeiger compiliert werden. 
{$WARN SYMBOL_PLATFORM OFF}
{$WRITEABLECONST ON}
{$VARPROPSETTER ON}
interface

uses Windows, ActiveX, Classes, Graphics, OleServer, StdVCL, Variants;
  

// *********************************************************************//
// In dieser Typbibliothek deklarierte GUIDS . Es werden folgende         
// Präfixe verwendet:                                                     
//   Typbibliotheken     : LIBID_xxxx                                     
//   CoClasses           : CLASS_xxxx                                     
//   DISPInterfaces      : DIID_xxxx                                      
//   Nicht-DISP-Schnittstellen: IID_xxxx                                       
// *********************************************************************//
const
  // Haupt- und Nebenversionen der Typbibliothek
  E816_COMLibMajorVersion = 1;
  E816_COMLibMinorVersion = 0;

  LIBID_E816_COMLib: TGUID = '{3F8F5361-5810-11D6-9F1E-0004768D79CE}';

  DIID__IE816Events: TGUID = '{3F8F536F-5810-11D6-9F1E-0004768D79CE}';
  IID_IE816: TGUID = '{3F8F536D-5810-11D6-9F1E-0004768D79CE}';
  CLASS_E816: TGUID = '{3F8F536E-5810-11D6-9F1E-0004768D79CE}';
type

// *********************************************************************//
// Forward-Deklaration von in der Typbibliothek definierten Typen         
// *********************************************************************//
  _IE816Events = dispinterface;
  IE816 = interface;
  IE816Disp = dispinterface;

// *********************************************************************//
// Deklaration von in der Typbibliothek definierten CoClasses             
// (HINWEIS: Hier wird jede CoClass zu ihrer Standardschnittstelle        
// zugewiesen)                                                            
// *********************************************************************//
  E816 = IE816;


// *********************************************************************// 
// Deklaration von  Strukturen, Unions und Aliasen.                        
// *********************************************************************// 
  PWideString1 = ^WideString; {*}


// *********************************************************************//
// DispIntf:  _IE816Events
// Flags:     (4096) Dispatchable
// GUID:      {3F8F536F-5810-11D6-9F1E-0004768D79CE}
// *********************************************************************//
  _IE816Events = dispinterface
    ['{3F8F536F-5810-11D6-9F1E-0004768D79CE}']
  end;

// *********************************************************************//
// Schnittstelle: IE816
// Flags:     (4416) Dual OleAutomation Dispatchable
// GUID:      {3F8F536D-5810-11D6-9F1E-0004768D79CE}
// *********************************************************************//
  IE816 = interface(IDispatch)
    ['{3F8F536D-5810-11D6-9F1E-0004768D79CE}']
    function InterfaceSetupDlg(var pStrRegKeyname: WideString): Integer; safecall;
    procedure CloseConnection; safecall;
    function ConnectRS232(nPortNr: Integer; nBaud: Integer): Integer; safecall;
    function FindOnRS(var plStartPort: Integer; var plStartBaud: Integer): Integer; safecall;
    function IsConnected: Integer; safecall;
    function GetError: Integer; safecall;
    function TranslateError(errNr: Integer): WideString; safecall;
    function qIDN(out pStrIdn: WideString): Integer; safecall;
    function qERR(out pError: Integer): Integer; safecall;
    function MOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qMOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function MVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qPOS(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qONT(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function AVG(Average: Integer): Integer; safecall;
    function qAVG(out pAverage: Integer): Integer; safecall;
    function SVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qSVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function SVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qSVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function SVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qVOL(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qOVF(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function SPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer; safecall;
    function qSPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer; safecall;
    function WPA(var pStrPassword: WideString): Integer; safecall;
    function qSAI(out pStrAxes: WideString): Integer; safecall;
    function qSSN(var pStrAxes: WideString; var pValArray: PSafeArray): Integer; safecall;
    function qSCH(out pStrAxis: WideString): Integer; safecall;
    function SCH(var pStrAxis: WideString): Integer; safecall;
    function RST: Integer; safecall;
    function BDR(nBaudRate: Integer): Integer; safecall;
    function qBDR(out pnBaudRate: Integer): Integer; safecall;
    function qI2C(out pnErrorCode: Integer; out pStrChannel: WideString): Integer; safecall;
    function WTO(var pStrAxis: WideString; nNumber: Integer): Integer; safecall;
    function SWT(var pStrAxis: WideString; nIndex: Integer; dValue: Double): Integer; safecall;
  end;

// *********************************************************************//
// DispIntf:  IE816Disp
// Flags:     (4416) Dual OleAutomation Dispatchable
// GUID:      {3F8F536D-5810-11D6-9F1E-0004768D79CE}
// *********************************************************************//
  IE816Disp = dispinterface
    ['{3F8F536D-5810-11D6-9F1E-0004768D79CE}']
    function InterfaceSetupDlg(var pStrRegKeyname: WideString): Integer; dispid 1;
    procedure CloseConnection; dispid 2;
    function ConnectRS232(nPortNr: Integer; nBaud: Integer): Integer; dispid 3;
    function FindOnRS(var plStartPort: Integer; var plStartBaud: Integer): Integer; dispid 4;
    function IsConnected: Integer; dispid 5;
    function GetError: Integer; dispid 6;
    function TranslateError(errNr: Integer): WideString; dispid 7;
    function qIDN(out pStrIdn: WideString): Integer; dispid 8;
    function qERR(out pError: Integer): Integer; dispid 9;
    function MOV(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 10;
    function qMOV(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 11;
    function MVR(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 12;
    function qPOS(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 13;
    function qONT(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 14;
    function AVG(Average: Integer): Integer; dispid 15;
    function qAVG(out pAverage: Integer): Integer; dispid 16;
    function SVO(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 17;
    function qSVO(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 18;
    function SVA(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 19;
    function qSVA(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 20;
    function SVR(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 21;
    function qVOL(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 22;
    function qOVF(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 23;
    function SPA(var pStrAxes: WideString; var plCmdArray: {??PSafeArray}OleVariant; 
                 var pdValArray: {??PSafeArray}OleVariant): Integer; dispid 24;
    function qSPA(var pStrAxes: WideString; var plCmdArray: {??PSafeArray}OleVariant; 
                  var pdValArray: {??PSafeArray}OleVariant): Integer; dispid 25;
    function WPA(var pStrPassword: WideString): Integer; dispid 26;
    function qSAI(out pStrAxes: WideString): Integer; dispid 27;
    function qSSN(var pStrAxes: WideString; var pValArray: {??PSafeArray}OleVariant): Integer; dispid 28;
    function qSCH(out pStrAxis: WideString): Integer; dispid 29;
    function SCH(var pStrAxis: WideString): Integer; dispid 30;
    function RST: Integer; dispid 31;
    function BDR(nBaudRate: Integer): Integer; dispid 32;
    function qBDR(out pnBaudRate: Integer): Integer; dispid 33;
    function qI2C(out pnErrorCode: Integer; out pStrChannel: WideString): Integer; dispid 34;
    function WTO(var pStrAxis: WideString; nNumber: Integer): Integer; dispid 35;
    function SWT(var pStrAxis: WideString; nIndex: Integer; dValue: Double): Integer; dispid 36;
  end;

// *********************************************************************//
// Die Klasse CoE816 stellt die Methoden Create und CreateRemote zur      
// Verfügung, um Instanzen der Standardschnittstelle IE816, dargestellt von
// CoClass E816, zu erzeugen. Diese Funktionen können                     
// von einem Client verwendet werden, der die CoClasses automatisieren    
// möchte, die von dieser Typbibliothek dargestellt werden.               
// *********************************************************************//
  CoE816 = class
    class function Create: IE816;
    class function CreateRemote(const MachineName: string): IE816;
  end;


// *********************************************************************//
// OLE-Server-Proxy-Klassendeklaration
// Server-Objekt    : TE816
// Hilfe-String     : E816 Class
// Standardschnittstelle: IE816
// Def. Intf. DISP? : No
// Ereignisschnittstelle: _IE816Events
// TypeFlags        : (2) CanCreate
// *********************************************************************//
{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
  TE816Properties= class;
{$ENDIF}
  TE816 = class(TOleServer)
  private
    FIntf:        IE816;
{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
    FProps:       TE816Properties;
    function      GetServerProperties: TE816Properties;
{$ENDIF}
    function      GetDefaultInterface: IE816;
  protected
    procedure InitServerData; override;
    procedure InvokeEvent(DispID: TDispID; var Params: TVariantArray); override;
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    procedure Connect; override;
    procedure ConnectTo(svrIntf: IE816);
    procedure Disconnect; override;
    function InterfaceSetupDlg(var pStrRegKeyname: WideString): Integer;
    procedure CloseConnection;
    function ConnectRS232(nPortNr: Integer; nBaud: Integer): Integer;
    function FindOnRS(var plStartPort: Integer; var plStartBaud: Integer): Integer;
    function IsConnected: Integer;
    function GetError: Integer;
    function TranslateError(errNr: Integer): WideString;
    function qIDN(out pStrIdn: WideString): Integer;
    function qERR(out pError: Integer): Integer;
    function MOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qMOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function MVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qPOS(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qONT(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function AVG(Average: Integer): Integer;
    function qAVG(out pAverage: Integer): Integer;
    function SVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qSVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function SVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qSVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function SVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qVOL(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qOVF(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function SPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer;
    function qSPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer;
    function WPA(var pStrPassword: WideString): Integer;
    function qSAI(out pStrAxes: WideString): Integer;
    function qSSN(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
    function qSCH(out pStrAxis: WideString): Integer;
    function SCH(var pStrAxis: WideString): Integer;
    function RST: Integer;
    function BDR(nBaudRate: Integer): Integer;
    function qBDR(out pnBaudRate: Integer): Integer;
    function qI2C(out pnErrorCode: Integer; out pStrChannel: WideString): Integer;
    function WTO(var pStrAxis: WideString; nNumber: Integer): Integer;
    function SWT(var pStrAxis: WideString; nIndex: Integer; dValue: Double): Integer;
    property DefaultInterface: IE816 read GetDefaultInterface;
  published
{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
    property Server: TE816Properties read GetServerProperties;
{$ENDIF}
  end;

{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
// *********************************************************************//
// OLE-Server-Properties-Proxy-Klasse
// Server-Objekt    : TE816
// (Dieses Objekt wird vom Eigenschaftsinspektor der IDE verwendet,
//  um die Eigenschaften dieses Servers zu bearbeiten)
// *********************************************************************//
 TE816Properties = class(TPersistent)
  private
    FServer:    TE816;
    function    GetDefaultInterface: IE816;
    constructor Create(AServer: TE816);
  protected
  public
    property DefaultInterface: IE816 read GetDefaultInterface;
  published
  end;
{$ENDIF}


procedure Register;

resourcestring
  dtlServerPage = 'ActiveX';

  dtlOcxPage = 'ActiveX';

implementation

uses ComObj;

class function CoE816.Create: IE816;
begin
  Result := CreateComObject(CLASS_E816) as IE816;
end;

class function CoE816.CreateRemote(const MachineName: string): IE816;
begin
  Result := CreateRemoteComObject(MachineName, CLASS_E816) as IE816;
end;

procedure TE816.InitServerData;
const
  CServerData: TServerData = (
    ClassID:   '{3F8F536E-5810-11D6-9F1E-0004768D79CE}';
    IntfIID:   '{3F8F536D-5810-11D6-9F1E-0004768D79CE}';
    EventIID:  '{3F8F536F-5810-11D6-9F1E-0004768D79CE}';
    LicenseKey: nil;
    Version: 500);
begin
  ServerData := @CServerData;
end;

procedure TE816.Connect;
var
  punk: IUnknown;
begin
  if FIntf = nil then
  begin
    punk := GetServer;
    ConnectEvents(punk);
    Fintf:= punk as IE816;
  end;
end;

procedure TE816.ConnectTo(svrIntf: IE816);
begin
  Disconnect;
  FIntf := svrIntf;
  ConnectEvents(FIntf);
end;

procedure TE816.DisConnect;
begin
  if Fintf <> nil then
  begin
    DisconnectEvents(FIntf);
    FIntf := nil;
  end;
end;

function TE816.GetDefaultInterface: IE816;
begin
  if FIntf = nil then
    Connect;
  Assert(FIntf <> nil, 'DefaultInterface ist NULL. Die Komponente ist nicht mit dem Server verbunden. Sie müssen vor dieser Operation ''Connect'' oder ''ConnectTo'' aufrufen');
  Result := FIntf;
end;

constructor TE816.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
  FProps := TE816Properties.Create(Self);
{$ENDIF}
end;

destructor TE816.Destroy;
begin
{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
  FProps.Free;
{$ENDIF}
  inherited Destroy;
end;

{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
function TE816.GetServerProperties: TE816Properties;
begin
  Result := FProps;
end;
{$ENDIF}

procedure TE816.InvokeEvent(DispID: TDispID; var Params: TVariantArray);
begin
  case DispID of
    -1: Exit;  // DISPID_UNKNOWN
  end; {case DispID}
end;

function TE816.InterfaceSetupDlg(var pStrRegKeyname: WideString): Integer;
begin
  Result := DefaultInterface.InterfaceSetupDlg(pStrRegKeyname);
end;

procedure TE816.CloseConnection;
begin
  DefaultInterface.CloseConnection;
end;

function TE816.ConnectRS232(nPortNr: Integer; nBaud: Integer): Integer;
begin
  Result := DefaultInterface.ConnectRS232(nPortNr, nBaud);
end;

function TE816.FindOnRS(var plStartPort: Integer; var plStartBaud: Integer): Integer;
begin
  Result := DefaultInterface.FindOnRS(plStartPort, plStartBaud);
end;

function TE816.IsConnected: Integer;
begin
  Result := DefaultInterface.IsConnected;
end;

function TE816.GetError: Integer;
begin
  Result := DefaultInterface.GetError;
end;

function TE816.TranslateError(errNr: Integer): WideString;
begin
  Result := DefaultInterface.TranslateError(errNr);
end;

function TE816.qIDN(out pStrIdn: WideString): Integer;
begin
  Result := DefaultInterface.qIDN(pStrIdn);
end;

function TE816.qERR(out pError: Integer): Integer;
begin
  Result := DefaultInterface.qERR(pError);
end;

function TE816.MOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.MOV(pStrAxes, pValArray);
end;

function TE816.qMOV(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qMOV(pStrAxes, pValArray);
end;

function TE816.MVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.MVR(pStrAxes, pValArray);
end;

function TE816.qPOS(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qPOS(pStrAxes, pValArray);
end;

function TE816.qONT(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qONT(pStrAxes, pValArray);
end;

function TE816.AVG(Average: Integer): Integer;
begin
  Result := DefaultInterface.AVG(Average);
end;

function TE816.qAVG(out pAverage: Integer): Integer;
begin
  Result := DefaultInterface.qAVG(pAverage);
end;

function TE816.SVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.SVO(pStrAxes, pValArray);
end;

function TE816.qSVO(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qSVO(pStrAxes, pValArray);
end;

function TE816.SVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.SVA(pStrAxes, pValArray);
end;

function TE816.qSVA(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qSVA(pStrAxes, pValArray);
end;

function TE816.SVR(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.SVR(pStrAxes, pValArray);
end;

function TE816.qVOL(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qVOL(pStrAxes, pValArray);
end;

function TE816.qOVF(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qOVF(pStrAxes, pValArray);
end;

function TE816.SPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.SPA(pStrAxes, plCmdArray, pdValArray);
end;

function TE816.qSPA(var pStrAxes: WideString; var plCmdArray: PSafeArray; var pdValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qSPA(pStrAxes, plCmdArray, pdValArray);
end;

function TE816.WPA(var pStrPassword: WideString): Integer;
begin
  Result := DefaultInterface.WPA(pStrPassword);
end;

function TE816.qSAI(out pStrAxes: WideString): Integer;
begin
  Result := DefaultInterface.qSAI(pStrAxes);
end;

function TE816.qSSN(var pStrAxes: WideString; var pValArray: PSafeArray): Integer;
begin
  Result := DefaultInterface.qSSN(pStrAxes, pValArray);
end;

function TE816.qSCH(out pStrAxis: WideString): Integer;
begin
  Result := DefaultInterface.qSCH(pStrAxis);
end;

function TE816.SCH(var pStrAxis: WideString): Integer;
begin
  Result := DefaultInterface.SCH(pStrAxis);
end;

function TE816.RST: Integer;
begin
  Result := DefaultInterface.RST;
end;

function TE816.BDR(nBaudRate: Integer): Integer;
begin
  Result := DefaultInterface.BDR(nBaudRate);
end;

function TE816.qBDR(out pnBaudRate: Integer): Integer;
begin
  Result := DefaultInterface.qBDR(pnBaudRate);
end;

function TE816.qI2C(out pnErrorCode: Integer; out pStrChannel: WideString): Integer;
begin
  Result := DefaultInterface.qI2C(pnErrorCode, pStrChannel);
end;

function TE816.WTO(var pStrAxis: WideString; nNumber: Integer): Integer;
begin
  Result := DefaultInterface.WTO(pStrAxis, nNumber);
end;

function TE816.SWT(var pStrAxis: WideString; nIndex: Integer; dValue: Double): Integer;
begin
  Result := DefaultInterface.SWT(pStrAxis, nIndex, dValue);
end;

{$IFDEF LIVE_SERVER_AT_DESIGN_TIME}
constructor TE816Properties.Create(AServer: TE816);
begin
  inherited Create;
  FServer := AServer;
end;

function TE816Properties.GetDefaultInterface: IE816;
begin
  Result := FServer.DefaultInterface;
end;

{$ENDIF}

procedure Register;
begin
  RegisterComponents(dtlServerPage, [TE816]);
end;

end.
