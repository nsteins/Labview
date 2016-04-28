unit UE816DelphiExample;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls,E816_COMLib_TLB,ActiveX;

type
  TForm1 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Label1: TLabel;
    Memo1: TMemo;
    procedure Button2Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure Button1Click(Sender: TObject);
  private
    { Private-Deklarationen }
  public
    { Public-Deklarationen }
    E816             : TE816;
    pDoubleArray     : PSafeArray;
    pLongArray       : PSafeArray;
  end;

var
  Form1: TForm1;

implementation

{$R *.dfm}


procedure TForm1.Button2Click(Sender: TObject);
begin
  close;
end;

procedure TForm1.FormCreate(Sender: TObject);


begin
  E816:=TE816.Create(self);

end;

procedure TForm1.Button1Click(Sender: TObject);
var
  r,id             : integer;
  ws               : WideString;
  v                : double;
  l                : longint;
  index,i          : integer;
  iNChannels       : integer;
  SafeArrayBound : TSafeArrayBound;
begin
  ws:='';
  with E816 do
  begin
    id:=InterfaceSetupDlg(ws);
    if id=-1 then
    begin
      Application.MessageBox('Error','Connection failed');
      Exit;
    end;
    r:=GetError;
    if (r<>0) then
    begin
      ShowMessage(TranslateError(r));
      exit;
    end;
    r:=IsConnected;
    if (r=0) then
    begin
      Application.MessageBox('Error','Connection failed');
      Exit;
    end;
    r:=qIDN(ws);
    if (r=0) then
    begin
      r:=GetError;
      ShowMessage(TranslateError(r));
      Exit;
    end;
    Label1.Caption:=ws;
    r:=qSAI(ws);
    if (r=0) then
    begin
      r:=GetError;
      ShowMessage(TranslateError(r));
      Exit;
    end;
    iNChannels := Length(ws);
    SafeArrayBound.cElements:=iNChannels;
    SafeArrayBound.lLbound:=0;
    pDoubleArray:=SafeArrayCreate(VT_R8,1,SafeArrayBound);
    pLongArray  :=SafeArrayCreate(VT_I4,1,SafeArrayBound);
    for index:=0 to (iNChannels-1) do
    begin
      l:=1;
      r:=SafeArrayPutElement(pLongArray,index,l);
    end;
    r:=SVO(ws,pLongArray);
    if (r=0) then
    begin
      r:=GetError;
      ShowMessage(TranslateError(r));
    end;
    v:=0;
    for index:=0 to (iNChannels-1) do
    begin
      r:=SafeArrayPutElement(pDoubleArray,index,v);
    end   ;
    r:=MOV(ws,pDoubleArray);
    if (r=0) then
    begin
      r:=GetError;
      ShowMessage(TranslateError(r));
    end;
    r:=qPOS(ws,pDoubleArray);
    if (r=0) then
    begin
      r:=GetError;
      ShowMessage(TranslateError(r));
    end;
    for index:=0 to (iNChannels-1) do
    begin
      r:=SafeArrayGetElement(pDoubleArray,index,v);
      Memo1.Lines.Add(FloatToStr(v));
    end ;
    for i:=1 to 10 do
    begin
      v:=0.1;
      for index:=0 to (iNChannels-1) do
      begin
         v := v+(index*0.5);
         r:=SafeArrayPutElement(pDoubleArray,index,v);
      end          ;
      r:=MVR(ws,pDoubleArray);
      if (r=0) then
      begin
        r:=GetError;
        ShowMessage(TranslateError(r));
      end;
      sleep(10);
      r:=qPOS(ws,pDoubleArray);
      if (r=0) then
      begin
        r:=GetError;
        ShowMessage(TranslateError(r));
      end;
      for index:=0 to (iNChannels-1) do
      begin
        r:=SafeArrayGetElement(pDoubleArray,index,v);
        if (r=0) then
          Memo1.Lines.Add(IntToStr(index)+': '+FloatToStr(v))
        else
          ShowMessage('SafeArrayGetElement failed');
      end  ;
    end;
  end;
end;

end.
