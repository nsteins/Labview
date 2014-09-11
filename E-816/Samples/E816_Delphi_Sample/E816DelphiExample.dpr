program E816DelphiExample;

uses
  Forms,
  UE816DelphiExample in 'UE816DelphiExample.pas' {Form1},
  E816_COMLib_TLB in 'E816_COMLib_TLB.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.
