unit UAboutBox;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, StdCtrls, ExtCtrls, Buttons, ComCtrls;

type
  TVersionEntry = record
    Version, Date, Changes: string;
  end;

const
  VERSION_HISTORY: array[0..4] of TVersionEntry = (
    (Version: '0.1.0'; Date: '202605'; Changes: 'Initial release'),
    (Version: ''; Date: ''; Changes: ''),
    (Version: ''; Date: ''; Changes: ''),
    (Version: ''; Date: ''; Changes: ''),
    (Version: ''; Date: ''; Changes: '')
  );

  APPLICATION_NAME = 'MBMaster';
  AUTHOR_NAME      = 'R.N. Cernic';
  COPYRIGHT        = '© 2026. All rights reserved.';
  DESCRIPTION      = 'Modbus RTU Master is ' +
                     'a lightweight, single-window Modbus RTU Master for serial devices,' +
                     ' written in Free Pascal / Lazarus (LCL). It reads and writes coils, ' +
                     'discrete inputs, holding registers, and input registers over an RS232/RS485 ' +
                     'serial link, with a built-in protocol bus monitor.';

type
  TAboutForm = class(TForm)
  private
    lblTitle       : TLabel;
    lblVersion     : TLabel;
    lblAuthor      : TLabel;
    lblCopyright   : TLabel;
    mmoDescription : TMemo;
    PageControl    : TPageControl;
    tabInfo        : TTabSheet;
    tabHistory     : TTabSheet;
    lvHistory      : TListView;
    btnOK          : TBitBtn;
    procedure BuildUI;
    procedure PopulateHistory;
    procedure FormShow(Sender: TObject);
  public
    constructor Create(AOwner: TComponent); override;
  end;

procedure ShowAboutBox;

implementation

procedure TAboutForm.PopulateHistory;
var
  I   : Integer;
  Item: TListItem;
begin
  lvHistory.Items.BeginUpdate;
  try
    lvHistory.Items.Clear;
    for I := Low(VERSION_HISTORY) to High(VERSION_HISTORY) do
    begin
      Item := lvHistory.Items.Add;
      Item.Caption := VERSION_HISTORY[I].Version;
      Item.SubItems.Add(VERSION_HISTORY[I].Date);
      Item.SubItems.Add(VERSION_HISTORY[I].Changes);
    end;
  finally
    lvHistory.Items.EndUpdate;
  end;
end;

// ← Triggered after the form and all controls are fully realized
procedure TAboutForm.FormShow(Sender: TObject);
begin
  PopulateHistory;
end;

procedure TAboutForm.BuildUI;
var
  col: TListColumn;
begin
  // --- Form ---
  Caption      := 'About ' + APPLICATION_NAME;
  BorderStyle  := bsDialog;
  Position     := poScreenCenter;
  Width        := 520;
  Height       := 400;
  Position     := poOwnerFormCenter;
  OnShow       := @FormShow;

  // --- PageControl ---
  PageControl := TPageControl.Create(Self);
  with PageControl do begin
    Parent  := Self;
    Left    := 10; Top := 10;
    Width   := Self.Width  - 28;
    Height  := Self.Height - 90;
    Anchors := [akLeft, akTop, akRight, akBottom];
  end;

  // ── Tab 1: Info ──────────────────────────────────────────
  tabInfo := TTabSheet.Create(PageControl);
  tabInfo.Parent  := PageControl;
  tabInfo.Caption := 'Information';

  lblTitle := TLabel.Create(Self);
  with lblTitle do begin
    Parent     := tabInfo;
    Caption    := APPLICATION_NAME;
    Font.Size  := 16;
    Font.Style := [fsBold];
    Left       := 16;
    Top        := 20;
  end;

  lblVersion := TLabel.Create(Self);
  with lblVersion do begin
    Parent  := tabInfo;
    Caption := 'Version: ' + VERSION_HISTORY[0].Version;
    Left    := 16;
    Top     := 58;
  end;

  lblAuthor := TLabel.Create(Self);
  with lblAuthor do begin
    Parent  := tabInfo;
    Caption := 'Author: ' + AUTHOR_NAME;
    Left    := 16;
    Top     := 80;
  end;

  lblCopyright := TLabel.Create(Self);
  with lblCopyright do begin
    Parent  := tabInfo;
    Caption := COPYRIGHT;
    Left    := 16;
    Top     := 102;
  end;

  mmoDescription := TMemo.Create(Self);
  with mmoDescription do begin
    Parent               := tabInfo;
    Text                 := DESCRIPTION;
    ReadOnly             := True;
    Left                 := 16;
    Top                  := 124;
    Width                := Self.Width - 64;
    Height               := Self.Height - 250;
    BorderSpacing.Around := 4;
  end;

  // ── Tab 2: Version History ────────────────────────────────
  tabHistory := TTabSheet.Create(PageControl);
  tabHistory.Parent  := PageControl;
  tabHistory.Caption := 'Version History';

  lvHistory := TListView.Create(Self);
  with lvHistory do begin
    Parent        := tabHistory;
    Left          := 4; Top := 4;
    Align         := alClient;
    ViewStyle     := vsReport;
    ReadOnly      := True;
    RowSelect     := True;
    GridLines     := True;
    HideSelection := False;

    col := Columns.Add; col.Caption := 'Version'; col.Width := 80;
    col := Columns.Add; col.Caption := 'Date';    col.Width := 100;
    col := Columns.Add; col.Caption := 'Changes'; col.Width := 280;
  end;

  // --- OK Button ---
  btnOK := TBitBtn.Create(Self);
  with btnOK do begin
    Parent      := Self;
    Kind        := bkOK;
    Caption     := 'OK';
    Width       := 90;
    Height      := 32;
    Left        := (Self.Width - Width) div 2;
    Top         := Self.Height - Height - 32;
    Anchors     := [akBottom];
    ModalResult := mrOK;
  end;
end;

constructor TAboutForm.Create(AOwner: TComponent);
begin
  inherited CreateNew(AOwner);
  BuildUI;
end;

procedure ShowAboutBox;
var
  F: TAboutForm;
begin
  F := TAboutForm.Create(nil);
  try
    F.ShowModal;
  finally
    F.Free;
  end;
end;

end.
