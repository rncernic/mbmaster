unit umain;

{$mode objfpc}{$H+}

{ =============================================================================
  UMain — Modbus RTU Master main form
  =============================================================================

  DISCLAIMER: All documentation comments were generated with the assistance
  of Anthropic's Claude.

  OVERVIEW
  ────────
  This unit implements the complete GUI and control logic for a Modbus RTU
  Master.  It is the only form in the application and owns every runtime
  object: the serial port, the Modbus protocol engine, the polling timer,
  and the two dynamically-created display controls (the register grid and
  the bus monitor).

  ARCHITECTURE
  ────────────

    User interaction
      btnConnect  → open / close serial port
      btnScan     → start / stop continuous polling (any FC)
      btnSend     → one-shot execution of the selected FC
      grid edits  → values to write (write FCs only)
                              │
                              ▼
    RunTransaction(FC, StartAddr, Qty)
      Single dispatch point. Translates the current UI state into
      exactly one call on TModbusMaster, regardless of whether it was
      triggered by btnSend (once) or ScanTimerTick (repeatedly).
                              │
                              ▼
    TModbusMaster (UModbusMaster)
      Builds the request frame, sends it over TLazSerial, waits for the
      response, validates CRC + address, then fires OnTransaction
      synchronously on the main thread.
                              │ OnTransaction(ATx)
                              ▼
    OnTransaction
      • MonitorAppend  — logs the raw TX and RX frames as hex
      • PopulateReadGrid / PopulateWriteConfirmGrid — fills pnlGrid
      • increments FTxCount / FErrCount

  SNIFFER UNIT REUSE
  ──────────────────
    UModbusTypes   — FUNC_* constants, TByteArray, TModbusPacket,
                     GetFunctionName, GetExceptionName.
    UModbusUtils   — CRC16, IsValidCRC, BytesToHex, BigEndianWord,
                     LittleEndianWord, FormatTimestamp.
    UModbusParser  — ParsePacket (used inside UModbusMaster).
    UModbusMaster  — TModbusMaster, TModbusTransaction.

  SCAN CYCLE
  ──────────
  When scanning is active, FScanTimer fires every speScanRate ms.  Each
  tick disables the timer (no re-entry), runs one RunTransaction, then
  re-arms the timer only if scanning is still active.

  WRITE FLOW
  ──────────
  For write FCs the grid switches to an editable single-column
  "Value to Write" layout.  btnSend (or each scan tick) reads those cells
  via SafeCell and passes the values to the appropriate write method.

  THREADING
  ──────────
  TLazSerial delivers data on the main thread and TModbusMaster fires
  OnTransaction synchronously, so no locking is needed anywhere here.

  MEMORY OWNERSHIP
  ────────────────
  TModbusTransaction carries two TModbusPacket objects.  Ownership passes
  to the caller of RunTransaction, which frees them in its finally block
  after OnTransaction has consumed them.  Nil-guards keep the free safe.

  ============================================================================= }

interface

uses
  Classes, SysUtils, Math, StrUtils,
  Forms, Controls, Graphics, Dialogs,
  ExtCtrls, Buttons, Spin, StdCtrls, LedIndicator, LazSerial, Grids,
  UAboutBox,      { ShowAboutBox, APPLICATION_NAME, VERSION_HISTORY }
  UModbusTypes,   { TModbusPacket, TByteArray, FUNC_* constants }
  UModbusUtils,   { FormatTimestamp, BigEndianWord }
  UModbusMaster;  { TModbusMaster, TModbusTransaction }

{ =============================================================================
  Grid column indices  (FGrid always has 4 columns)
  ─────────────────────────────────────────────────
    READ mode             WRITE mode
    0  Address            0  Address
    1  Dec / Raw          1  Value to Write  (editable)
    2  Formatted          2  Status
    3  Hex                3  Hex                                          }
const
  GCOL_ADDR = 0;
  GCOL_RAW  = 1;
  GCOL_FMT  = 2;
  GCOL_HEX  = 3;

{ Display-format combo indices (cmbFormat.ItemIndex).
  FLOAT and INT32 consume two consecutive registers (a 32-bit pair). }
  FMT_DEC   = 0;   { Decimal — signed or unsigned per chkSigned }
  FMT_HEX   = 1;   { Hexadecimal, $XXXX }
  FMT_BIN   = 2;   { Binary, 16 digits }
  FMT_FLOAT = 3;   { IEEE-754 single precision over two registers }
  FMT_INT32 = 4;   { 32-bit integer over two registers }

{ Endian combo indices (cmbEndian.ItemIndex).
  Letters describe byte order A(MSB)…D(LSB). }
  END_ABCD  = 0;   { Big-Endian            — high word first, normal bytes  }
  END_CDAB  = 1;   { Mid-Big (word-swap)   — low word first, normal bytes   }
  END_BADC  = 2;   { Mid-Little (byte-swap)— high word first, bytes swapped }
  END_DCBA  = 3;   { Little-Endian         — low word first, bytes swapped  }

  { Cap on bus-monitor lines; oldest are trimmed past this. }
  MAX_MONITOR_LINES = 1000;

type
  { TfrmMain — the application's single form. }
  TfrmMain = class(TForm)
    { ── Toolbar buttons ── }
    btnScan             : TBitBtn;   { start/stop continuous polling }
    btnSend             : TBitBtn;   { one-shot transaction }
    btnClear            : TBitBtn;   { clear the bus monitor }
    btnConnect          : TBitBtn;   { open/close the serial port }
    btnCommConfig       : TBitBtn;   { LazSerial setup dialog }

    { ── Settings controls ── }
    chkSigned           : TCheckBox; { interpret decimals as signed }
    cmbEndian           : TComboBox; { 32-bit byte order }
    cmbStartAddressBase : TComboBox; { Decimal / Hex address entry mode }
    cmbFunctionCode     : TComboBox; { Modbus function code selector }
    cmbFormat           : TComboBox; { value display format }
    lblFormat           : TLabel;
    lblNumberRegisters  : TLabel;
    lblStartAddressBase : TLabel;
    lblStartAddress     : TLabel;
    lblFunctionCode     : TLabel;
    lblSlaveAddress     : TLabel;
    lblEndian           : TLabel;
    lblBaseAddress      : TLabel;
    lblResponseTime     : TLabel;
    lblScanRate         : TLabel;
    ledConnected        : TLedIndicator;  { green when port open }

    { ── Layout panels ── }
    pnlBusMonitor       : TPanel;    { hosts the FMonitor memo }
    pnlGrid             : TPanel;    { hosts the FGrid string grid }
    pnlToolBar          : TPanel;
    pnlSettings         : TPanel;
    pnlFunction         : TPanel;
    pnlRegisters        : TPanel;
    pnlStatusBar        : TPanel;    { packet/error counters }
    spbAbout            : TSpeedButton;

    { ── Spin edits ── }
    speResponseTime     : TSpinEdit; { per-request timeout, ms }
    speBaseAddress      : TSpinEdit; { 0/1 origin offset }
    speStartAddress     : TSpinEdit; { first register/coil address }
    speScanRate         : TSpinEdit; { scan interval, ms }
    speSlaveAddress     : TSpinEdit; { Modbus slave address 1–247 }
    speNumberRegisters  : TSpinEdit; { quantity to read/write }
    splRigth            : TSplitter;

    { ── Form lifecycle ── }
    procedure FormCreate (Sender: TObject);
    procedure FormDestroy(Sender: TObject);

    { ── Button events ── }
    procedure spbAboutClick     (Sender: TObject);
    procedure btnClearClick     (Sender: TObject);
    procedure btnConnectClick   (Sender: TObject);
    procedure btnCommConfigClick(Sender: TObject);
    procedure btnScanClick      (Sender: TObject);
    procedure btnSendClick      (Sender: TObject);

    { ── Settings change events (wired in FormCreate) ── }
    procedure cmbFunctionCodeChange      (Sender: TObject);
    procedure cmbFormatChange            (Sender: TObject);
    procedure speNumberRegistersChange   (Sender: TObject);
    procedure speStartAddressChange      (Sender: TObject);
    procedure cmbStartAddressBaseChange  (Sender: TObject);

  private
    { ── Runtime objects ── }
    FSerial    : TLazSerial;   { the serial port }
    FMaster    : TModbusMaster;{ the Modbus engine, wraps FSerial }
    FScanTimer : TTimer;       { drives continuous polling }
    FConnected : Boolean;      { True between Connect and Disconnect }
    FScanning  : Boolean;      { True while the scan timer is active }

    { ── Dynamically created controls ── }
    FGrid    : TStringGrid;    { register/value grid in pnlGrid }
    FMonitor : TMemo;          { bus monitor in pnlBusMonitor }

    { ── Statistics ── }
    FTxCount  : Integer;       { total transactions attempted }
    FErrCount : Integer;       { transactions that failed }

    { ── Reserved for future use (e.g. cached last response) ── }
    FLastResponse : TModbusPacket;

    { ── Construction helpers ── }
    procedure BuildGrid;
    procedure BuildMonitor;
    procedure InitCombos;
    procedure InitSpins;
    procedure RebuildGridRows;

    { ── State management ── }
    procedure SetConnected(AValue: Boolean);
    procedure SetScanning (AValue: Boolean);
    procedure UpdateStatus;
    procedure UpdateButtonStates;

    { ── UI accessors ── }
    function  GetFuncCode    : Byte;
    function  GetStartAddress: Word;
    function  IsReadFC       : Boolean;
    function  IsWriteFC      : Boolean;
    function  GetEndianIndex : Integer;

    { Safe grid cell reader — '' if the row is out of range. }
    function  SafeCell(ACol, ARow: Integer): string;

    { ── Unified transaction core (btnSend once / scan repeatedly) ── }
    procedure RunTransaction(AFC: Byte; AStartAddr, AQty: Word);

    { ── Master callback ── }
    procedure OnTransaction(const ATx: TModbusTransaction);

    { ── Grid population ── }
    procedure PopulateReadGrid         (AResp: TModbusPacket);
    procedure PopulateWriteConfirmGrid (const ATx: TModbusTransaction);

    { ── Bus monitor ── }
    procedure MonitorAppend(const ARaw: TByteArray;
                            AIsRequest: Boolean;
                            const ANote: string = '');

    { ── Scan timer handler ── }
    procedure ScanTimerTick(Sender: TObject);
  public
  end;

var
  frmMain: TfrmMain;

implementation

{$R *.lfm}

{ =============================================================================
  FormCreate / FormDestroy
  =============================================================================
  Order matters: build the dynamic controls BEFORE populating combos/spins,
  then wire OnChange handlers LAST so they cannot fire against half-built
  state during initialisation (InitCombos sets ItemIndex, which would reach
  RebuildGridRows; if FGrid were nil that would crash).
  ============================================================================= }

procedure TfrmMain.FormCreate(Sender: TObject);
begin
  frmMain.Caption := APPLICATION_NAME + ' v. ' + VERSION_HISTORY[0].Version +
    ' (' + VERSION_HISTORY[0].Date + ')';

  FConnected := False;
  FScanning  := False;
  FTxCount   := 0;
  FErrCount  := 0;
  FGrid      := nil;   { explicit nil so the RebuildGridRows guard works }
  FMonitor   := nil;

  { Serial port — fully qualified enum names required by this LazSerial build }
  FSerial             := TLazSerial.Create(Self);
  FSerial.BaudRate    := TBaudRate.br__9600;
  FSerial.DataBits    := TDataBits.db8bits;
  FSerial.StopBits    := TStopBits.sbOne;
  FSerial.Parity      := TParity.pNone;
  FSerial.FlowControl := TFlowControl.fcNone;

  { Modbus engine }
  FMaster                 := TModbusMaster.Create(FSerial);
  FMaster.ResponseTimeout := 1000;
  FMaster.OnTransaction   := @OnTransaction;

  { Scan timer — created disabled; started only via SetScanning(True) }
  FScanTimer          := TTimer.Create(Self);
  FScanTimer.Enabled  := False;
  FScanTimer.Interval := 1000;
  FScanTimer.OnTimer  := @ScanTimerTick;

  { Status bar }
  pnlStatusBar.Color   := $00CC9966;
  pnlStatusBar.Caption := 'Packets: 0 | Errors: 0';

  { Build dynamic controls before anything can reference them }
  BuildGrid;
  BuildMonitor;

  InitCombos;
  InitSpins;

  { Wire change events last }
  cmbFunctionCode.OnChange     := @cmbFunctionCodeChange;
  cmbFormat.OnChange           := @cmbFormatChange;
  speNumberRegisters.OnChange  := @speNumberRegistersChange;
  cmbStartAddressBase.OnChange := @cmbStartAddressBaseChange;
  speStartAddress.OnChange     := @speStartAddressChange;

  RebuildGridRows;
  UpdateButtonStates;
end;

procedure TfrmMain.FormDestroy(Sender: TObject);
{ Stop the timer, close the port if open, free the engine.
  FSerial and FScanTimer are owned by the form and freed by the LCL. }
begin
  FScanTimer.Enabled := False;
  if FSerial.Active  then FSerial.Active := False;
  FMaster.Free;
end;

{ =============================================================================
  Clear — empty the bus monitor
  ============================================================================= }

procedure TfrmMain.btnClearClick(Sender: TObject);
begin
  if FMonitor <> nil then FMonitor.Clear;
end;

{ =============================================================================
  Combo and spin initialisation
  ============================================================================= }

procedure TfrmMain.InitCombos;
begin
  with cmbFunctionCode do
  begin
    Items.Clear;
    Items.Add('01 – Read Coils');
    Items.Add('02 – Read Discrete Inputs');
    Items.Add('03 – Read Holding Registers');
    Items.Add('04 – Read Input Registers');
    Items.Add('05 – Write Single Coil');
    Items.Add('06 – Write Single Register');
    Items.Add('15 – Write Multiple Coils');
    Items.Add('16 – Write Multiple Registers');
    ItemIndex := 2;   { Read Holding Registers — most common operation }
  end;

  with cmbFormat do
  begin
    Items.Clear;
    Items.Add('Decimal');
    Items.Add('Hex');
    Items.Add('Binary');
    Items.Add('Float 32');
    Items.Add('Int 32');
    ItemIndex := 0;
  end;

  with cmbEndian do
  begin
    Items.Clear;
    Items.Add('AB CD  (Big-Endian)');
    Items.Add('CD AB  (Mid-Big)');
    Items.Add('BA DC  (Mid-Little)');
    Items.Add('DC BA  (Little-Endian)');
    ItemIndex := 0;
  end;

  with cmbStartAddressBase do
  begin
    Items.Clear;
    Items.Add('Decimal');
    Items.Add('Hex');
    ItemIndex := 0;
  end;
end;

procedure TfrmMain.InitSpins;
{ speBaseAddress is capped at 1: it is a 0/1 origin offset, not an address. }
begin
  speSlaveAddress.MinValue    := 1;
  speSlaveAddress.MaxValue    := 247;
  speSlaveAddress.Value       := 1;

  speStartAddress.MinValue    := 0;
  speStartAddress.MaxValue    := 65535;
  speStartAddress.Value       := 0;

  speNumberRegisters.MinValue := 1;
  speNumberRegisters.MaxValue := 125;
  speNumberRegisters.Value    := 10;

  speBaseAddress.MinValue     := 0;
  speBaseAddress.MaxValue     := 1;
  speBaseAddress.Value        := 0;

  speResponseTime.MinValue    := 100;
  speResponseTime.MaxValue    := 10000;
  speResponseTime.Value       := 1000;

  speScanRate.MinValue        := 100;
  speScanRate.MaxValue        := 60000;
  speScanRate.Value           := 1000;
end;

{ =============================================================================
  Dynamic control creation
  ============================================================================= }

procedure TfrmMain.BuildGrid;
begin
  FGrid                  := TStringGrid.Create(Self);
  FGrid.Parent           := pnlGrid;
  FGrid.Align            := alClient;
  FGrid.RowCount         := 2;
  FGrid.FixedRows        := 1;
  FGrid.FixedCols        := 0;
  FGrid.ColCount         := 4;
  FGrid.DefaultRowHeight := 18;
  FGrid.RowHeights[0]    := 20;
  FGrid.Options          := FGrid.Options
                            + [goRowSelect, goColSizing, goThumbTracking]
                            - [goEditing, goAlwaysShowEditor];

  { Dark theme — matches the companion sniffer }
  FGrid.Color          := $001E1E1E;
  FGrid.Font.Color     := $00CCCCCC;
  FGrid.Font.Name      := 'Consolas';
  FGrid.Font.Size      := 9;
  FGrid.FixedColor     := $00262525;
  FGrid.GridLineColor  := $00333333;

  FGrid.ColWidths[GCOL_ADDR] := 80;
  FGrid.ColWidths[GCOL_RAW]  := 90;
  FGrid.ColWidths[GCOL_FMT]  := 130;
  FGrid.ColWidths[GCOL_HEX]  := 70;

  FGrid.Cells[GCOL_ADDR, 0] := 'Address';
  FGrid.Cells[GCOL_RAW,  0] := 'Dec / Raw';
  FGrid.Cells[GCOL_FMT,  0] := 'Formatted';
  FGrid.Cells[GCOL_HEX,  0] := 'Hex';
end;

procedure TfrmMain.BuildMonitor;
begin
  FMonitor            := TMemo.Create(Self);
  FMonitor.Parent     := pnlBusMonitor;
  FMonitor.Align      := alClient;
  FMonitor.ReadOnly   := True;
  FMonitor.ScrollBars := ssVertical;
  FMonitor.WordWrap   := False;
  FMonitor.Font.Name  := 'Consolas';
  FMonitor.Font.Size  := 9;
  FMonitor.Color      := $001E1E1E;
  FMonitor.Font.Color := $00CCCCCC;
end;

{ =============================================================================
  Grid row rebuild
  =============================================================================
  READ mode  → four informational columns, no editing.
  WRITE mode → "Dec / Raw" becomes an editable "Value to Write" column.
  The address column is pre-filled in the user-selected base (dec / hex).
  ============================================================================= }

procedure TfrmMain.RebuildGridRows;
var
  N, I      : Integer;
  AddrBase  : Integer;
  WriteMode : Boolean;
  HexMode   : Boolean;

  function FormatAddr(AAddr: Integer): string;
  begin
    if HexMode then
      Result := Format('0x%.04X', [AAddr])
    else
      Result := IntToStr(AAddr);
  end;

begin
  if FGrid = nil then Exit;   { guard: may be called before BuildGrid }

  N         := speNumberRegisters.Value;
  HexMode   := cmbStartAddressBase.ItemIndex = 1;
  AddrBase  := speBaseAddress.Value + Integer(GetStartAddress);
  WriteMode := IsWriteFC;

  FGrid.BeginUpdate;
  try
    if WriteMode then
    begin
      FGrid.Cells[GCOL_ADDR, 0] := 'Address';
      FGrid.Cells[GCOL_RAW,  0] := 'Value to Write';
      FGrid.Cells[GCOL_FMT,  0] := 'Status';
      FGrid.Cells[GCOL_HEX,  0] := 'Hex';
      FGrid.Options := FGrid.Options + [goEditing] - [goAlwaysShowEditor];
      FGrid.Col     := GCOL_RAW;
    end
    else
    begin
      FGrid.Cells[GCOL_ADDR, 0] := 'Address';
      FGrid.Cells[GCOL_RAW,  0] := 'Dec / Raw';
      FGrid.Cells[GCOL_FMT,  0] := 'Formatted';
      FGrid.Cells[GCOL_HEX,  0] := 'Hex';
      FGrid.Options := FGrid.Options - [goEditing, goAlwaysShowEditor];
    end;

    FGrid.RowCount := N + 1;

    for I := 0 to N - 1 do
    begin
      FGrid.Cells[GCOL_ADDR, I+1] := FormatAddr(AddrBase + I);
      FGrid.Cells[GCOL_RAW,  I+1] := IfThen(WriteMode, '0', '');
      FGrid.Cells[GCOL_FMT,  I+1] := '';
      FGrid.Cells[GCOL_HEX,  I+1] := '';
    end;
  finally
    FGrid.EndUpdate;
  end;
end;

{ =============================================================================
  Button state management
  =============================================================================
  Single chokepoint deriving every button state from FConnected/FScanning.
    • Connect always clickable; caption reflects FConnected.
    • Scan requires a connected port; caption reflects FScanning.
    • Send requires connected AND not scanning.
    • All settings locked while scanning so loop parameters stay stable.
  ============================================================================= }

procedure TfrmMain.UpdateButtonStates;
begin
  btnConnect.Caption := IfThen(FConnected, 'Disconnect', 'Connect');

  btnScan.Enabled := FConnected;
  btnScan.Caption := IfThen(FScanning, 'Stop', 'Scan');

  btnSend.Enabled := FConnected and (not FScanning);

  cmbFunctionCode.Enabled     := not FScanning;
  speSlaveAddress.Enabled     := not FScanning;
  speStartAddress.Enabled     := not FScanning;
  speNumberRegisters.Enabled  := not FScanning;
  speBaseAddress.Enabled      := not FScanning;
  speScanRate.Enabled         := not FScanning;
  speResponseTime.Enabled     := not FScanning;
  cmbStartAddressBase.Enabled := not FScanning;
end;

{ =============================================================================
  Connect / Disconnect — serial port only
  =============================================================================
  Connect does NOT start scanning.  Disconnect always stops any active scan
  first so the timer can never fire against a closed port.
  ============================================================================= }

procedure TfrmMain.btnConnectClick(Sender: TObject);
begin
  if FConnected then
  begin
    SetScanning(False);
    FSerial.Active := False;
    SetConnected(False);
  end
  else
  begin
    if FSerial.Device = '' then
    begin
      ShowMessage('Please configure the serial port first (Comm Config).');
      Exit;
    end;
    FMaster.ResponseTimeout := speResponseTime.Value;
    try
      FSerial.Active := True;
    except
      on E: Exception do
      begin
        ShowMessage('Could not open port: ' + E.Message);
        Exit;
      end;
    end;
    SetConnected(True);
  end;
end;

procedure TfrmMain.btnCommConfigClick(Sender: TObject);
{ Delegates to LazSerial's own modal setup dialog, then reflects the choice
  in the status bar.  ConstsBaud[] maps TBaudRate → integer for display. }
begin
  FSerial.ShowSetupDialog;
  if FSerial.Device <> '' then
    pnlStatusBar.Caption :=
      Format('Ready — %s @ %d baud | Packets: %d | Errors: %d',
             [FSerial.Device, ConstsBaud[FSerial.BaudRate],
              FTxCount, FErrCount])
  else
    pnlStatusBar.Caption := 'Ready — click Comm Config to select port';
end;

procedure TfrmMain.SetConnected(AValue: Boolean);
begin
  FConnected := AValue;
  ledConnected.Active := AValue;
  if not AValue then
    SetScanning(False);
  UpdateButtonStates;
end;

{ =============================================================================
  Safe grid cell reader
  =============================================================================
  Returns the trimmed cell text, or '' if the row is out of range, so write
  paths stay robust when the grid has fewer rows than expected.
  ============================================================================= }

function TfrmMain.SafeCell(ACol, ARow: Integer): string;
begin
  if (FGrid <> nil) and (ARow >= 0) and (ARow < FGrid.RowCount) then
    Result := Trim(FGrid.Cells[ACol, ARow])
  else
    Result := '';
end;

{ =============================================================================
  Unified transaction core
  =============================================================================
  The ONLY place that maps a function code to an engine method, so btnSend
  and the scan timer behave identically for every FC — the sole difference
  is repetition.  Write values come from the grid's GCOL_RAW column.
  TModbusMaster fires OnTransaction synchronously inside the call, so the
  monitor and grid are already updated by the time control returns here.
  ============================================================================= }

procedure TfrmMain.RunTransaction(AFC: Byte; AStartAddr, AQty: Word);
var
  ATx    : TModbusTransaction;
  Values : array of Word;
  I      : Integer;
begin
  ATx.Request  := nil;
  ATx.Response := nil;
  FMaster.ResponseTimeout := speResponseTime.Value;

  try
    case AFC of
      { ── Reads ── }
      FUNC_READ_COILS,
      FUNC_READ_DISCRETE_INPUTS,
      FUNC_READ_HOLDING_REGS,
      FUNC_READ_INPUT_REGS:
        FMaster.PollRead(speSlaveAddress.Value, AFC,
                         AStartAddr, AQty, ATx);

      { ── Write single coil ── }
      FUNC_WRITE_SINGLE_COIL:
        FMaster.WriteSingleCoil(
          speSlaveAddress.Value, AStartAddr,
          SafeCell(GCOL_RAW, 1) = '1',
          ATx);

      { ── Write single register ── }
      FUNC_WRITE_SINGLE_REG:
        FMaster.WriteSingleRegister(
          speSlaveAddress.Value, AStartAddr,
          Word(StrToIntDef(SafeCell(GCOL_RAW, 1), 0)),
          ATx);

      { ── Write multiple coils / registers ── }
      FUNC_WRITE_MULTIPLE_COILS,
      FUNC_WRITE_MULTIPLE_REGS:
        begin
          SetLength(Values, AQty);
          for I := 0 to AQty - 1 do
            Values[I] := Word(StrToIntDef(SafeCell(GCOL_RAW, I + 1), 0));
          FMaster.WriteMultipleRegisters(
            speSlaveAddress.Value, AStartAddr, AQty, Values, ATx);
        end;
    else
      Exit;   { unknown FC — nothing to do }
    end;
  finally
    if ATx.Request  <> nil then ATx.Request.Free;
    if ATx.Response <> nil then ATx.Response.Free;
    UpdateStatus;
  end;
end;

{ =============================================================================
  Scan — continuous polling of the selected function code
  ============================================================================= }

procedure TfrmMain.SetScanning(AValue: Boolean);
begin
  FScanning          := AValue;
  FScanTimer.Enabled := AValue;
  if AValue then
    FScanTimer.Interval := speScanRate.Value;
  UpdateButtonStates;
end;

procedure TfrmMain.btnScanClick(Sender: TObject);
begin
  if FScanning then
    SetScanning(False)
  else
  begin
    if not FConnected then Exit;
    SetScanning(True);
  end;
end;

procedure TfrmMain.ScanTimerTick(Sender: TObject);
{ One scan iteration.  The timer is disabled first so a slow transaction
  cannot cause overlapping ticks; re-armed afterwards only if still active. }
begin
  FScanTimer.Enabled := False;
  try
    RunTransaction(GetFuncCode, GetStartAddress, speNumberRegisters.Value);
  finally
    if FScanning then
    begin
      FScanTimer.Interval := speScanRate.Value;
      FScanTimer.Enabled  := True;
    end;
  end;
end;

{ =============================================================================
  Send — one-shot execution of the selected function code
  ============================================================================= }

procedure TfrmMain.btnSendClick(Sender: TObject);
begin
  if not FConnected then Exit;
  if FScanning      then Exit;
  RunTransaction(GetFuncCode, GetStartAddress, speNumberRegisters.Value);
end;

{ =============================================================================
  Master callback
  =============================================================================
  Fired synchronously by TModbusMaster.Execute on the main thread for every
  transaction.  Logs TX/RX to the monitor, fills the grid, and updates the
  packet/error counters.
  ============================================================================= }

procedure TfrmMain.OnTransaction(const ATx: TModbusTransaction);
begin
  if ATx.Request <> nil then
    MonitorAppend(ATx.Request.Raw, True);

  if ATx.Response <> nil then
    MonitorAppend(ATx.Response.Raw, False,
      IfThen(ATx.Response.ValidCRC, '', '  ← BAD CRC'))
  else if not ATx.Success then
    MonitorAppend(nil, False, '  ← ' + ATx.ErrorMsg);

  if ATx.Success and (ATx.Response <> nil) then
  begin
    if IsReadFC then
      PopulateReadGrid(ATx.Response)
    else
      PopulateWriteConfirmGrid(ATx);
  end;

  Inc(FTxCount);
  if not ATx.Success then Inc(FErrCount);
end;

{ =============================================================================
  Grid population — READ
  =============================================================================
  Safety rules:
    1. RowCount = N+1 before any cell write — indices 1..N always valid.
    2. A 32-bit pair writes row I+2 only when I+1<N AND I+2<=N; a lone
       trailing register falls back to a 16-bit display.
    3. AResp nil-guarded at entry.
    4. Address column matches the decimal/hex base the user selected.
  ============================================================================= }

procedure TfrmMain.PopulateReadGrid(AResp: TModbusPacket);
var
  N, I       : Integer;
  AddrBase   : Integer;
  FC         : Byte;
  IsCoil     : Boolean;
  IsDouble   : Boolean;
  Fmt, Endx  : Integer;
  Signed     : Boolean;
  HexMode    : Boolean;
  RawHi      : Word;
  RawLo      : Word;
  Raw32      : LongInt;
  U32        : LongWord;
  RawVal     : Word;
  FmtStr     : string;
  HexStr     : string;
  HasPair    : Boolean;

  function FormatAddr(AAddr: Integer): string;
  begin
    if HexMode then
      Result := Format('0x%.04X', [AAddr])
    else
      Result := IntToStr(AAddr);
  end;

begin
  if AResp = nil then Exit;

  FC       := GetFuncCode;
  IsCoil   := FC in [FUNC_READ_COILS, FUNC_READ_DISCRETE_INPUTS];
  Fmt      := cmbFormat.ItemIndex;
  Endx     := GetEndianIndex;
  Signed   := chkSigned.Checked;
  HexMode  := cmbStartAddressBase.ItemIndex = 1;
  IsDouble := (not IsCoil) and (Fmt in [FMT_FLOAT, FMT_INT32]);
  AddrBase := speBaseAddress.Value + Integer(GetStartAddress);
  N        := speNumberRegisters.Value;

  { Pre-initialise to silence compiler hints }
  U32    := 0;
  Raw32  := 0;
  RawHi  := 0;
  RawLo  := 0;
  FmtStr := '';
  HexStr := '';

  FGrid.BeginUpdate;
  try
    FGrid.RowCount := N + 1;

    I := 0;
    while I < N do
    begin
      FGrid.Cells[GCOL_ADDR, I+1] := FormatAddr(AddrBase + I);

      { ── Coil / Discrete Input — single bit ── }
      if IsCoil then
      begin
        if TModbusMaster.GetCoilBit(AResp, I) then
          FmtStr := 'ON  [1]'
        else
          FmtStr := 'OFF [0]';
        FGrid.Cells[GCOL_RAW, I+1] := '';
        FGrid.Cells[GCOL_FMT, I+1] := FmtStr;
        FGrid.Cells[GCOL_HEX, I+1] := '';
        Inc(I);
        Continue;
      end;

      RawVal := TModbusMaster.GetRegisterWord(AResp, I);

      HasPair := IsDouble and (I + 1 < N) and (I + 2 <= N);

      if HasPair then
      begin
        RawHi  := TModbusMaster.GetRegisterWord(AResp, I);
        RawLo  := TModbusMaster.GetRegisterWord(AResp, I + 1);
        HexStr := Format('%04X %04X', [RawHi, RawLo]);

        if Fmt = FMT_FLOAT then
        begin
          case Endx of
            END_ABCD: FmtStr := Format('%.6g',
                         [TModbusMaster.GetRegisterFloat_ABCD(AResp, I)]);
            END_CDAB: FmtStr := Format('%.6g',
                         [TModbusMaster.GetRegisterFloat_CDAB(AResp, I)]);
            END_BADC: FmtStr := Format('%.6g',
                         [TModbusMaster.GetRegisterFloat_BADC(AResp, I)]);
            END_DCBA: FmtStr := Format('%.6g',
                         [TModbusMaster.GetRegisterFloat_DCBA(AResp, I)]);
          else
            FmtStr := '?';
          end;
        end
        else
        begin
          { Int 32 — assemble the 32-bit word per the chosen byte order }
          case Endx of
            END_ABCD: U32 := (LongWord(RawHi) shl 16) or LongWord(RawLo);
            END_CDAB: U32 := (LongWord(RawLo) shl 16) or LongWord(RawHi);
            END_BADC:
              begin
                RawHi := Word(((RawHi and $FF) shl 8) or (RawHi shr 8));
                RawLo := Word(((RawLo and $FF) shl 8) or (RawLo shr 8));
                U32   := (LongWord(RawHi) shl 16) or LongWord(RawLo);
              end;
            END_DCBA: U32 := (LongWord(RawLo) shl 16) or LongWord(RawHi);
          else
            U32 := (LongWord(RawHi) shl 16) or LongWord(RawLo);
          end;
          Raw32 := LongInt(U32);
          if Signed then FmtStr := IntToStr(Raw32)
          else           FmtStr := IntToStr(U32);
        end;

        FGrid.Cells[GCOL_RAW, I+1] := IntToStr(RawHi);
        FGrid.Cells[GCOL_FMT, I+1] := FmtStr;
        FGrid.Cells[GCOL_HEX, I+1] := HexStr;

        FGrid.Cells[GCOL_ADDR, I+2] := FormatAddr(AddrBase + I + 1) + ' *';
        FGrid.Cells[GCOL_RAW,  I+2] := IntToStr(RawLo);
        FGrid.Cells[GCOL_FMT,  I+2] := '(32-bit pair)';
        FGrid.Cells[GCOL_HEX,  I+2] := '';
        Inc(I, 2);
      end
      else
      begin
        FmtStr := TModbusMaster.FormatValue(RawVal, Signed, Fmt);
        HexStr := Format('%04X', [RawVal]);
        FGrid.Cells[GCOL_RAW, I+1] := IntToStr(RawVal);
        FGrid.Cells[GCOL_FMT, I+1] := FmtStr;
        FGrid.Cells[GCOL_HEX, I+1] := HexStr;
        Inc(I);
      end;
    end;
  finally
    FGrid.EndUpdate;
  end;
end;

{ =============================================================================
  Grid population — WRITE confirm
  =============================================================================
  After a write the grid shows whether it succeeded: every value row's
  Status column is stamped with a tick or an error message.
  ============================================================================= }

procedure TfrmMain.PopulateWriteConfirmGrid(const ATx: TModbusTransaction);
var
  I, N: Integer;
  Mark: string;
begin
  N    := speNumberRegisters.Value;
  Mark := IfThen(ATx.Success, '✓ Sent', '✗ Error: ' + ATx.ErrorMsg);
  FGrid.BeginUpdate;
  try
    for I := 1 to Min(N, FGrid.RowCount - 1) do
      FGrid.Cells[GCOL_FMT, I] := Mark;
  finally
    FGrid.EndUpdate;
  end;
end;

{ =============================================================================
  Bus monitor — append one timestamped TX or RX line
  =============================================================================
  Example output:
    [14:35:07.042] TX  01 03 00 00 00 0A C5 CD
    [14:35:07.061] RX  01 83 02 C0 F1            ← BAD CRC

  ARaw may be nil for an error-only line; Length(nil)=0 so the hex loop
  produces nothing.  Oldest lines are trimmed before appending; the caret
  is moved to the last line for a handle-safe auto-scroll.
  ============================================================================= }

procedure TfrmMain.MonitorAppend(const ARaw: TByteArray;
  AIsRequest: Boolean; const ANote: string);
var
  S  : string;
  I  : Integer;
  TS : string;
begin
  if FMonitor = nil then Exit;

  TS := FormatTimestamp(Now);

  if AIsRequest then
    S := '[' + TS + '] TX  '
  else
    S := '[' + TS + '] RX  ';

  for I := 0 to Length(ARaw) - 1 do
    S := S + Format('%02X ', [ARaw[I]]);

  if ANote <> '' then
    S := S + ANote;

  FMonitor.Lines.BeginUpdate;
  try
    while FMonitor.Lines.Count >= MAX_MONITOR_LINES do
      FMonitor.Lines.Delete(0);
    FMonitor.Lines.Add(S);
  finally
    FMonitor.Lines.EndUpdate;
  end;

  if FMonitor.Lines.Count > 0 then
    FMonitor.CaretPos := Point(0, FMonitor.Lines.Count - 1);
end;

{ =============================================================================
  Settings change events
  ============================================================================= }

procedure TfrmMain.cmbFunctionCodeChange(Sender: TObject);
{ Format/endian/signed only apply to register reads; write-single FCs are
  pinned to quantity 1.  Then refreshes buttons and rebuilds the grid. }
var
  FC     : Byte;
  IsCoil : Boolean;
  IsRead : Boolean;
begin
  FC     := GetFuncCode;
  IsCoil := FC in [FUNC_READ_COILS, FUNC_READ_DISCRETE_INPUTS];
  IsRead := IsReadFC;

  cmbFormat.Enabled := IsRead and (not IsCoil);
  cmbEndian.Enabled := cmbFormat.Enabled and
                       (cmbFormat.ItemIndex in [FMT_FLOAT, FMT_INT32]);
  chkSigned.Enabled := cmbFormat.Enabled;

  if FC in [FUNC_WRITE_SINGLE_COIL, FUNC_WRITE_SINGLE_REG] then
  begin
    speNumberRegisters.MaxValue := 1;
    speNumberRegisters.Value    := 1;
  end
  else
    speNumberRegisters.MaxValue := 125;

  UpdateButtonStates;
  RebuildGridRows;
end;

procedure TfrmMain.cmbFormatChange(Sender: TObject);
{ Re-renders the Formatted column in place from the raw values already in
  the grid, so dec/hex/bin can be switched without re-reading the device. }
var
  I      : Integer;
  RawVal : Word;
  Fmt    : Integer;
  Signed : Boolean;
begin
  cmbEndian.Enabled := cmbFormat.ItemIndex in [FMT_FLOAT, FMT_INT32];

  Signed := chkSigned.Checked;
  Fmt    := cmbFormat.ItemIndex;

  FGrid.BeginUpdate;
  try
    for I := 1 to speNumberRegisters.Value do
    begin
      RawVal := StrToIntDef(FGrid.Cells[GCOL_RAW, I], 0);
      FGrid.Cells[GCOL_FMT, I] :=
        TModbusMaster.FormatValue(RawVal, Signed, Fmt);
    end;
  finally
    FGrid.EndUpdate;
  end;
end;

procedure TfrmMain.spbAboutClick(Sender: TObject);
begin
  ShowAboutBox;
end;

procedure TfrmMain.speNumberRegistersChange(Sender: TObject);
begin
  RebuildGridRows;
end;

procedure TfrmMain.speStartAddressChange(Sender: TObject);
begin
  RebuildGridRows;
end;

procedure TfrmMain.cmbStartAddressBaseChange(Sender: TObject);
begin
  RebuildGridRows;
end;

{ =============================================================================
  UI accessors
  ============================================================================= }

function TfrmMain.GetFuncCode: Byte;
{ Maps the combo selection to a FUNC_* constant; default Read Holding Regs. }
begin
  case cmbFunctionCode.ItemIndex of
    0: Result := FUNC_READ_COILS;
    1: Result := FUNC_READ_DISCRETE_INPUTS;
    2: Result := FUNC_READ_HOLDING_REGS;
    3: Result := FUNC_READ_INPUT_REGS;
    4: Result := FUNC_WRITE_SINGLE_COIL;
    5: Result := FUNC_WRITE_SINGLE_REG;
    6: Result := FUNC_WRITE_MULTIPLE_COILS;
    7: Result := FUNC_WRITE_MULTIPLE_REGS;
  else
    Result := FUNC_READ_HOLDING_REGS;
  end;
end;

function TfrmMain.GetStartAddress: Word;
{ In hex mode the spin's decimal digits are re-interpreted as hex, so
  typing 100 with Hex selected addresses register 0x100 (256). }
var
  S: string;
begin
  if cmbStartAddressBase.ItemIndex = 1 then
  begin
    S      := IntToStr(speStartAddress.Value);
    Result := Word(StrToIntDef('$' + S, 0));
  end
  else
    Result := Word(speStartAddress.Value);
end;

function TfrmMain.IsReadFC: Boolean;
begin
  Result := GetFuncCode in [FUNC_READ_COILS, FUNC_READ_DISCRETE_INPUTS,
                             FUNC_READ_HOLDING_REGS, FUNC_READ_INPUT_REGS];
end;

function TfrmMain.IsWriteFC: Boolean;
begin
  Result := not IsReadFC;
end;

function TfrmMain.GetEndianIndex: Integer;
begin
  Result := cmbEndian.ItemIndex;
  if Result < 0 then Result := END_ABCD;
end;

procedure TfrmMain.UpdateStatus;
{ Refreshes the status bar totals and tints it red once any error occurs. }
begin
  pnlStatusBar.Caption :=
    Format('Packets: %d | Errors: %d', [FTxCount, FErrCount]);
  if FErrCount > 0 then
    pnlStatusBar.Color := $002020AA   { red tint }
  else
    pnlStatusBar.Color := $00CC9966;  { amber }
end;

end.
