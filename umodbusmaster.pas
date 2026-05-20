unit UModbusMaster;

{$mode objfpc}{$H+}

{ =============================================================================
  UModbusMaster — Modbus RTU Master engine
  =============================================================================
  PURPOSE
  ───────
  Implements TModbusMaster: a blocking, single-threaded Modbus RTU master
  that builds request frames, sends them over TLazSerial, waits for a
  response within a configurable timeout, validates the response CRC and
  slave address, and returns a TModbusTransaction record containing both
  the raw request and response packets (as TModbusPacket objects, fully
  compatible with the sniffer architecture).

  LAZSERIAL API NOTES
  ───────────────────
  LazSerial exposes only a string-based serial API:
    • WriteData(const AData: string)  — send bytes (AnsiString = raw bytes)
    • ReadData: string                — read all available bytes
    • Active: Boolean                 — port open/close state
  There is no binary Read(buf,len) or FlushBuffer method.
  This unit converts TByteArray ↔ AnsiString for every transfer.

  REUSE OF SNIFFER UNITS
  ───────────────────────
  • UModbusTypes  — FUNC_* constants, TByteArray, TModbusPacket
  • UModbusUtils  — CRC16, BigEndianWord, FormatTimestamp
  • UModbusParser — ParsePacket (populates Description, Data, etc.)

  ============================================================================= }

interface

uses
  Classes, SysUtils, LazSerial,
  UModbusTypes,   { TByteArray, TModbusPacket, FUNC_* constants }
  UModbusUtils,   { CRC16, BigEndianWord }
  UModbusParser;  { ParsePacket }

type

  { TModbusTransaction — result of one master poll cycle.
    The caller owns both TModbusPacket objects and must free them. }
  TModbusTransaction = record
    Request  : TModbusPacket;  { TX frame — always created }
    Response : TModbusPacket;  { RX frame — nil on timeout }
    Success  : Boolean;
    ErrorMsg : string;
  end;

  { Callback fired synchronously inside Execute (on the main thread) }
  TOnTransactionEvent = procedure(const ATx: TModbusTransaction) of object;

  { TModbusMaster }
  TModbusMaster = class
  private
    FSerial         : TLazSerial;
    FResponseTimeout: Integer;
    FPacketCount    : Integer;
    FErrorCount     : Integer;
    FOnTransaction  : TOnTransactionEvent;

    { Frame building — all return a complete frame with CRC appended }
    function  AppendCRC(const AData: TByteArray): TByteArray;
    function  BuildReadRequest(ASlaveAddr, AFuncCode: Byte;
                               AStartAddr, AQuantity: Word): TByteArray;
    function  BuildWriteSingleCoilRequest(ASlaveAddr: Byte;
                                          ACoilAddr: Word;
                                          AValue: Boolean): TByteArray;
    function  BuildWriteSingleRegRequest(ASlaveAddr: Byte;
                                         ARegAddr, AValue: Word): TByteArray;
    function  BuildWriteMultipleRegsRequest(ASlaveAddr: Byte;
                                            AStartAddr, AQuantity: Word;
                                            const AValues: array of Word): TByteArray;

    { Wire I/O using LazSerial string API }
    procedure SendFrame(const AData: TByteArray);
    procedure DrainRxBuffer;
    function  ReadResponse(AFuncCode: Byte; AQuantity: Word;
                           out AResponse: TByteArray): Boolean;

    { Expected response length for a given FC and quantity }
    function  ExpectedResponseLength(AFuncCode: Byte;
                                     AQuantity: Word): Integer;

    { Core transaction: send ARequest, collect response, parse, validate }
    function  Execute(const ARequest: TByteArray; AFuncCode: Byte;
                      AQuantity: Word;
                      out ATx: TModbusTransaction): Boolean;

  public
    constructor Create(ASerial: TLazSerial);
    destructor  Destroy; override;

    { ── Read operations ── }
    function PollRead(ASlaveAddr, AFuncCode: Byte;
                      AStartAddr, AQuantity: Word;
                      out ATx: TModbusTransaction): Boolean;

    { ── Write operations ── }
    function WriteSingleCoil(ASlaveAddr: Byte; ACoilAddr: Word;
                             AValue: Boolean;
                             out ATx: TModbusTransaction): Boolean;

    function WriteSingleRegister(ASlaveAddr: Byte;
                                 ARegAddr, AValue: Word;
                                 out ATx: TModbusTransaction): Boolean;

    function WriteMultipleRegisters(ASlaveAddr: Byte;
                                    AStartAddr, AQuantity: Word;
                                    const AValues: array of Word;
                                    out ATx: TModbusTransaction): Boolean;

    { ── Value decoding helpers (class methods — no instance needed) ── }

    { Read a 16-bit register from a parsed RX packet.
      ARegIndex is 0-based relative to the first returned register.
      Data[0] = ByteCount; registers start at Data[1]. }
    class function GetRegisterWord(const APkt: TModbusPacket;
                                   ARegIndex: Integer): Word;

    { Read a 32-bit IEEE-754 float from two consecutive registers.
      Four byte orders are supported to cover all common PLC implementations. }
    class function GetRegisterFloat_ABCD(const APkt: TModbusPacket;
                                         ARegIndex: Integer): Single;
    class function GetRegisterFloat_CDAB(const APkt: TModbusPacket;
                                         ARegIndex: Integer): Single;
    class function GetRegisterFloat_BADC(const APkt: TModbusPacket;
                                         ARegIndex: Integer): Single;
    class function GetRegisterFloat_DCBA(const APkt: TModbusPacket;
                                         ARegIndex: Integer): Single;

    { Read a single coil/discrete-input bit from a parsed RX packet.
      ACoilIndex is 0-based. }
    class function GetCoilBit(const APkt: TModbusPacket;
                              ACoilIndex: Integer): Boolean;

    { Format a 16-bit word for display.
      AFormat: 0=Decimal, 1=Hex ($XXXX), 2=Binary (16 digits) }
    class function FormatValue(AValue: Word; ASigned: Boolean;
                               AFormat: Integer): string;

    property ResponseTimeout : Integer             read FResponseTimeout
                                                   write FResponseTimeout;
    property PacketCount     : Integer             read FPacketCount;
    property ErrorCount      : Integer             read FErrorCount;
    property OnTransaction   : TOnTransactionEvent read FOnTransaction
                                                   write FOnTransaction;
  end;

implementation

{ =============================================================================
  Constructor / Destructor
  ============================================================================= }

constructor TModbusMaster.Create(ASerial: TLazSerial);
begin
  inherited Create;
  FSerial          := ASerial;
  FResponseTimeout := 1000;
  FPacketCount     := 0;
  FErrorCount      := 0;
end;

destructor TModbusMaster.Destroy;
begin
  inherited Destroy;
end;

{ =============================================================================
  Frame building
  ============================================================================= }

function TModbusMaster.AppendCRC(const AData: TByteArray): TByteArray;
var
  CRC : Word;
  Len : Integer;
begin
  Len := Length(AData);
  SetLength(Result, Len + 2);
  Move(AData[0], Result[0], Len);
  CRC          := CRC16(AData, Len);
  Result[Len]  := Lo(CRC);   { low byte first — Modbus little-endian CRC }
  Result[Len+1]:= Hi(CRC);
end;

function TModbusMaster.BuildReadRequest(ASlaveAddr, AFuncCode: Byte;
  AStartAddr, AQuantity: Word): TByteArray;
var
  Raw: TByteArray;
begin
  SetLength(Raw, 6);
  Raw[0] := ASlaveAddr;
  Raw[1] := AFuncCode;
  Raw[2] := Hi(AStartAddr);
  Raw[3] := Lo(AStartAddr);
  Raw[4] := Hi(AQuantity);
  Raw[5] := Lo(AQuantity);
  Result := AppendCRC(Raw);
end;

function TModbusMaster.BuildWriteSingleCoilRequest(ASlaveAddr: Byte;
  ACoilAddr: Word; AValue: Boolean): TByteArray;
var
  Raw: TByteArray;
begin
  SetLength(Raw, 6);
  Raw[0] := ASlaveAddr;
  Raw[1] := FUNC_WRITE_SINGLE_COIL;
  Raw[2] := Hi(ACoilAddr);
  Raw[3] := Lo(ACoilAddr);
  if AValue then begin Raw[4] := $FF; Raw[5] := $00; end
  else           begin Raw[4] := $00; Raw[5] := $00; end;
  Result := AppendCRC(Raw);
end;

function TModbusMaster.BuildWriteSingleRegRequest(ASlaveAddr: Byte;
  ARegAddr, AValue: Word): TByteArray;
var
  Raw: TByteArray;
begin
  SetLength(Raw, 6);
  Raw[0] := ASlaveAddr;
  Raw[1] := FUNC_WRITE_SINGLE_REG;
  Raw[2] := Hi(ARegAddr);
  Raw[3] := Lo(ARegAddr);
  Raw[4] := Hi(AValue);
  Raw[5] := Lo(AValue);
  Result := AppendCRC(Raw);
end;

function TModbusMaster.BuildWriteMultipleRegsRequest(ASlaveAddr: Byte;
  AStartAddr, AQuantity: Word; const AValues: array of Word): TByteArray;
var
  Raw : TByteArray;
  BC  : Integer;
  I   : Integer;
begin
  BC := AQuantity * 2;
  SetLength(Raw, 7 + BC);
  Raw[0] := ASlaveAddr;
  Raw[1] := FUNC_WRITE_MULTIPLE_REGS;
  Raw[2] := Hi(AStartAddr);
  Raw[3] := Lo(AStartAddr);
  Raw[4] := Hi(AQuantity);
  Raw[5] := Lo(AQuantity);
  Raw[6] := Byte(BC);
  for I := 0 to Integer(AQuantity) - 1 do
  begin
    Raw[7 + I*2]     := Hi(AValues[I]);
    Raw[7 + I*2 + 1] := Lo(AValues[I]);
  end;
  Result := AppendCRC(Raw);
end;

{ =============================================================================
  Wire I/O  —  LazSerial string API
  =============================================================================
  LazSerial has no binary Read(buf, n) or FlushBuffer method.
  All data transfer goes through AnsiString (each Char = one raw byte).
  ============================================================================= }

procedure TModbusMaster.SendFrame(const AData: TByteArray);
{ Convert TByteArray → AnsiString and hand it to LazSerial.WriteData. }
var
  S: AnsiString;
  I: Integer;
begin
  SetLength(S, Length(AData));
  for I := 0 to Length(AData) - 1 do
    S[I+1] := AnsiChar(AData[I]);
  FSerial.WriteData(S);
end;

procedure TModbusMaster.DrainRxBuffer;
{ Consume and discard any bytes already in the OS RX buffer.
  LazSerial has no FlushBuffer; we read until nothing comes back.
  Two successive empty reads at 2 ms spacing are enough to clear
  a stale byte or two from a previous transaction. }
var
  Attempts: Integer;
begin
  for Attempts := 0 to 1 do
  begin
    if FSerial.ReadData = '' then Break;
    Sleep(2);
  end;
end;

function TModbusMaster.ExpectedResponseLength(AFuncCode: Byte;
  AQuantity: Word): Integer;
begin
  case AFuncCode of
    FUNC_READ_COILS,
    FUNC_READ_DISCRETE_INPUTS:
      { 3 header + ceil(Q/8) coil bytes + 2 CRC }
      Result := 5 + Integer((AQuantity + 7) div 8);

    FUNC_READ_HOLDING_REGS,
    FUNC_READ_INPUT_REGS:
      { 3 header + Q*2 register bytes + 2 CRC }
      Result := 5 + Integer(AQuantity) * 2;

    FUNC_WRITE_SINGLE_COIL,
    FUNC_WRITE_SINGLE_REG,
    FUNC_WRITE_MULTIPLE_COILS,
    FUNC_WRITE_MULTIPLE_REGS:
      Result := 8;   { echo / confirmation frame }
  else
    Result := 8;
  end;
end;

function TModbusMaster.ReadResponse(AFuncCode: Byte; AQuantity: Word;
  out AResponse: TByteArray): Boolean;
{ Accumulates bytes from LazSerial.ReadData (string API) into an internal
  TByteArray until either the expected frame length is satisfied or the
  response timeout expires.

  Byte-count adjustment:
    Once byte[2] arrives for read FCs, the actual frame length is known
    (3 + ByteCount + 2) and ExpLen is updated accordingly.

  Exception detection:
    If byte[1] has bit 7 set the slave sent a 5-byte exception response;
    ExpLen is shrunk to 5 immediately. }
var
  Buf       : TByteArray;   { accumulator — grows as bytes arrive }
  Got       : Integer;
  ExpLen    : Integer;
  StartTick : QWord;
  Chunk     : AnsiString;
  I         : Integer;
begin
  Result   := False;
  Got      := 0;
  ExpLen   := ExpectedResponseLength(AFuncCode, AQuantity);
  SetLength(AResponse, 0);
  SetLength(Buf, 256);       { pre-allocate; won't reallocate unless frame > 256 }

  StartTick := GetTickCount64;

  while (Got < ExpLen) and
        ((GetTickCount64 - StartTick) < QWord(FResponseTimeout)) do
  begin
    Chunk := FSerial.ReadData;
    if Chunk <> '' then
    begin
      { Append chunk bytes into Buf }
      if Got + Length(Chunk) > Length(Buf) then
        SetLength(Buf, Got + Length(Chunk) + 64);

      for I := 1 to Length(Chunk) do
      begin
        Buf[Got] := Byte(Chunk[I]);
        Inc(Got);
      end;

      { After receiving at least 2 bytes, detect exception response }
      if (Got >= 2) and ((Buf[1] and $80) <> 0) then
        ExpLen := 5

      { After receiving at least 3 bytes for a read FC, use the byte-count
        field to determine the actual frame length }
      else if (Got >= 3) and
              (AFuncCode in [FUNC_READ_COILS, FUNC_READ_DISCRETE_INPUTS,
                             FUNC_READ_HOLDING_REGS, FUNC_READ_INPUT_REGS]) then
        ExpLen := 3 + Integer(Buf[2]) + 2;
    end
    else
      Sleep(2);   { nothing yet — yield for 2 ms before retrying }
  end;

  if Got < 4 then Exit;   { minimum valid Modbus frame = 4 bytes }

  SetLength(AResponse, Got);
  Move(Buf[0], AResponse[0], Got);
  Result := True;
end;

{ =============================================================================
  Core transaction
  ============================================================================= }

function TModbusMaster.Execute(const ARequest: TByteArray; AFuncCode: Byte;
  AQuantity: Word; out ATx: TModbusTransaction): Boolean;
var
  RawResp: TByteArray;
begin
  Result         := False;
  ATx.Request    := nil;
  ATx.Response   := nil;
  ATx.Success    := False;
  ATx.ErrorMsg   := '';
  SetLength(RawResp, 0);

  { Parse request bytes into a TModbusPacket for the bus monitor }
  ATx.Request := ParsePacket(ARequest, 'TX');

  if not FSerial.Active then
  begin
    ATx.ErrorMsg := 'Serial port not open';
    Inc(FErrorCount);
    if Assigned(FOnTransaction) then FOnTransaction(ATx);
    Exit;
  end;

  { Drain any leftover bytes from the previous transaction }
  DrainRxBuffer;

  { Send the request frame }
  SendFrame(ARequest);

  { Collect and accumulate the response }
  if not ReadResponse(AFuncCode, AQuantity, RawResp) then
  begin
    ATx.ErrorMsg := 'Response timeout — no data received';
    Inc(FErrorCount);
    Inc(FPacketCount);
    if Assigned(FOnTransaction) then FOnTransaction(ATx);
    Exit;
  end;

  { Parse response into a TModbusPacket (CRC is validated inside ParsePacket) }
  ATx.Response := ParsePacket(RawResp, 'RX');

  { CRC check }
  if not ATx.Response.ValidCRC then
  begin
    ATx.ErrorMsg := Format('CRC error — expected 0x%04X', [ATx.Response.CRC]);
    Inc(FErrorCount);
    Inc(FPacketCount);
    if Assigned(FOnTransaction) then FOnTransaction(ATx);
    Exit;
  end;

  { Slave address echo check }
  if ATx.Response.Address <> ARequest[0] then
  begin
    ATx.ErrorMsg := Format('Slave address mismatch: sent $%02X, got $%02X',
                           [ARequest[0], ATx.Response.Address]);
    Inc(FErrorCount);
    Inc(FPacketCount);
    if Assigned(FOnTransaction) then FOnTransaction(ATx);
    Exit;
  end;

  { Modbus exception from the slave }
  if ATx.Response.IsException then
  begin
    ATx.ErrorMsg := Format('Slave exception 0x%02X — %s',
                           [ATx.Response.ExceptionCode,
                            GetExceptionName(ATx.Response.ExceptionCode)]);
    Inc(FErrorCount);
    Inc(FPacketCount);
    if Assigned(FOnTransaction) then FOnTransaction(ATx);
    Exit;
  end;

  ATx.Success := True;
  Inc(FPacketCount);
  if Assigned(FOnTransaction) then FOnTransaction(ATx);
  Result := True;
end;

{ =============================================================================
  Public API
  ============================================================================= }

function TModbusMaster.PollRead(ASlaveAddr, AFuncCode: Byte;
  AStartAddr, AQuantity: Word; out ATx: TModbusTransaction): Boolean;
begin
  Result := Execute(
    BuildReadRequest(ASlaveAddr, AFuncCode, AStartAddr, AQuantity),
    AFuncCode, AQuantity, ATx);
end;

function TModbusMaster.WriteSingleCoil(ASlaveAddr: Byte; ACoilAddr: Word;
  AValue: Boolean; out ATx: TModbusTransaction): Boolean;
begin
  Result := Execute(
    BuildWriteSingleCoilRequest(ASlaveAddr, ACoilAddr, AValue),
    FUNC_WRITE_SINGLE_COIL, 1, ATx);
end;

function TModbusMaster.WriteSingleRegister(ASlaveAddr: Byte;
  ARegAddr, AValue: Word; out ATx: TModbusTransaction): Boolean;
begin
  Result := Execute(
    BuildWriteSingleRegRequest(ASlaveAddr, ARegAddr, AValue),
    FUNC_WRITE_SINGLE_REG, 1, ATx);
end;

function TModbusMaster.WriteMultipleRegisters(ASlaveAddr: Byte;
  AStartAddr, AQuantity: Word; const AValues: array of Word;
  out ATx: TModbusTransaction): Boolean;
begin
  Result := Execute(
    BuildWriteMultipleRegsRequest(ASlaveAddr, AStartAddr, AQuantity, AValues),
    FUNC_WRITE_MULTIPLE_REGS, AQuantity, ATx);
end;

{ =============================================================================
  Value decoding helpers
  ============================================================================= }

class function TModbusMaster.GetRegisterWord(const APkt: TModbusPacket;
  ARegIndex: Integer): Word;
var
  Offset: Integer;
begin
  Result := 0;
  if APkt = nil then Exit;
  Offset := 1 + ARegIndex * 2;   { Data[0]=ByteCount; registers start at [1] }
  if (Offset >= 0) and (Offset + 1 < Length(APkt.Data)) then
    Result := BigEndianWord(APkt.Data, Offset);
end;

class function TModbusMaster.GetRegisterFloat_ABCD(const APkt: TModbusPacket;
  ARegIndex: Integer): Single;
{ Big-Endian: high word first  (AB CD → float bytes A B C D) }
var
  RawHi, RawLo : Word;
  Raw32        : LongWord;
begin
  RawHi := GetRegisterWord(APkt, ARegIndex);
  RawLo := GetRegisterWord(APkt, ARegIndex + 1);
  Raw32 := (LongWord(RawHi) shl 16) or LongWord(RawLo);
  Move(Raw32, Result, SizeOf(Result));
end;

class function TModbusMaster.GetRegisterFloat_CDAB(const APkt: TModbusPacket;
  ARegIndex: Integer): Single;
{ Mid-Big: low word first  (CD AB → float bytes A B C D) }
var
  RawHi, RawLo : Word;
  Raw32         : LongWord;
begin
  RawLo := GetRegisterWord(APkt, ARegIndex);
  RawHi := GetRegisterWord(APkt, ARegIndex + 1);
  Raw32 := (LongWord(RawHi) shl 16) or LongWord(RawLo);
  Move(Raw32, Result, SizeOf(Result));
end;

class function TModbusMaster.GetRegisterFloat_BADC(const APkt: TModbusPacket;
  ARegIndex: Integer): Single;
{ Mid-Little: byte-swapped big-endian  (BA DC → float bytes A B C D) }
var
  W0, W1 : Word;
  Raw32  : LongWord;
begin
  W0    := GetRegisterWord(APkt, ARegIndex);
  W1    := GetRegisterWord(APkt, ARegIndex + 1);
  W0    := ((W0 and $FF) shl 8) or (W0 shr 8);
  W1    := ((W1 and $FF) shl 8) or (W1 shr 8);
  Raw32 := (LongWord(W0) shl 16) or LongWord(W1);
  Move(Raw32, Result, SizeOf(Result));
end;

class function TModbusMaster.GetRegisterFloat_DCBA(const APkt: TModbusPacket;
  ARegIndex: Integer): Single;
{ Little-Endian: low word first, bytes reversed  (DC BA → float A B C D) }
var
  W0, W1 : Word;
  Raw32  : LongWord;
begin
  W0    := GetRegisterWord(APkt, ARegIndex);
  W1    := GetRegisterWord(APkt, ARegIndex + 1);
  Raw32 := (LongWord(W1) shl 16) or LongWord(W0);
  Move(Raw32, Result, SizeOf(Result));
end;

class function TModbusMaster.GetCoilBit(const APkt: TModbusPacket;
  ACoilIndex: Integer): Boolean;
var
  ByteIdx : Integer;
  BitIdx  : Integer;
begin
  Result  := False;
  if APkt = nil then Exit;
  ByteIdx := 1 + ACoilIndex div 8;   { Data[0]=ByteCount; bits start at [1] }
  BitIdx  := ACoilIndex mod 8;
  if (ByteIdx >= 0) and (ByteIdx < Length(APkt.Data)) then
    Result := (APkt.Data[ByteIdx] and (Byte(1) shl BitIdx)) <> 0;
end;

class function TModbusMaster.FormatValue(AValue: Word; ASigned: Boolean;
  AFormat: Integer): string;
var
  SVal: SmallInt;
begin
  Result := '';
  case AFormat of
    1:   Result := Format('$%04X', [AValue]);
    2:   Result := BinStr(AValue, 16);
  else
    { Decimal — signed or unsigned }
    if ASigned then
    begin
      SVal   := SmallInt(AValue);   { reinterpret bit pattern }
      Result := IntToStr(SVal);
    end
    else
      Result := IntToStr(AValue);
  end;
end;

end.
