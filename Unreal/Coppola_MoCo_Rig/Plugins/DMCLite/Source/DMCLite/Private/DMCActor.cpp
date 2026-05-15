#include "DMCActor.h"
#include "DMCProtocol.h"

ADMCActor::ADMCActor()
{
    PrimaryActorTick.bCanEverTick = true;
    BaudRate = 115200;
}

void ADMCActor::BeginPlay()
{
    Super::BeginPlay();

    SerialHandler = NewObject<UDMCSerialHandler>(this);
    SerialHandler->OnPacketReceived.AddUObject(this, &ADMCActor::HandlePacketReceived);

    if (!PortName.IsEmpty())
    {
        if (SerialHandler->Connect(PortName, BaudRate))
        {
            UE_LOG(LogTemp, Log, TEXT("DMC: Connected to %s"), *PortName);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("DMC: Failed to connect to %s"), *PortName);
        }
    }
}

void ADMCActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (SerialHandler)
    {
        SerialHandler->Tick();
    }
}

void ADMCActor::SendHandshake()
{
    if (SerialHandler && SerialHandler->IsConnected())
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Sending HI handshake..."));
        SerialHandler->SendCommand(1, (int32)DMCLite::ECommand::HI, {});
    }
}

void ADMCActor::GetMotorStatus(int32 Axis)
{
    if (SerialHandler && SerialHandler->IsConnected())
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Requesting status for Axis %d..."), Axis);
        TArray<uint8> Payload;
        Payload.Add((uint8)Axis);
        SerialHandler->SendCommand(2, (int32)DMCLite::ECommand::MOTOR_STATUS, Payload);
    }
}

void ADMCActor::GetMotorPosition(int32 Axis)
{
    if (SerialHandler && SerialHandler->IsConnected())
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Requesting position for Axis %d..."), Axis);
        TArray<uint8> Payload;
        Payload.Add((uint8)Axis);
        SerialHandler->SendCommand(3, (int32)DMCLite::ECommand::MOTOR_GET_POSITION, Payload);
    }
}

void ADMCActor::ResetMotorPosition(int32 Axis)
{
    if (SerialHandler && SerialHandler->IsConnected())
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Resetting position for Axis %d..."), Axis);
        TArray<uint8> Payload;
        Payload.Add((uint8)Axis);
        SerialHandler->SendCommand(4, (int32)DMCLite::ECommand::MOTOR_RESET_POSITION, Payload);
    }
}

void ADMCActor::HandlePacketReceived(const TArray<uint8>& Packet)
{
    if (Packet.Num() < DMCLite::HEADER_SIZE) return;

    // Read Command ID (offset 6, 2 bytes)
    uint16 CommandID = *(uint16*)&Packet[6];
    uint16 PayloadSize = *(uint16*)&Packet[8];

    UE_LOG(LogTemp, Log, TEXT("DMC: Received packet, CommandID: 0x%04X, Size: %d"), CommandID, Packet.Num());

    if (CommandID == (uint16)DMCLite::ECommand::HI)
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Handshake SUCCESS! Hardware replied to HI."));
    }
    else if (CommandID == (uint16)DMCLite::ECommand::MOTOR_STATUS)
    {
        if (PayloadSize >= 2)
        {
            uint8 Axis = Packet[DMCLite::HEADER_SIZE];
            uint8 Status = Packet[DMCLite::HEADER_SIZE + 1];
            UE_LOG(LogTemp, Log, TEXT("DMC: Axis %d Status: 0x%02X"), Axis, Status);
        }
    }
    else if (CommandID == (uint16)DMCLite::ECommand::MOTOR_GET_POSITION)
    {
        if (PayloadSize >= 5) // 1 byte axis + 4 byte int32 position
        {
            uint8 Axis = Packet[DMCLite::HEADER_SIZE];
            int32 Position = *(int32*)&Packet[DMCLite::HEADER_SIZE + 1];
            UE_LOG(LogTemp, Log, TEXT("DMC: Axis %d Position: %d"), Axis, Position);
        }
    }
}
