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

void ADMCActor::HandlePacketReceived(const TArray<uint8>& Packet)
{
    if (Packet.Num() < DMCLite::HEADER_SIZE) return;

    // Read Command ID (offset 6, 2 bytes)
    uint16 CommandID = *(uint16*)&Packet[6];

    UE_LOG(LogTemp, Log, TEXT("DMC: Received packet, CommandID: 0x%04X, Size: %d"), CommandID, Packet.Num());

    if (CommandID == (uint16)DMCLite::ECommand::HI)
    {
        UE_LOG(LogTemp, Log, TEXT("DMC: Handshake SUCCESS! Hardware replied to HI."));
    }
}
