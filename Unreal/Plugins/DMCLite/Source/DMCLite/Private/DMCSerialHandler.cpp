#include "DMCSerialHandler.h"
#include "HAL/RunnableThread.h"

UDMCSerialHandler::UDMCSerialHandler()
    : Worker(nullptr)
    , Thread(nullptr)
{
}

void UDMCSerialHandler::BeginDestroy()
{
    Disconnect();
    Super::BeginDestroy();
}

bool UDMCSerialHandler::Connect(const FString& PortName, int32 BaudRate)
{
    Disconnect();

    Worker = new FDMCSerialWorker(PortName, BaudRate);
    Thread = FRunnableThread::Create(Worker, *FString::Printf(TEXT("DMCSerialWorker_%s"), *PortName));

    if (!Thread)
    {
        delete Worker;
        Worker = nullptr;
        return false;
    }

    return true;
}

void UDMCSerialHandler::Disconnect()
{
    if (Thread)
    {
        Thread->Kill(true);
        delete Thread;
        Thread = nullptr;
    }

    if (Worker)
    {
        delete Worker;
        Worker = nullptr;
    }
}

bool UDMCSerialHandler::IsConnected() const
{
    return Worker && Worker->IsConnected();
}

void UDMCSerialHandler::SendCommand(uint32 MessageID, int32 CommandID, const TArray<uint8>& Payload)
{
    if (!Worker || !Worker->IsConnected()) return;

    TArray<uint8> Packet = DMCLite::CreateHeader(MessageID, (DMCLite::ECommand)CommandID, Payload.Num());
    Packet.Append(Payload);
    DMCLite::FinalizePacket(Packet);

    Worker->EnqueuePacket(Packet);
}

void UDMCSerialHandler::Tick()
{
    if (!Worker) return;

    TArray<uint8> Packet;
    while (Worker->DequeuePacket(Packet))
    {
        OnPacketReceived.Broadcast(Packet);
    }
}