#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "Containers/Queue.h"
#include "DMCProtocol.h"

/**
 * Background worker thread for serial communication with DMC-Lite hardware.
 */
class FDMCSerialWorker : public FRunnable
{
public:
    FDMCSerialWorker(const FString& InPortName, int32 InBaudRate);
    virtual ~FDMCSerialWorker();

    // FRunnable interface
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

    /** Queue a packet to be sent to the hardware */
    void EnqueuePacket(const TArray<uint8>& Packet);

    /** Get the next received packet from the hardware */
    bool DequeuePacket(TArray<uint8>& OutPacket);

    bool IsConnected() const { return bIsConnected; }

private:
    FString PortName;
    int32 BaudRate;
    
    FThreadSafeCounter StopTaskCounter;
    bool bIsConnected;

    /** Outgoing packets (Unreal -> Hardware) */
    TQueue<TArray<uint8>, EQueueMode::Spsc> SendQueue;
    
    /** Incoming packets (Hardware -> Unreal) */
    TQueue<TArray<uint8>, EQueueMode::Spsc> ReceiveQueue;

    /** OS-specific file handle/descriptor */
#if PLATFORM_WINDOWS
    void* hSerial;
#else
    int fd;
#endif

    bool OpenPort();
    void ClosePort();
    
    /** Processing logic */
    void ProcessRead();
    void ProcessWrite();

    // Packet reconstruction buffer
    TArray<uint8> ReadBuffer;
};