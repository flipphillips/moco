#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DMCSerialWorker.h"
#include "DMCSerialHandler.generated.h"

/**
 * High-level interface for the DMC-Lite Serial connection in Unreal.
 * Acts as the owner of the background worker thread.
 */
UCLASS(BlueprintType)
class DMCLITE_API UDMCSerialHandler : public UObject
{
    GENERATED_BODY()

public:
    UDMCSerialHandler();
    virtual void BeginDestroy() override;

    /** Open the connection to the DMC-Lite hardware */
    UFUNCTION(BlueprintCallable, Category = "DMCLite")
    bool Connect(const FString& PortName, int32 BaudRate = 115200);

    /** Get a list of available serial ports on the system */
    UFUNCTION(BlueprintPure, Category = "DMCLite")
    static TArray<FString> GetAvailableSerialPorts();

    /** Close the connection */
    UFUNCTION(BlueprintCallable, Category = "DMCLite")
    void Disconnect();

    /** Send a command to the rig */
    UFUNCTION(BlueprintCallable, Category = "DMCLite")
    void SendCommand(int32 MessageID, int32 CommandID, const TArray<uint8>& Payload);

    /** Check if we are currently connected to hardware */
    UFUNCTION(BlueprintPure, Category = "DMCLite")
    bool IsConnected() const;

    /** Call this in your Actor's Tick to process incoming messages */
    void Tick();

    /** Event triggered when a position report is received */
    DECLARE_MULTICAST_DELEGATE_OneParam(FOnPacketReceived, const TArray<uint8>&);
    FOnPacketReceived OnPacketReceived;

private:
    FDMCSerialWorker* Worker;
    FRunnableThread* Thread;
};