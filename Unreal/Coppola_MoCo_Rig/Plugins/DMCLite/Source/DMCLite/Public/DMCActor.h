#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DMCSerialHandler.h"
#include "DMCActor.generated.h"

UCLASS()
class DMCLITE_API ADMCActor : public AActor
{
    GENERATED_BODY()
    
public:    
    ADMCActor();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

public:
    /** Port to connect to (e.g. COM3 or /dev/tty.usb...) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DMCLite")
    FString PortName;

    /** Baud rate for the connection */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DMCLite")
    int32 BaudRate;

    /** Handshake: Send HI command */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "DMCLite")
    void SendHandshake();

    /** Get status for a specific axis (1-indexed) */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "DMCLite")
    void GetMotorStatus(int32 Axis = 1);

    /** Get current position for a specific axis (1-indexed) */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "DMCLite")
    void GetMotorPosition(int32 Axis = 1);

    /** Reset position to zero for a specific axis (1-indexed) */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "DMCLite")
    void ResetMotorPosition(int32 Axis = 1);

    /** The serial handler instance */
    UPROPERTY(BlueprintReadOnly, Category = "DMCLite")
    UDMCSerialHandler* SerialHandler;

private:
    void HandlePacketReceived(const TArray<uint8>& Packet);
};
