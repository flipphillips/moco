#!/bin/bash
# Rebuild DMCLite Unreal Plugin

# Default UE paths
UE_MAC="/Users/Shared/Epic Games/UE_5.7"
UE_LINUX="/opt/EpicGames/UnrealEngine_5.7"

# Determine UE Path
if [[ "$OSTYPE" == "darwin"* ]]; then
    UE_ROOT="${UE_ROOT:-$UE_MAC}"
else
    UE_ROOT="${UE_ROOT:-$UE_LINUX}"
fi

RUN_UAT="$UE_ROOT/Engine/Build/BatchFiles/RunUAT.sh"

if [ ! -f "$RUN_UAT" ]; then
    echo "Error: RunUAT.sh not found at $RUN_UAT"
    echo "Please set UE_ROOT environment variable to your Unreal Engine directory."
    exit 1
fi

PLUGIN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$PLUGIN_DIR/Build"
PROJECT_UPROJECT="$(cd "$PLUGIN_DIR/../../" && pwd)/Coppola_MoCo_Rig.uproject"

echo "Building DMCLite Plugin..."
echo "Plugin: $PLUGIN_DIR/DMCLite.uplugin"
echo "Project: $PROJECT_UPROJECT"
echo "Output: $PACKAGE_DIR"

# Clean build
rm -rf "$PACKAGE_DIR"
rm -rf "$PLUGIN_DIR/Intermediate"
rm -rf "$PLUGIN_DIR/Binaries"

"$RUN_UAT" BuildPlugin \
    -Plugin="$PLUGIN_DIR/DMCLite.uplugin" \
    -Package="$PACKAGE_DIR" \
    -Project="$PROJECT_UPROJECT" \
    -Rocket

if [ $? -eq 0 ]; then
    echo "----------------------------------------"
    echo "BUILD SUCCESSFUL"
    echo "Built plugin is in: $PACKAGE_DIR"
else
    echo "----------------------------------------"
    echo "BUILD FAILED"
    exit 1
fi
