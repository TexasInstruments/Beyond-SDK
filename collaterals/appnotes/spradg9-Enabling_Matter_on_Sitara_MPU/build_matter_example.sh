#!/bin/bash
set -e

# =============================================================================
# Matter aarch64 Cross-Compilation Build Script (Unified)
#
# This script handles all necessary fixes and configurations:
# - Bluezoo dependency fix for Python 3.10
# - TI SDK toolchain wrapper creation
# - Complete Matter build process
# =============================================================================

if [[ $# -ne 1 ]]; then
  echo "Error: Please enter exactly one example-name as argument"
  echo "Usage: $0 <your_argument>"
  exit 1
fi

EXAMPLE_NAME="$1"

# =============================================================================
# CONFIGURATION - MODIFY THESE VARIABLES FOR YOUR SETUP
# =============================================================================

# SDK Path
SDK_PATH="/home/<user>/ti-processor-sdk-linux-am62lxx-evm-11.00.15.05"

# Path to your aarch64 sysroot
SYSROOT_AARCH64="$SDK_PATH/filesystem/am62lxx-evm/temp"     # TI SDK sysroot

# Toolchain binary prefix (TI SDK uses aarch64-oe-linux)
TOOLCHAIN_TARGET="aarch64-oe-linux"       # TI SDK compatible target

# Path to your aarch64 cross-compilation toolchain (using TI SDK native toolchain)
TOOLCHAIN_PREFIX="$SDK_PATH/linux-devkit/sysroots/x86_64-arago-linux/usr/bin/$TOOLCHAIN_TARGET"  # TI SDK toolchain

# Path to connectedhomeip repository (relative to script location)
REPO_PATH="."                               # CHANGE THIS if different

echo "=== Matter aarch64 Cross-Compilation Build Script (Unified) ==="
echo "Toolchain: $TOOLCHAIN_TARGET"
echo "Sysroot: $SYSROOT_AARCH64"
echo "Example: $EXAMPLE_NAME"
echo ""

echo "============================================================================="
echo "STEP 1: FIX BLUEZOO DEPENDENCY ISSUE"
echo "============================================================================="

REQUIREMENTS_FILE="$REPO_PATH/scripts/tests/requirements.txt"
if [ -f "$REQUIREMENTS_FILE" ]; then
    # Check if bluezoo is already commented out
    if grep -q "^bluezoo" "$REQUIREMENTS_FILE"; then
        echo "Commenting out bluezoo dependency (requires Python 3.11+)..."
        sed -i 's/^bluezoo/#bluezoo/' "$REQUIREMENTS_FILE"
        echo "✓ Bluezoo dependency commented out"
    else
        echo "✓ Bluezoo dependency already fixed"
    fi
else
    echo "Warning: Requirements file not found at $REQUIREMENTS_FILE"
fi

echo "============================================================================="
echo "STEP 2: VERIFY PATHS AND TOOLCHAIN"
echo "============================================================================="

# Construct toolchain binary paths
export CC_AARCH64="$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-gcc"
export CXX_AARCH64="$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-g++"
export AR_AARCH64="$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-ar"
export STRIP_AARCH64="$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-strip"
export LD_AARCH64="$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-ld"

# Add toolchain to PATH for any remaining usage
export PATH="$TOOLCHAIN_PREFIX:$PATH"

if [ ! -f "$CC_AARCH64" ]; then
    echo "Error: Compiler not found at $CC_AARCH64"
    echo "Please check your TOOLCHAIN_PREFIX and TOOLCHAIN_TARGET variables"
    exit 1
fi

if [ ! -d "$SYSROOT_AARCH64" ]; then
    echo "Error: Sysroot not found at $SYSROOT_AARCH64"
    echo "Please check your SYSROOT_AARCH64 path"
    exit 1
fi

if [ ! -d "$REPO_PATH" ]; then
    echo "Error: Repository not found at $REPO_PATH"
    echo "Please check your REPO_PATH variable"
    exit 1
fi

echo "✓ Toolchain: $($CC_AARCH64 --version | head -1)"
echo "✓ Sysroot: $SYSROOT_AARCH64"
echo "✓ Repository: $REPO_PATH"

echo "============================================================================="
echo "STEP 3: CREATE TOOLCHAIN WRAPPER"
echo "============================================================================="

cd "$REPO_PATH"
#
# # Create toolchain wrapper directory
WRAPPER_DIR="$PWD/toolchain-wrapper-bin"
mkdir -p "$WRAPPER_DIR"

# Create symbolic links with the names GN expects
echo "Creating symbolic links for GN compatibility..."
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-gcc" "$WRAPPER_DIR/aarch64-linux-gnu-gcc"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-g++" "$WRAPPER_DIR/aarch64-linux-gnu-g++"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-ar" "$WRAPPER_DIR/aarch64-linux-gnu-ar"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-strip" "$WRAPPER_DIR/aarch64-linux-gnu-strip"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-ld" "$WRAPPER_DIR/aarch64-linux-gnu-ld"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-objdump" "$WRAPPER_DIR/aarch64-linux-gnu-objdump"
ln -sf "$TOOLCHAIN_PREFIX/${TOOLCHAIN_TARGET}-nm" "$WRAPPER_DIR/aarch64-linux-gnu-nm"

echo "✓ Created toolchain wrapper directory: $WRAPPER_DIR"
echo "Contents:"
ls -la "$WRAPPER_DIR/"

# Add wrapper to PATH
export PATH="$PWD/toolchain-wrapper-bin:$PATH"

echo "============================================================================="
echo "STEP 4: SETUP BUILD ENVIRONMENT"
echo "============================================================================="

source scripts/activate.sh

echo "Testing cross-compilation..."
echo 'int main(){return 0;}' > test.c
$CC_AARCH64 --sysroot="$SYSROOT_AARCH64" -o test test.c
file test
rm test test.c
echo "✓ Cross-compilation test passed"

echo "============================================================================="
echo "STEP 5: CONFIGURE AND BUILD"
echo "============================================================================="

#Check if the example exists
if [ ! -d "examples/${EXAMPLE_NAME}" ]; then
    echo -e "No such '$EXAMPLE_NAME' exists in examples!! \nExiting !!"
    exit 1
#Check if example does not need specific platform like linux to build
elif [[ $(find ./examples/ -maxdepth 2 -type f -name args.gni | grep -c "$EXAMPLE_NAME") -gt 0 ]]; then
    ROOT_PATH="examples/$EXAMPLE_NAME"
#Check if example needs specific platform to build and linux platform is available
elif [[ $(find ./examples/ -type f -name "args.gni" -path "*/linux/*" | grep -c "$EXAMPLE_NAME") -gt 0 ]]; then
    ROOT_PATH="examples/$EXAMPLE_NAME/linux"
#Check if example needs specific platform to build but linux platform is NOT available
else
    echo -e "'$EXAMPLE_NAME' is not supported on Linux!! \nExiting !!"
    exit 1
fi

gn gen "out/${EXAMPLE_NAME}-arm64" --root="$ROOT_PATH" --args="
  target_cpu=\"arm64\"
  target_os=\"linux\"
  sysroot=\"$SYSROOT_AARCH64\"
  is_clang=false
  treat_warnings_as_errors=false
  target_cflags = [
    \"-D_GNU_SOURCE\",
    \"-D__USE_GNU\",
    \"-pthread\",
    \"-DCHIP_DEVICE_CONFIG_WIFI_STATION_IF_NAME=\\\"wlan0\\\"\",
    \"-DCHIP_DEVICE_CONFIG_LINUX_DHCPC_CMD=\\\"udhcpc -b -i %s \\\"\",
  ]
  target_ldflags=[\"-pthread\"]
"

echo "Building $EXAMPLE_NAME..."
ninja -C "out/${EXAMPLE_NAME}-arm64"

echo "============================================================================="
echo "STEP 6: VERIFY BUILD RESULTS"
echo "============================================================================="
EXECUTABLE_NAME=$(awk -F'"' '/executable\("/ {print $2}' $ROOT_PATH/BUILD.gn)
echo "Expected executable name: $EXECUTABLE_NAME"
BINARY_PATH="out/${EXAMPLE_NAME}-arm64/${EXECUTABLE_NAME}"
if [ -f "$BINARY_PATH" ]; then
    file "$BINARY_PATH"
    echo "✓ Build complete! Binary located at: $BINARY_PATH"
else
    echo "Error: Build failed, binary not found at $BINARY_PATH"
    exit 1
fi

echo "============================================================================="
echo "BUILD SUMMARY"
echo "============================================================================="
echo "Target: aarch64 (ARM64)"
echo "Toolchain: $TOOLCHAIN_TARGET"
echo "Example: $EXAMPLE_NAME"
echo "Output: out/${EXAMPLE_NAME}-arm64/${EXECUTABLE_NAME}"
echo ""
echo "All fixes applied:"
echo "✓ Bluezoo dependency commented out for Python 3.10 compatibility"
echo "✓ TI SDK native toolchain configured for compatibility"
echo "✓ Toolchain wrapper created for GN naming conventions"
echo "✓ Matter build completed successfully"
echo ""
echo "To modify configuration, edit the variables at the top of this script:"
echo "- TOOLCHAIN_PREFIX: $TOOLCHAIN_PREFIX"
echo "- SYSROOT_AARCH64: $SYSROOT_AARCH64"
echo "- TOOLCHAIN_TARGET: $TOOLCHAIN_TARGET"
echo "- EXAMPLE_NAME: $EXAMPLE_NAME"
echo ""
echo "To build a different example, run the script with other example's name as argument."
