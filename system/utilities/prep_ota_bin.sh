#!/bin/bash

# prep_ota_bin.sh - Prepare ESP32 binary for OTA update (per-sparknode artifacts)
# Place this script in ../system/utilities/ for shared use across projects.
#
# Usage examples (run from your ESP32 project root):
#   ../system/utilities/prep_ota_bin.sh 5         # generate for sparknode05
#   ../system/utilities/prep_ota_bin.sh 1-8       # generate for sparknode01..sparknode08
#
# The script will automatically use the current directory name as the binary name.

# The workflow for preparing an OTA binary is as follows:
#
# 1. Build your project: `idf.py build`
# 2. Prepare for OTA: `../system/utilities/prep_ota_bin.sh <ID|START-END>`
# 3. Your per-node binaries and checksums are automatically ready in `../system/ota/`

# The script handles all the error checking, directory creation, and provides 
# clear feedback about what it's doing. 

set -e  # Exit on any error

# --- Parse arguments: single ID (N) or range (A-B) ---
if [ $# -ne 1 ]; then
    echo "Usage: $0 <ID|START-END>"
    echo "  ID: 1..99 (e.g., 5)"
    echo "  Range: A-B inclusive, 1..99 (e.g., 1-8)"
    exit 1
fi

ARG="$1"
START_ID=""
END_ID=""

if [[ "$ARG" =~ ^[0-9]+$ ]]; then
    START_ID="$ARG"
    END_ID="$ARG"
elif [[ "$ARG" =~ ^([0-9]+)-([0-9]+)$ ]]; then
    START_ID="${BASH_REMATCH[1]}"
    END_ID="${BASH_REMATCH[2]}"
else
    echo "Error: Invalid argument '$ARG'. Use a single number (N) or a range (A-B)."
    exit 1
fi

# Validate bounds
if ! [[ "$START_ID" =~ ^[0-9]+$ && "$END_ID" =~ ^[0-9]+$ ]]; then
    echo "Error: IDs must be numeric."
    exit 1
fi
if [ "$START_ID" -lt 1 ] || [ "$START_ID" -gt 99 ] || [ "$END_ID" -lt 1 ] || [ "$END_ID" -gt 99 ]; then
    echo "Error: IDs must be between 1 and 99."
    exit 1
fi
if [ "$START_ID" -gt "$END_ID" ]; then
    echo "Error: START-ID ($START_ID) must be <= END-ID ($END_ID)."
    exit 1
fi

# Auto-detect binary name from current directory
BINARY_NAME=$(basename "$(pwd)")
BUILD_DIR="build"
SOURCE_BINARY="${BUILD_DIR}/${BINARY_NAME}.bin"

# Use current working directory as base for relative paths
CURRENT_DIR=$(pwd)
DEST_DIR="${CURRENT_DIR}/../system/ota"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
TIMESTAMP_FILE="${DEST_DIR}/${BINARY_NAME}.${TIMESTAMP}"

echo "=== ESP32 OTA Binary Preparation ==="
echo "Auto-detected binary name: ${BINARY_NAME}"
echo "Source: ${SOURCE_BINARY}"
echo "Destination directory: ${DEST_DIR}"
echo "Target IDs: ${START_ID}-${END_ID}"

# Check if we're in the right directory (should have build subdirectory)
if [ ! -d "${BUILD_DIR}" ]; then
    echo "Error: ${BUILD_DIR} directory not found"
    echo "Please run this script from the top directory of your ESP32 project"
    exit 1
fi

# Check if source binary exists
if [ ! -f "${SOURCE_BINARY}" ]; then
    echo "Error: Binary file ${SOURCE_BINARY} not found"
    echo "Please ensure you have built the project with 'idf.py build'"
    echo "Available .bin files in ${BUILD_DIR}:"
    ls -la "${BUILD_DIR}"/*.bin 2>/dev/null || echo "  No .bin files found"
    exit 1
fi

# Create destination directory if it doesn't exist
if [ ! -d "${DEST_DIR}" ]; then
    echo "Creating destination directory: ${DEST_DIR}"
    mkdir -p "${DEST_DIR}"
fi

echo "Preparing per-sparknode artifacts..."

# Compute checksum once from the source binary
CHECKSUM=$(md5sum "${SOURCE_BINARY}" | awk '{print $1}')

# Generate per-node files
for (( i=${START_ID}; i<=${END_ID}; i++ )); do
    printf -v HOSTNAME "sparknode%02d" "$i"
    DEST_BIN="${DEST_DIR}/${HOSTNAME}.bin"
    DEST_MD5="${DEST_DIR}/${HOSTNAME}.md5"

    echo "- ${HOSTNAME}: copying binary -> $(basename "${DEST_BIN}")"
    cp "${SOURCE_BINARY}" "${DEST_BIN}"

    if [ ! -f "${DEST_BIN}" ]; then
        echo "Error: Failed to copy binary for ${HOSTNAME}"
        exit 1
    fi

    echo "  writing checksum -> $(basename "${DEST_MD5}")"
    printf "%s" "${CHECKSUM}" > "${DEST_MD5}"

    if [ ! -f "${DEST_MD5}" ]; then
        echo "Error: Failed to write checksum for ${HOSTNAME}"
        exit 1
    fi
done

# Create timestamp file with original binary name in destination directory
echo "Creating timestamp file..."
touch "${TIMESTAMP_FILE}"

"${SHELL}" -lc 'true' >/dev/null 2>&1 || true

# Verify timestamp file was created
if [ ! -f "${TIMESTAMP_FILE}" ]; then
    echo "Error: Failed to create timestamp file"
    exit 1
fi

# Display results
echo ""
echo "=== OTA Binary Preparation Complete ==="
echo "Timestamp file: ${TIMESTAMP_FILE}"
echo ""
echo "Files ready for OTA deployment in ${DEST_DIR}:"
TOTAL=0
for (( i=${START_ID}; i<=${END_ID}; i++ )); do
    printf -v HOSTNAME "sparknode%02d" "$i"
    BIN_PATH="${DEST_DIR}/${HOSTNAME}.bin"
    MD5_PATH="${DEST_DIR}/${HOSTNAME}.md5"
    if [ -f "${BIN_PATH}" ]; then
        SIZE=$(stat -c%s "${BIN_PATH}")
        echo "  - ${HOSTNAME}.bin (${SIZE} bytes)"
        echo "  - ${HOSTNAME}.md5 (${#CHECKSUM} chars)"
        TOTAL=$((TOTAL+1))
    fi
done
echo "Prepared ${TOTAL} node(s)."
