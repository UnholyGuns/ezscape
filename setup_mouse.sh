#!/bin/bash

set -e

GADGET="/sys/kernel/config/usb_gadget/mouse"
UDC="fe980000.usb"
HID_NODE="/dev/hidg0"

echo "======================================"
echo " Raspberry Pi USB HID Mouse Setup"
echo "======================================"
echo


# ============================================================
# 1. Verify USB Device Controller
# ============================================================

if [ ! -d "/sys/class/udc/$UDC" ]; then
    echo "[ERROR] USB device controller '$UDC' not found."
    echo
    echo "Available UDCs:"
    ls /sys/class/udc
    exit 1
fi

echo "[OK] UDC available: $UDC"


# ============================================================
# 2. Remove stale /dev/hidg0
# ============================================================

# /dev/hidg0 MUST be a character device.
# If a regular file exists here, programs will write HID reports
# into that file instead of sending them to the USB host.
if [ -e "$HID_NODE" ] && [ ! -c "$HID_NODE" ]; then
    echo "[WARNING] $HID_NODE exists but is NOT a character device."
    echo "          Removing stale file..."
    rm -f "$HID_NODE"
fi


# ============================================================
# 3. Remove Existing Gadget
# ============================================================

if [ -d "$GADGET" ]; then

    echo "Removing existing mouse gadget..."

    cd "$GADGET"

    # Disconnect from the USB host before modifying ConfigFS.
    if [ -n "$(cat UDC 2>/dev/null)" ]; then
        echo "" > UDC
        sleep 2
    fi

    # Configuration symlink must be removed first.
    rm -f configs/c.1/hid.usb0 2>/dev/null || true

    rmdir configs/c.1/strings/0x409 2>/dev/null || true
    rmdir configs/c.1 2>/dev/null || true

    rmdir functions/hid.usb0 2>/dev/null || true

    rmdir strings/0x409 2>/dev/null || true

    cd /sys/kernel/config/usb_gadget
    rmdir mouse 2>/dev/null || true
fi


# ============================================================
# 4. Create Gadget
# ============================================================

echo "Creating HID mouse gadget..."

mkdir -p "$GADGET"
cd "$GADGET"

echo 0x1d6b > idVendor
echo 0x0104 > idProduct

echo 0x0200 > bcdUSB
echo 0x0100 > bcdDevice


# ============================================================
# 5. USB Identification Strings
# ============================================================

mkdir -p strings/0x409

echo "0001" > strings/0x409/serialnumber
echo "Raspberry Pi" > strings/0x409/manufacturer
echo "USB HID Mouse Bridge" > strings/0x409/product


# ============================================================
# 6. HID Mouse Function
# ============================================================

mkdir -p functions/hid.usb0

echo 1 > functions/hid.usb0/protocol
echo 1 > functions/hid.usb0/subclass
echo 4 > functions/hid.usb0/report_length

# HID report:
#
# Byte 0 = 5 mouse buttons
# Byte 1 = relative X
# Byte 2 = relative Y
# Byte 3 = relative wheel
#
printf '\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x05\x15\x00\x25\x01\x95\x05\x75\x01\x81\x02\x95\x01\x75\x03\x81\x01\x05\x01\x09\x30\x09\x31\x09\x38\x15\x81\x25\x7f\x75\x08\x95\x03\x81\x06\xc0\xc0' \
    > functions/hid.usb0/report_desc


# ============================================================
# 7. USB Configuration
# ============================================================

mkdir -p configs/c.1/strings/0x409

echo "HID Mouse" > configs/c.1/strings/0x409/configuration
echo 120 > configs/c.1/MaxPower

ln -s functions/hid.usb0 configs/c.1/hid.usb0


# ============================================================
# 8. Bind Gadget to USB Controller
# ============================================================

echo "Binding gadget to $UDC..."

echo "$UDC" > UDC

# Give Linux/udev time to create the HID character device and
# give the host computer time to enumerate the gadget.
sleep 2


# ============================================================
# 9. Determine HID Major / Minor Device Numbers
# ============================================================

HID_DEV_FILE="$GADGET/functions/hid.usb0/dev"

if [ ! -f "$HID_DEV_FILE" ]; then
    echo "[ERROR] HID kernel device information was not created."
    exit 1
fi

HID_DEV=$(cat "$HID_DEV_FILE")

MAJOR=$(echo "$HID_DEV" | cut -d: -f1)
MINOR=$(echo "$HID_DEV" | cut -d: -f2)

echo "[OK] HID kernel device: $MAJOR:$MINOR"


# ============================================================
# 10. Validate /dev/hidg0
# ============================================================

# If something recreated /dev/hidg0 as a regular file while the
# gadget was being configured, remove it.
if [ -e "$HID_NODE" ] && [ ! -c "$HID_NODE" ]; then
    echo "[WARNING] $HID_NODE is a regular file."
    echo "          Removing it..."
    rm -f "$HID_NODE"
fi

# Normally udev creates this character device automatically.
# If it didn't, create the node ourselves using the numbers
# assigned by the HID gadget driver.
if [ ! -c "$HID_NODE" ]; then
    echo "Creating HID character device manually..."
    mknod "$HID_NODE" c "$MAJOR" "$MINOR"
fi

chmod 666 "$HID_NODE"


# ============================================================
# 11. Verify Character Device Numbers
# ============================================================

if [ ! -c "$HID_NODE" ]; then
    echo "[ERROR] $HID_NODE is not a character device."
    exit 1
fi

# stat reports major/minor in hexadecimal.
NODE_MAJOR_HEX=$(stat -c '%t' "$HID_NODE")
NODE_MINOR_HEX=$(stat -c '%T' "$HID_NODE")

NODE_MAJOR=$((16#$NODE_MAJOR_HEX))
NODE_MINOR=$((16#$NODE_MINOR_HEX))

if [ "$NODE_MAJOR" -ne "$MAJOR" ] || [ "$NODE_MINOR" -ne "$MINOR" ]; then

    echo "[WARNING] $HID_NODE points to the wrong device."
    echo "          Existing: $NODE_MAJOR:$NODE_MINOR"
    echo "          Expected: $MAJOR:$MINOR"

    rm -f "$HID_NODE"

    echo "Recreating $HID_NODE..."
    mknod "$HID_NODE" c "$MAJOR" "$MINOR"
    chmod 666 "$HID_NODE"
fi


# ============================================================
# 12. Final Verification
# ============================================================

echo
echo "======================================"
echo " Verification"
echo "======================================"

echo
echo -n "Bound UDC: "
cat "$GADGET/UDC"

echo -n "UDC state: "
cat "/sys/class/udc/$UDC/state"

echo -n "HID report length: "
cat "$GADGET/functions/hid.usb0/report_length"

echo "HID kernel device: $MAJOR:$MINOR"

echo
echo "$HID_NODE:"
ls -l "$HID_NODE"

echo


# ============================================================
# 13. Final Sanity Checks
# ============================================================

if [ "$(cat "$GADGET/UDC")" != "$UDC" ]; then
    echo "[ERROR] Gadget is not bound to expected UDC."
    exit 1
fi

if [ ! -c "$HID_NODE" ]; then
    echo "[ERROR] $HID_NODE is not a character device."
    exit 1
fi

echo "[OK] Gadget bound"
echo "[OK] HID function configured"
echo "[OK] $HID_NODE is a character device"
echo "[OK] Device numbers match $MAJOR:$MINOR"

echo
echo "======================================"
echo " Mouse gadget ready"
echo "======================================"
echo
