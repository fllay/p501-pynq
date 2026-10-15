#!/bin/sh
# Configure a CDC-ECM (USB Ethernet) gadget on the dwc3 controller (forced to
# dr_mode=peripheral, see system-user.dtsi). Presents as a plain USB-Ethernet
# adapter on the host PC so Jupyter (and ssh) are reachable over the USB3
# Type-A port, independent of the board's real Ethernet/PL state.

GADGET=/sys/kernel/config/usb_gadget/g_ecm
ECM_DEV_ADDR="02:1a:2b:3c:4d:11"   # device side of the virtual Ethernet link
ECM_HOST_ADDR="02:1a:2b:3c:4d:12"  # host side, as seen by the PC
ECM_DEV_IP="192.168.2.1/24"        # matches the well-known Pluto/p501_iio convention

log() {
	echo "usb-gadget-ecm: $*" | systemd-cat -t usb-gadget-ecm 2>/dev/null || echo "usb-gadget-ecm: $*"
}

if [ -d "$GADGET" ]; then
	log "already configured, exiting"
	exit 0
fi

mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config

mkdir -p "$GADGET" || { log "failed to create $GADGET"; exit 1; }
cd "$GADGET" || exit 1

echo 0x1209 > idVendor      # pid.codes shared test/hobbyist VID
echo 0x0002 > idProduct     # distinct from the earlier composite IIO+ECM gadget
                            # (same idVendor/idProduct/serial/MAC as that gadget
                            # made macOS reuse cached device identity and never
                            # bind a fresh network driver for this one -- see
                            # boards/P501/README.md)
echo 0x0320 > bcdUSB        # advertise USB 3.2 Gen1 (SuperSpeed) capability
echo 0x0100 > bcdDevice

mkdir -p strings/0x409
echo "P501-0002"          > strings/0x409/serialnumber
echo "Puzhi"               > strings/0x409/manufacturer
echo "P501 PYNQ (ECM gadget)" > strings/0x409/product

mkdir -p configs/c.1/strings/0x409
echo "ECM"  > configs/c.1/strings/0x409/configuration
echo 250     > configs/c.1/MaxPower

mkdir -p functions/ecm.usb0
echo "$ECM_DEV_ADDR"  > functions/ecm.usb0/dev_addr
echo "$ECM_HOST_ADDR" > functions/ecm.usb0/host_addr
ln -s functions/ecm.usb0 configs/c.1/

UDC=$(ls /sys/class/udc 2>/dev/null | head -n1)
if [ -z "$UDC" ]; then
	log "no UDC found -- is dr_mode really peripheral on usb@fe200000?"
	exit 1
fi

echo "$UDC" > UDC || { log "failed to bind gadget to UDC $UDC"; exit 1; }

# usb0 only appears once the gadget is bound to a UDC; wait briefly for it.
for i in $(seq 1 20); do
	[ -e /sys/class/net/usb0 ] && break
	sleep 0.1
done
if [ -e /sys/class/net/usb0 ]; then
	ip addr add "$ECM_DEV_IP" dev usb0 2>/dev/null
	ip link set usb0 up
	log "usb0 up at $ECM_DEV_IP"
else
	log "usb0 never appeared after UDC bind"
fi
