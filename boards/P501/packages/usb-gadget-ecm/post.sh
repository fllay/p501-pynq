#!/bin/bash
# Installs the CDC-ECM USB gadget script + systemd service directly into the
# target rootfs (plain file copy, no qemu/chroot needed) and enables it the
# same way "systemctl enable" would (a unit symlink under
# multi-user.target.wants/), so it starts automatically on every boot.
#
# Also installs the automatic usb0 IP configuration (usb-gadget-net): DHCP
# client when the host shares its connection, otherwise 192.168.2.1/24 plus a
# DHCP server, and an mDNS responder (systemd-resolved) so `pynq.local` works.
# This needs systemd-networkd, which is disabled in the stock image; it only
# manages usb0 (eth0 stays under ifupdown) and its wait-online unit is masked
# so boot is never delayed.

set -x
set -e

target=$1
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
wants="$target/etc/systemd/system/multi-user.target.wants"

sudo install -m 0755 "$script_dir/usb-gadget-ecm.sh" "$target/usr/bin/usb-gadget-ecm.sh"
sudo install -m 0644 "$script_dir/usb-gadget-ecm.service" "$target/lib/systemd/system/usb-gadget-ecm.service"
sudo mkdir -p "$wants"
sudo ln -sf /lib/systemd/system/usb-gadget-ecm.service "$wants/usb-gadget-ecm.service"

# --- automatic usb0 addressing + mDNS ---
sudo install -m 0755 "$script_dir/usb-gadget-net.sh" "$target/usr/bin/usb-gadget-net.sh"
sudo install -m 0644 "$script_dir/usb-gadget-net.service" "$target/lib/systemd/system/usb-gadget-net.service"
sudo mkdir -p "$target/usr/share/usb-gadget" "$target/etc/systemd/resolved.conf.d"
sudo install -m 0644 "$script_dir/usb0-client.network" "$target/usr/share/usb-gadget/usb0-client.network"
sudo install -m 0644 "$script_dir/usb0-shared.network" "$target/usr/share/usb-gadget/usb0-shared.network"
sudo install -m 0644 "$script_dir/10-mdns.conf" "$target/etc/systemd/resolved.conf.d/10-mdns.conf"
sudo ln -sf /lib/systemd/system/usb-gadget-net.service "$wants/usb-gadget-net.service"

# what `systemctl enable systemd-networkd` does (no chroot available here)
sudo mkdir -p "$target/etc/systemd/system/sockets.target.wants" "$target/etc/systemd/system/sysinit.target.wants"
sudo ln -sf /lib/systemd/system/systemd-networkd.service "$target/etc/systemd/system/dbus-org.freedesktop.network1.service"
sudo ln -sf /lib/systemd/system/systemd-networkd.service "$wants/systemd-networkd.service"
sudo ln -sf /lib/systemd/system/systemd-networkd.socket "$target/etc/systemd/system/sockets.target.wants/systemd-networkd.socket"
sudo ln -sf /lib/systemd/system/systemd-network-generator.service "$target/etc/systemd/system/sysinit.target.wants/systemd-network-generator.service"
sudo ln -sf /dev/null "$target/etc/systemd/system/systemd-networkd-wait-online.service"
