#!/bin/bash
# Installs the CDC-ECM USB gadget script + systemd service directly into the
# target rootfs (plain file copy, no qemu/chroot needed) and enables it the
# same way "systemctl enable" would (a unit symlink under
# multi-user.target.wants/), so it starts automatically on every boot.

set -x
set -e

target=$1
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo install -m 0755 "$script_dir/usb-gadget-ecm.sh" "$target/usr/bin/usb-gadget-ecm.sh"
sudo install -m 0644 "$script_dir/usb-gadget-ecm.service" "$target/lib/systemd/system/usb-gadget-ecm.service"
sudo mkdir -p "$target/etc/systemd/system/multi-user.target.wants"
sudo ln -sf /lib/systemd/system/usb-gadget-ecm.service \
	"$target/etc/systemd/system/multi-user.target.wants/usb-gadget-ecm.service"
