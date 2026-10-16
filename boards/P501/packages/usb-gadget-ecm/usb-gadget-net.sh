#!/bin/sh
# Automatic IP configuration for the USB gadget interface usb0 (modelled on the
# Raspberry Pi OS rpi-usb-gadget service), using only systemd-networkd:
#
#   on every link-up:  1. try DHCP as a CLIENT for $DHCP_WAIT seconds
#                         (host sharing its connection: NetworkManager
#                          ipv4.method=shared, Windows ICS) -> stay a client
#                      2. no lease -> SHARED mode: static 192.168.2.1/24 plus a
#                         DHCP server, so the host auto-configures itself
#   link down:         reset, decide again at the next link-up
#   host change:       the carrier does not drop when the host merely changes its
#                      sharing mode, so peers are probed every 5 s (ping, or a still-
#                      resolved ARP entry for hosts that drop ICMP); a peer that was
#                      alive and then disappears (or a lost lease) restarts the sequence
#
# pynq.local resolves through systemd-resolved's mDNS responder in both modes.
IF=usb0
DIR=/usr/share/usb-gadget
RUN=/run/systemd/network/10-usb0.network
DHCP_WAIT=${DHCP_WAIT:-10}

log() { echo "usb-gadget-net: $*" | systemd-cat -t usb-gadget-net 2>/dev/null || echo "usb-gadget-net: $*"; }
carrier() { [ "$(cat /sys/class/net/$IF/carrier 2>/dev/null)" = 1 ]; }
has_lease() { ip -4 -o addr show dev $IF 2>/dev/null | grep -q dynamic; }
# Addresses worth probing: our DHCP leases (shared mode) / the DHCP gateway
# (client mode) plus any IPv4 neighbour seen on usb0.
peer_list() {
	{
		networkctl status $IF --no-pager 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+ \(to ' | cut -d' ' -f1
		ip -4 route show default dev $IF 2>/dev/null | awk '{print $3}'
		ip -4 neigh show dev $IF 2>/dev/null | awk '$1 != "" {print $1}'
	} | sort -u
}
# alive | dead | none: are those peers still answering? An unanswered ping alone
# is not proof (hosts may drop ICMP); a FAILED/INCOMPLETE ARP entry is.
peer_state() {
	peers=$(peer_list)
	[ -z "$peers" ] && { echo none; return; }
	for p in $peers; do
		if ping -c1 -W1 "$p" >/dev/null 2>&1; then echo alive; return; fi
		st=$(ip -4 neigh show "$p" dev $IF | awk '{print $NF}')
		case $st in FAILED|INCOMPLETE|"") ;; *) echo alive; return ;; esac
	done
	echo dead
}
apply() {
	mkdir -p /run/systemd/network
	install -m 0644 "$DIR/usb0-$1.network" "$RUN"
	networkctl reload
	networkctl reconfigure $IF
}

# wait for the interface (created by usb-gadget-ecm.sh once the gadget is bound)
while [ ! -e /sys/class/net/$IF ]; do sleep 1; done
ip link set $IF up
apply client
mode=none
t=0
seen=0
miss=0
tick=0
rescan() {
	log "$1: restarting DHCP-client attempt"
	mode=none; seen=0; miss=0
	apply client
}
while :; do
	if carrier; then
		case $mode in
		none)
			mode=client; t=0; seen=0; miss=0
			log "link up: trying DHCP client for ${DHCP_WAIT}s"
			apply client
			;;
		client)
			if has_lease; then
				mode=client-ok
				log "DHCP lease from host: $(ip -4 -o addr show dev $IF | awk '{print $4}' | head -1) (client mode)"
			else
				t=$((t + 1))
				[ $((t % 2)) -eq 0 ] && networkctl renew $IF >/dev/null 2>&1
				if [ "$t" -ge "$DHCP_WAIT" ]; then
					mode=shared
					log "no DHCP server answered: shared mode, 192.168.2.1/24 + DHCP server"
					apply shared
				fi
			fi
			;;
		client-ok|shared)
			if [ "$mode" = client-ok ] && ! has_lease; then
				rescan "DHCP lease lost"
			else
				tick=$((tick + 1))
				if [ $((tick % 5)) -eq 0 ]; then
					case $(peer_state) in
					alive) seen=1; miss=0 ;;
					dead)
						if [ "$seen" = 1 ]; then
							miss=$((miss + 1))
							[ "$miss" -ge 2 ] && rescan "host no longer answering ($mode)"
						fi
						;;
					esac
				fi
			fi
			;;
		esac
	elif [ "$mode" != none ]; then
		log "link down"
		mode=none
		apply client
	fi
	sleep 1
done
