#!/bin/bash
# WiFi connect — manual tool: write config + restart wpa_supplicant
# Usage: ./wifi-connect.sh <SSID> <PSK>
#        ./wifi-connect.sh scan

IFACE="wlan0"
CONF="/etc/wpa_supplicant/wpa_supplicant.conf"

if [ "$1" = "scan" ]; then
	iwlist $IFACE scan 2>/dev/null | grep -E "ESSID|Signal|Frequency"
	exit 0
fi

[ $# -eq 2 ] || { echo "Usage: $0 <SSID> <PSK>"; exit 1; }

tee "$CONF" > /dev/null << EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
	ssid="${1}"
	psk="${2}"
}
EOF

pkill -f "wpa_supplicant.*$IFACE" 2>/dev/null || true
wpa_supplicant -B -i "$IFACE" -c "$CONF"
echo "WiFi: $1 — connecting..."
