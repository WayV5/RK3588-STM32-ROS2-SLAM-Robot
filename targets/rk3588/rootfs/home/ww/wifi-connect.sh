#!/bin/bash
# WiFi connect script for RK3588
# Usage: ./wifi-connect.sh <SSID> <PSK>
#        ./wifi-connect.sh scan          (scan nearby APs)
set -e

IFACE="wlan0"
WPA_CONF="/etc/wpa_supplicant/wpa_supplicant-$IFACE.conf"

usage() {
	echo "Usage: $0 <SSID> <PSK>"
	echo "       $0 scan"
	echo ""
	echo "Examples:"
	echo "  $0 daemon 12345678"
	echo "  $0 \"my wifi\" \"my password\""
	exit 1
}

# ---- scan ----
#sudo iwlist wlan0 scan | grep -E "ESSID|Frequency|Signal"
if [ "$1" = "scan" ]; then
	echo "Scanning (wlan0, ~5s) ..."
	sudo iwlist $IFACE scan 2>/dev/null | \
		grep -E "ESSID|Signal|Frequency" | \
		sed 's/^[[:space:]]*//' | \
		grep --color=auto -E "ESSID|^$"
	exit 0
fi

[ $# -eq 2 ] || usage

SSID="$1"
PSK="$2"

echo "WiFi: $SSID"

echo "[1/3] Write wpa_supplicant config"
sudo tee "$WPA_CONF" > /dev/null << EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
	ssid="${SSID}"
	psk="${PSK}"
}
EOF

echo "[2/3] Connect to $SSID ..."
sudo systemctl restart wpa_supplicant@$IFACE
sudo systemctl restart systemd-networkd

echo "[3/3] Wait for IP ..."
for i in $(seq 1 15); do
	IP=$(ip -4 addr show $IFACE 2>/dev/null | grep -oP 'inet \K[\d.]+')
	if [ -n "$IP" ]; then
		echo "Done!  IP: $IP"
		echo ""
		echo "  ssh ww@$IP"
		exit 0
	fi
	sleep 1
	echo -n "."
done

echo ""
echo "Timeout. Check logs:"
echo "  sudo journalctl -u wpa_supplicant@$IFACE --no-pager | tail -15"
exit 1
