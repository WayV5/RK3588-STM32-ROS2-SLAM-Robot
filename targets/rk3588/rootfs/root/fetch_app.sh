#!/bin/bash
# fetch_app.sh — RK3588 side: fetch app source from PC via HTTP, build, install
#
# Usage (run on RK3588):
#   ./fetch_app.sh 192.168.137.1:8080           fetch + build
#   ./fetch_app.sh 192.168.137.1:8080 --no-colcon  fetch only
#
# Zero dependencies — uses wget (built-in).
#
set -euo pipefail

PC_URL="${1:-}"
NO_COLCON=false

for arg in "$@"; do
	case "$arg" in
		--no-colcon) NO_COLCON=true ;;
	esac
done

if [ -z "$PC_URL" ]; then
	echo "Usage: $0 <PC_IP:PORT> [--no-colcon]"
	echo "  e.g. $0 192.168.137.1:8080"
	exit 1
fi

APP_DIR="/app"
TARBALL="app_src.tar.gz"

# ── Step 1: Download tarball via HTTP ────────────────────────────────
echo "===== Fetching from PC (http://${PC_URL}) ====="

cd /tmp
rm -f "$TARBALL" "${TARBALL}.sha256"

wget -q --show-progress "http://${PC_URL}/$TARBALL" || {
	echo "ERROR: download failed. Check:"
	echo "  1. PC: ./serve_app.sh is running"
	echo "  2. ping $(echo $PC_URL | cut -d: -f1)"
	echo "  3. wget http://${PC_URL}/ (should list files)"
	exit 1
}

SIZE=$(du -sh "$TARBALL" | cut -f1)
echo "  → Downloaded: $TARBALL ($SIZE)"

# Download checksum
wget -q "http://${PC_URL}/${TARBALL}.sha256" 2>/dev/null || true

# ── Step 2: Verify checksum ──────────────────────────────────────────
if [ -f "${TARBALL}.sha256" ]; then
	echo "  → Verifying checksum..."
	if sha256sum -c "${TARBALL}.sha256"; then
		echo "  → Checksum OK"
	else
		echo "ERROR: Checksum mismatch! Corrupted download, aborting."
		exit 1
	fi
else
	echo "  → No checksum file, skipping verification"
fi

# ── Step 3: Stop running services ────────────────────────────────────
echo "===== Stopping ROS2 services ====="
sudo systemctl stop can_gateway 2>/dev/null && echo "  → can_gateway stopped" || true

# ── Step 4: Extract to /app ──────────────────────────────────────────
echo "===== Extracting to $APP_DIR ====="

# Backup existing install
if [ -d "$APP_DIR/install" ]; then
	echo "  → Backing up install/ → install.bak/"
	sudo rm -rf "$APP_DIR/install.bak"
	sudo mv "$APP_DIR/install" "$APP_DIR/install.bak"
fi

sudo rm -rf "$APP_DIR/src"
sudo tar xzf "/tmp/$TARBALL" -C "$APP_DIR/"
sudo chown -R root:root "$APP_DIR/src"

echo "  → src/ extracted:"
find "$APP_DIR/src" -maxdepth 3 -type d | sort
echo "  ..."

# ── Step 5: colcon build ─────────────────────────────────────────────
if ! $NO_COLCON; then
	echo ""
	echo "===== Building (colcon) ====="
	cd "$APP_DIR"
	set +u
	source /opt/ros/humble/setup.bash
	set -u

	if colcon build --symlink-install; then
		echo ""
		echo "===== Build OK ====="
		sudo rm -rf "$APP_DIR/install.bak"
	else
		echo ""
		echo "===== Build FAILED ====="
		echo "  Restoring install.bak..."
		sudo rm -rf "$APP_DIR/install"
		sudo mv "$APP_DIR/install.bak" "$APP_DIR/install"
		exit 1
	fi
else
	echo ""
	echo "===== Skipping build (--no-colcon) ====="
fi

# ── Step 6: Restart services ─────────────────────────────────────────
if sudo systemctl start can_gateway 2>/dev/null; then
	echo "  → can_gateway started"
else
	echo "  → can_gateway not configured (skip)"
fi

# Cleanup
rm -f "/tmp/$TARBALL" "/tmp/${TARBALL}.sha256"

echo ""
echo "===== Done ====="
echo "  source $APP_DIR/install/setup.bash"
echo "  ros2 run robot_can_gateway can_gateway_node"
