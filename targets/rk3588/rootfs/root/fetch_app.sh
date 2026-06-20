#!/bin/bash
# fetch_app.sh — RK3588 side: fetch src/ from PC, extract (src only)
#
# Only replaces source files under /app/src/. Does NOT touch build/, install/,
# log/, or any build artifacts. colcon build must be run separately.
#
# Usage (run on RK3588):
#   ./fetch_app.sh 192.168.137.1:8080
#
# Zero dependencies — uses wget (built-in).
#
set -euo pipefail

PC_URL="${1:-}"
if [ -z "$PC_URL" ]; then
	echo "Usage: $0 <PC_IP:PORT>"
	echo "  e.g. $0 192.168.137.1:8080"
	exit 1
fi

APP_DIR="/app"
TARBALL="app_src.tar.gz"

# ── Download tarball ─────────────────────────────────────────────────
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

wget -q "http://${PC_URL}/${TARBALL}.sha256" 2>/dev/null || true

# ── Verify checksum ──────────────────────────────────────────────────
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

# ── Extract src/ only ────────────────────────────────────────────────
echo "===== Extracting src/ to $APP_DIR ====="
sudo tar xzf "/tmp/$TARBALL" -C "$APP_DIR/"

echo "  → src/ updated"

# Cleanup
rm -f "/tmp/$TARBALL" "/tmp/${TARBALL}.sha256"

echo "===== Done ====="
echo "  cd $APP_DIR && colcon build"
