#!/bin/bash
# app/serve_app.sh — PC side: pack app source, serve via HTTP for RK3588 to fetch
#
# Usage:
#   ./serve_app.sh              pack + serve
#   ./serve_app.sh --no-pack    serve only (already packed)
#
# Zero dependencies — uses Python built-in http.server.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVE_DIR="/tmp/http_app_serve"
TARBALL="app_src.tar.gz"
NO_PACK=false

for arg in "$@"; do
	case "$arg" in
		--no-pack) NO_PACK=true ;;
	esac
done

# ── Step 1: Pack app source ──────────────────────────────────────────
if ! $NO_PACK; then
	echo "===== Packing app source ====="

	rm -rf "$SERVE_DIR"
	mkdir -p "$SERVE_DIR"

	cd "$SCRIPT_DIR/../app"
	tar czf "$SERVE_DIR/$TARBALL" \
		--exclude='.cache' \
		--exclude='build' \
		--exclude='install' \
		--exclude='log' \
		src/

	SIZE=$(du -sh "$SERVE_DIR/$TARBALL" | cut -f1)
	echo "  → $TARBALL ($SIZE)"

	# Checksum for integrity verification on RK3588
	cd "$SERVE_DIR"
	sha256sum "$TARBALL" > "${TARBALL}.sha256"
	echo "  → sha256: $(cat "$SERVE_DIR/${TARBALL}.sha256")"
else
	echo "===== Skipping pack (--no-pack) ====="
fi

# ── Step 2: Detect IP ────────────────────────────────────────────────
IFACE=$(ip -4 route show default 2>/dev/null | awk '{print $5}' | head -1)
IP_ADDR=$(ip -4 addr show "$IFACE" 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -1)

echo ""
echo "===== Serving on http://${IP_ADDR}:8080 ====="
echo ""
echo "  On RK3588 run:"
echo "    fetch_app.sh ${IP_ADDR}:8080"
echo ""

cd "$SERVE_DIR"
python3 -m http.server 8080
