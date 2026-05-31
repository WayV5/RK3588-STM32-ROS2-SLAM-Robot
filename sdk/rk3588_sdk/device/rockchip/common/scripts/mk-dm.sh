#!/bin/bash -e

MODE=$1
INPUT="$(readlink -f "$2")"
OUTDIR="$RK_OUTDIR/security-dm"

cd "$SDK_DIR"
mkdir -p "$OUTDIR"

TEMPDIR="$OUTDIR/tempfile"
if [ "$MODE" = "DM-E" ]; then
	ROOTFS="$OUTDIR/enc.img"
	cipher=aes-cbc-plain
	key=$(cat u-boot/keys/system_enc_key)
else
	ROOTFS="$OUTDIR/dmv.img"
fi
ROOT_HASH="$TEMPDIR/root.hash"
ROOT_HASH_OFFSET="$TEMPDIR/root.offset"
OVERLAY_DIR="$SDK_DIR/buildroot/board/rockchip/common/security-ramdisk-overlay"
INIT_FILE="$OVERLAY_DIR/init"
ROOTFS_INFO=$(ls -l "$INPUT")

PACK=TRUE
if [ -e "$OUTDIR/rootfs.info" ]; then
	if [ "$(cat "$OUTDIR/rootfs.info")" = "$(ls -l "$INPUT")" ]; then
		PACK=FALSE
	else
		echo "$(ls -l "$INPUT")" > "$OUTDIR/rootfs.info"
	fi
else
	echo "$(ls -l "$INPUT")" > "$OUTDIR/rootfs.info"
fi

pack_dmv()
{
	cp "$INPUT" "$ROOTFS"
	HASH_OFFSET=$[(ROOTFS_SIZE / 1024 / 1024 + 2) * 1024 * 1024]
	tempfile=$(mktemp)
	veritysetup --hash-offset=$HASH_OFFSET format "$ROOTFS" "$ROOTFS" > \
		$tempfile
	cat $tempfile | grep "Root hash" | awk '{printf $3}' > "$ROOT_HASH"

	cp $tempfile "$TEMPDIR/tempfile"
	echo $HASH_OFFSET > "$ROOT_HASH_OFFSET"
}

pack_dme()
{
	sectors=$(ls -l "$INPUT" | awk '{printf $5}')
	sectors=$[(sectors + (21 * 1024 * 1024) - 1) / 512] # remain 20M for partition info / unit: 512 bytes

	loopdevice=$(losetup -f)
	mappername=encfs-$(shuf -i 1-10000000000000000000 -n 1)
	dd if=/dev/null of="$ROOTFS" seek=$sectors bs=512
	sudo -S losetup $loopdevice "$ROOTFS" < u-boot/keys/root_passwd
	sudo -S dmsetup create $mappername --table "0 $sectors crypt $cipher $key 0 $loopdevice 0 1 allow_discards" < u-boot/keys/root_passwd
	sudo -S dd if="$INPUT" of=/dev/mapper/$mappername conv=fsync < u-boot/keys/root_passwd
	sync && sudo -S dmsetup remove $mappername < u-boot/keys/root_passwd
	sudo -S losetup -d $loopdevice < u-boot/keys/root_passwd

	rm "$TEMPDIR/enc.info" || true
	echo "sectors=$sectors" > "$TEMPDIR/enc.info"
	echo "cipher=$cipher" >> "$TEMPDIR/enc.info"
	echo "key=$key" >> "$TEMPDIR/enc.info"
}

make_misc() {
	INPUT=$1
	OUTPUT=$2
	SIZE=$3
	BUF=$4

	BIG_END=$[SIZE / 256]
	LIT_END=$[SIZE - (BIG_END * 256)]
	BIG_END=$(echo "ibase=10;obase=16;$BIG_END" | bc)
	LIT_END=$(echo "ibase=10;obase=16;$LIT_END" | bc)

	rm -rf "$OUTPUT"
	dd if="$INPUT" of="$OUTPUT" bs=1k count=10
	echo -en "\x$LIT_END\x$BIG_END" >> "$OUTPUT"
	echo -n "$BUF" >> "$OUTPUT"
	SKIP=$[10 * 1024 + SIZE + 2]
	dd if="$INPUT" of="$OUTPUT" seek=$SKIP skip=$SKIP bs=1
}

if [ "$PACK" = "TRUE" ]; then
	mkdir -p "$TEMPDIR"
	ROOTFS_SIZE=$(ls "$INPUT" -l | awk '{printf $5}')

	if [ "$MODE" = "DM-V" ]; then
		pack_dmv
	elif [ "$MODE" = "DM-E" ]; then
		pack_dme
	fi

	ln -rsf "$ROOTFS" "$RK_SECURITY_FIRMWARE_DIR/rootfs.img"
fi

cp "$OVERLAY_DIR/init.in" "$INIT_FILE"

if [ "$MODE" = "DM-V" ]; then
	TMP_HASH=$(cat "$ROOT_HASH")
	TMP_OFFSET=$(cat "$ROOT_HASH_OFFSET")
	sed -i "s/OFFSET=/OFFSET=$TMP_OFFSET/" "$INIT_FILE"
	sed -i "s/HASH=/HASH=$TMP_HASH/" "$INIT_FILE"
	sed -i "s/ENC_EN=/ENC_EN=false/" "$INIT_FILE"
elif [ "$MODE" = "DM-E" ]; then
	source "$TEMPDIR/enc.info"

	sed -i "s/ENC_EN=/ENC_EN=true/" "$INIT_FILE"
	sed -i "s/CIPHER=/CIPHER=$cipher/" "$INIT_FILE"

	echo "Generate misc with key"
	make_misc "$RK_IMAGE_DIR/misc/$RK_MISC_IMG" \
		"$RK_SECURITY_FIRMWARE_DIR/misc.img" 64 \
		$(cat "$SDK_DIR/u-boot/keys/system_enc_key")
fi

sed -i "s/# exec busybox switch_root/exec busybox switch_root/" "$INIT_FILE"

rm -rf "$TEMPDIR"
