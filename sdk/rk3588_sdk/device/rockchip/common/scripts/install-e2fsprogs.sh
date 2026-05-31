#!/bin/sh

# 1.46.6 has -d option and without metadata_csum_seed and orphan_file features
E2FSPROGS_VER="v1.46.6"
echo "git clone https://git.kernel.org/pub/scm/fs/ext2/e2fsprogs.git --depth 1 -b $E2FSPROGS_VER"
echo "cd e2fsprogs"
echo "./configure"
echo "sudo make install -j8 -k -i"
