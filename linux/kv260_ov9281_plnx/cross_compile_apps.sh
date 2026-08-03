#!/usr/bin/env bash
#
# cross_compile_apps.sh — build the meta-user recipes-apps C/C++ apps for the
# KV260 target WITHOUT a full petalinux/bitbake run.
#
# It reuses the cross toolchain and target libraries that PetaLinux already
# staged under build/tmp/sysroots-components/ (these persist even with rm_work,
# unlike the per-recipe recipe-sysroot/ dirs). From those components it stitches
# together a single merged sysroot once, then compiles each app's Makefile with
# the exact same tune flags the recipes use (-mcpu=cortex-a72.cortex-a53 ...).
#
# Usage:
#   ./cross_compile_apps.sh                 # build every app with a Makefile
#   ./cross_compile_apps.sh mocap-hdmi-drm  # build just one (or several) apps
#   ./cross_compile_apps.sh --clean         # wipe merged sysroot + outputs
#                                           #   (+ deployed copies in the NFS root)
#   ./cross_compile_apps.sh --deploy <app>  # build then copy into the NFS root
#
# Binaries land in   cross_build_out/<app>/<app>
# and can be copied straight onto the NFS-rooted target's /usr/bin.
#
set -euo pipefail

# --- locate ourselves / the project ------------------------------------------
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$PROJ/build/tmp"
APPS_DIR="$PROJ/project-spec/meta-user/recipes-apps"
NFS_BIN="$PROJ/../nfs-mount-point/usr/bin"           # NFS-root target /usr/bin
OUT="$PROJ/cross_build_out"

# Target tune (copied verbatim from the recipes' do_compile line).
TUNE="-mcpu=cortex-a72.cortex-a53 -march=armv8-a+crc"
TARGET_ARCH="cortexa72-cortexa53"

COMP="$TMP/sysroots-components/$TARGET_ARCH"          # target component sysroots
X86="$TMP/sysroots-components/x86_64"                 # host (cross) toolchain
SYSROOT="$TMP/cross-sysroot"                          # merged sysroot we build
TOOLBIN="$TMP/cross-toolbin"                          # unprefixed as/ld symlinks

# Target components merged into the sysroot. Order is irrelevant; paths are
# mostly disjoint (usr/include, usr/lib). Add more here if an app grows a dep.
COMPONENTS=(glibc gcc-runtime libgcc linux-libc-headers libdrm mocap-common)

# --- args --------------------------------------------------------------------
DO_CLEAN=0; DO_DEPLOY=0; SELECT=()
for a in "$@"; do
    case "$a" in
        --clean)  DO_CLEAN=1 ;;
        --deploy) DO_DEPLOY=1 ;;
        -*) echo "unknown option: $a" >&2; exit 2 ;;
        *)  SELECT+=("$a") ;;
    esac
done

log()  { printf '\033[1;36m>>> %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

if [ "$DO_CLEAN" = 1 ]; then
    log "cleaning merged sysroot, toolbin, and $OUT"
    rm -rf "$SYSROOT" "$TOOLBIN" "$OUT"

    # Also drop the deployed copies from the NFS root. Only remove binaries we
    # know we produce (named after a recipe), never a blanket rm on /usr/bin.
    # Scope: the selected apps, or every app with a Makefile if none given.
    clean_apps=("${SELECT[@]}")
    if [ ${#clean_apps[@]} -eq 0 ]; then
        for d in "$APPS_DIR"/*/; do
            [ -f "$d/files/Makefile" ] && clean_apps+=("$(basename "$d")")
        done
    fi
    if [ -d "$NFS_BIN" ]; then
        for app in "${clean_apps[@]}"; do
            if [ -e "$NFS_BIN/$app" ]; then
                sudo rm -f "$NFS_BIN/$app" && echo "    removed $NFS_BIN/$app"
            fi
        done
    fi

    [ ${#SELECT[@]} -eq 0 ] && exit 0   # --clean on its own just cleans
fi

# --- sanity: toolchain + components present -----------------------------------
GCC_BIN="$X86/gcc-cross-aarch64/usr/bin/aarch64-xilinx-linux"
BU_BIN="$X86/binutils-cross-aarch64/usr/bin/aarch64-xilinx-linux"
[ -x "$GCC_BIN/aarch64-xilinx-linux-g++" ] || \
    die "cross g++ not found under $GCC_BIN (has this project been built at all?)"
[ -x "$BU_BIN/aarch64-xilinx-linux-as" ] || \
    die "cross binutils not found under $BU_BIN"
[ -d "$COMP/glibc" ] || \
    die "target component sysroots missing under $COMP"

# --- 1. unprefixed as/ld/... so the gcc driver finds the cross assembler ------
# The gcc driver invokes 'as'/'ld' unprefixed; the staged binutils are named
# aarch64-xilinx-linux-as etc., so we expose plain-named symlinks and -B here.
if [ ! -e "$TOOLBIN/as" ]; then
    log "building toolbin (unprefixed binutils symlinks)"
    mkdir -p "$TOOLBIN"
    for t in as ld ld.bfd ar nm ranlib strip objcopy objdump; do
        [ -e "$BU_BIN/aarch64-xilinx-linux-$t" ] && \
            ln -sf "$BU_BIN/aarch64-xilinx-linux-$t" "$TOOLBIN/$t"
    done
fi

# --- 2. merged sysroot from the component sysroots ----------------------------
# rm_work never touches sysroots-components/, so this is stable regardless of
# which recipes were last built. Hardlinked (cp -al) so it's near-instant/cheap.
if [ ! -d "$SYSROOT/usr/include" ]; then
    log "assembling merged sysroot at $SYSROOT"
    mkdir -p "$SYSROOT"
    for c in "${COMPONENTS[@]}"; do
        [ -d "$COMP/$c" ] || die "expected component sysroot missing: $COMP/$c"
        cp -al "$COMP/$c/." "$SYSROOT/" 2>/dev/null || cp -a "$COMP/$c/." "$SYSROOT/"
    done
fi

# --- 3. build environment (mirrors the recipe's do_compile) -------------------
export PATH="$GCC_BIN:$BU_BIN:$PATH"
export CXX="aarch64-xilinx-linux-g++ $TUNE -B$TOOLBIN --sysroot=$SYSROOT"
export CC="aarch64-xilinx-linux-gcc $TUNE -B$TOOLBIN --sysroot=$SYSROOT"
# pkg-config resolves libdrm etc. against the merged sysroot (not the host).
export PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
export PKG_CONFIG_LIBDIR="$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig"
unset PKG_CONFIG_PATH

# --- 4. discover apps (any recipes-apps/*/files/Makefile) ---------------------
discover() {
    for d in "$APPS_DIR"/*/; do
        local name; name="$(basename "$d")"
        [ -f "$d/files/Makefile" ] && echo "$name"
    done
}

if [ ${#SELECT[@]} -gt 0 ]; then
    APP_LIST=("${SELECT[@]}")
else
    mapfile -t APP_LIST < <(discover)
fi

# --- 5. build each app --------------------------------------------------------
mkdir -p "$OUT"
built=(); failed=()
for app in "${APP_LIST[@]}"; do
    src="$APPS_DIR/$app/files"
    [ -f "$src/Makefile" ] || { echo "skip $app (no files/Makefile)"; continue; }

    log "building $app"
    bdir="$TMP/cross-build/$app"
    rm -rf "$bdir"; mkdir -p "$bdir"
    cp -a "$src/." "$bdir/"           # build out-of-tree; keep source dir clean

    if make -C "$bdir" clean >/dev/null 2>&1 && make -C "$bdir"; then
        # binary is named after the recipe (APP = <app> in every Makefile)
        if [ -f "$bdir/$app" ]; then
            mkdir -p "$OUT/$app"
            cp -a "$bdir/$app" "$OUT/$app/$app"
            built+=("$app")
            printf '    -> %s\n' "$OUT/$app/$app"
            if [ "$DO_DEPLOY" = 1 ]; then
                if [ -d "$NFS_BIN" ]; then
                    sudo cp "$OUT/$app/$app" "$NFS_BIN/" && \
                        printf '    deployed -> %s/%s\n' "$NFS_BIN" "$app"
                else
                    echo "    (deploy skipped: $NFS_BIN not found)"
                fi
            fi
        else
            echo "    built but no '$app' binary produced?"; failed+=("$app")
        fi
    else
        failed+=("$app")
    fi
done

# --- summary ------------------------------------------------------------------
echo
log "done: ${#built[@]} built, ${#failed[@]} failed"
[ ${#built[@]}  -gt 0 ] && printf '  ok:     %s\n' "${built[*]}"
[ ${#failed[@]} -gt 0 ] && { printf '  FAILED: %s\n' "${failed[*]}"; exit 1; }

cat <<EOF

Binaries are under $OUT/<app>/<app>.
Copy one onto the (NFS-rooted) target, e.g.:
  sudo cp $OUT/mocap-hdmi-drm/mocap-hdmi-drm \\
      $NFS_BIN/
(or re-run with:  ./cross_compile_apps.sh --deploy mocap-hdmi-drm)
EOF
