#!/usr/bin/env bash
set -euo pipefail

PI_USER="hii"
PI_IP="192.168.11.147"
PI_TARGET_DIR="/home/hii/wro2026_workspace"

CROSS_IMAGE="cross-pi"

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BUILD_DIR="$SOURCE_DIR/.cross-build"
INSTALL_DIR="$SOURCE_DIR/dist/install"
CCACHE_DIR="$SOURCE_DIR/.ccache-cross"

usage() {
    cat <<EOF
Usage:
  $0            Incremental cross-build + deploy
  $0 clean      Remove CMake build/install artifacts
  $0 clean-all  Remove build/install artifacts and ccache
  $0 cross      Rebuild cross compiler Docker image
EOF
}

build_cross_image() {
    echo "Building cross compiler image: $CROSS_IMAGE"

    docker build \
        -f "$SOURCE_DIR/Dockerfile.cross" \
        -t "$CROSS_IMAGE" \
        "$SOURCE_DIR"
}

case "${1:-}" in
    "")
        ;;

    clean)
        echo "Cleaning build artifacts..."
        rm -rf "$BUILD_DIR" "$SOURCE_DIR/dist"
        echo "Clean finished."
        exit 0
        ;;

    clean-all)
        echo "Cleaning build artifacts and ccache..."
        rm -rf \
            "$BUILD_DIR" \
            "$SOURCE_DIR/dist" \
            "$CCACHE_DIR"

        echo "Clean-all finished."
        exit 0
        ;;

    cross)
        build_cross_image
        exit 0
        ;;

    -h|--help)
        usage
        exit 0
        ;;

    *)
        echo "Unknown command: $1" >&2
        usage
        exit 2
        ;;
esac


if ! docker image inspect "$CROSS_IMAGE" >/dev/null 2>&1; then
    echo "Cross image '$CROSS_IMAGE' not found."
    echo "Building it first..."

    build_cross_image
fi

mkdir -p \
    "$BUILD_DIR" \
    "$INSTALL_DIR" \
    "$CCACHE_DIR"


if [[ -f "$BUILD_DIR/CMakeCache.txt" ]] && \
   ! grep -q '^CMAKE_GENERATOR:INTERNAL=Ninja$' \
       "$BUILD_DIR/CMakeCache.txt"; then

    echo "Old build directory is not using Ninja."
    echo "Recreating build directory..."

    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
fi

echo "Cross-compiling incrementally via Docker..."

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e CCACHE_DIR=/ccache \
    -v "$SOURCE_DIR:/src:ro" \
    -v "$BUILD_DIR:/build" \
    -v "$INSTALL_DIR:/install" \
    -v "$CCACHE_DIR:/ccache" \
    "$CROSS_IMAGE" \
    bash -lc '
        set -euo pipefail

        CCACHE_ARGS=()

        if command -v ccache >/dev/null 2>&1; then
            CCACHE_ARGS+=(
                -DCMAKE_C_COMPILER_LAUNCHER=ccache
                -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
            )
        fi

        cmake \
            -S /src \
            -B /build \
            -G Ninja \
            -DCMAKE_TOOLCHAIN_FILE="$CROSS_TOOLCHAIN" \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/install \
            "${CCACHE_ARGS[@]}"

        cmake --build /build --parallel

        cmake --install /build

        if command -v ccache >/dev/null 2>&1; then
            ccache -s
        fi
    '

echo "Deploying to Raspberry Pi 5 ($PI_IP)..."

ssh "$PI_USER@$PI_IP" \
    "mkdir -p '$PI_TARGET_DIR'"

rsync -av \
    --delete \
    --exclude 'include/' \
    "$INSTALL_DIR/" \
    "$PI_USER@$PI_IP:$PI_TARGET_DIR/"

echo
echo "SUCCESS!"
echo "Deployed to Pi 5 at: $PI_TARGET_DIR"