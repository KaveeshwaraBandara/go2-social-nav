#!/usr/bin/env bash
# Convenience wrapper around docker compose that also handles X11 access so
# Gazebo/RViz windows from the container appear on your host display.
#
# Usage:
#   ./run.sh build          Build the image
#   ./run.sh up             Grant X11 access + start container, drop into a shell
#   ./run.sh shell          Open another shell in the running container
#   ./run.sh exec <cmd...>  Run a command in the running container
#   ./run.sh down           Stop container + revoke X11 access
#
# NVIDIA: prefix with PROFILE=nvidia (e.g. PROFILE=nvidia ./run.sh up) on a
# host that has an NVIDIA GPU + nvidia-container-toolkit.
set -euo pipefail

cd "$(dirname "$0")"

SERVICE="ros2"
COMPOSE_ARGS=()
if [ "${PROFILE:-}" = "nvidia" ]; then
    SERVICE="ros2-nvidia"
    COMPOSE_ARGS=(--profile nvidia)
fi

grant_x11() {
    # Authorize local container clients to use the X server (Xwayland here).
    # Scoped to local connections; revoked again on 'down'.
    if command -v xhost >/dev/null 2>&1; then
        xhost +local: >/dev/null
        echo "[run.sh] X11 access granted to local clients (xhost +local:)"
    else
        echo "[run.sh] WARNING: xhost not found; GUI windows may not appear."
    fi
}

revoke_x11() {
    if command -v xhost >/dev/null 2>&1; then
        xhost -local: >/dev/null || true
        echo "[run.sh] X11 access revoked (xhost -local:)"
    fi
}

cmd="${1:-}"
shift || true

case "$cmd" in
    build)
        docker compose "${COMPOSE_ARGS[@]}" build "$@"
        ;;
    up)
        grant_x11
        docker compose "${COMPOSE_ARGS[@]}" up -d "$@"
        echo "[run.sh] Container up. Dropping into a shell (Ctrl-D to leave; container keeps running)."
        docker compose "${COMPOSE_ARGS[@]}" exec "$SERVICE" bash
        ;;
    shell)
        docker compose "${COMPOSE_ARGS[@]}" exec "$SERVICE" bash
        ;;
    exec)
        docker compose "${COMPOSE_ARGS[@]}" exec "$SERVICE" "$@"
        ;;
    down)
        docker compose "${COMPOSE_ARGS[@]}" down
        revoke_x11
        ;;
    *)
        echo "Usage: ./run.sh {build|up|shell|exec <cmd>|down}   (PROFILE=nvidia for GPU host)"
        exit 1
        ;;
esac
