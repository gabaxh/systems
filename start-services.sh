#!/usr/bin/env bash
set -uo pipefail

# checks for root
if [ ! "`whoami`" = "root" ]
then
    echo "Please run script as root."
    exit 1
fi

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${XDG_RUNTIME_DIR:-/tmp}/mbaigo-systems-$USER"
BIN_DIR="$RUN_DIR/bin"
LOG_DIR="$RUN_DIR/log"

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 250000

CORE=(ca maitreD esr orchestrator authorizer)
mkdir -p "$BIN_DIR" "$LOG_DIR"

# Discover all services
discover_all() {
	local service core skip mod

	for core in "${CORE[@]}"; do
		[[ -f "$ROOT/$core/go.mod" ]] && printf '%s\n' "$core"
	done

	for mod in "$ROOT"/*/go.mod; do
		service="$(basename "$(dirname "$mod")")"
		skip=false
		for core in "${CORE[@]}"; do
			[[ "$service" == "$core" ]] && skip=true
		done
		$skip || printf '%s\n' "$service"
	done
}

if [[ "${1:-}" == "all" ]]; then
	mapfile -t services < <(discover_all)
else
	services=("$@")
fi

if [[ ${#services[@]} -eq 0 ]]; then
	echo "Usage: $0 all | SERVICE [SERVICE ...]" >&2
	exit 2
fi

# Runtime files are resolved relative to each system's working directory. A
# first manual run generates systemconfig.json and exits for operator review.
missing=false
for service in "${services[@]}"; do
	if [[ ! -f "$ROOT/$service/go.mod" ]]; then
		echo "Unknown service: $service" >&2
		exit 2
	fi
	if [[ ! -f "$ROOT/$service/systemconfig.json" ]]; then
		echo "$service has no systemconfig.json; generate it with:"
		echo "  (cd \"$ROOT/$service\" && go run .)"
		missing=true
	fi
done
$missing && exit 1

pids=()

shutdown() {
	trap - INT TERM EXIT

	for pid in "${pids[@]}"; do
		kill -INT "$pid" 2>/dev/null || true
	done
	wait 2>/dev/null || true
}
trap shutdown INT TERM EXIT

for service in "${services[@]}"; do
	echo "Building $service"
	(
		cd "$ROOT/$service"
		go build -o "$BIN_DIR/$service" .
	) || exit 1

	echo "Starting $service; log: $LOG_DIR/$service.log"
	(
		cd "$ROOT/$service"
		exec "$BIN_DIR/$service"
	) >>"$LOG_DIR/$service.log" 2>&1 &

	pids+=("$!")

	# Reduce startup races between core systems and their consumers.
	sleep 1
done

echo "Running ${#pids[@]} services; press Ctrl-C to stop."
status=0
wait -n "${pids[@]}" || status=$?
echo "A service exited with status $status; stopping the group." >&2
exit "$status"
