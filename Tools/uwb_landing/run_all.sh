#!/usr/bin/env bash
# Task 7 - A-vs-B precision-landing sweep.
#
# For each seed and each scenario (A = GPS-only, B = GPS+UWB), reboot SITL so SIM_GPS_SEED is
# applied at boot and A & B see the SAME GPS bias trace per seed (fair). Each run = two short
# headless boots: a CONFIG boot (set seed + scenario params, save) and a MEASURE boot (run
# mission.py, which flies the box and lands). Then show the A-vs-B comparison charts.
#
#   Tools/uwb_landing/run_all.sh [N_seeds] [base_seed] [laps]
#       N_seeds   number of seeds per scenario (default 3)
#       base_seed first SIM_GPS_SEED (default 1; uses base..base+N-1)
#       laps      box laps per flight (default 1)
set -u
PX4_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PX4_DIR"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
: "${MISSION:=$PX4_DIR/Tools/uwb_landing/mission.py}"
ANALYZE="$PX4_DIR/Tools/uwb_landing/analyze.py"
RESULTS="$PX4_DIR/Tools/uwb_landing/results"
MODEL="gz_f450-uwb_uwb"
: "${PX4_LAUNCH:=make px4_sitl $MODEL}"

N="${1:-3}"; BASE="${2:-1}"; LAPS="${3:-1}"
mkdir -p "$RESULTS"

cleanup_gz() {
	pkill -f "gz sim" 2>/dev/null
	pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
	pkill -f "make px4_sitl" 2>/dev/null
	sleep 3
}
newest_ulg() { ls -t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1; }

# sim_session <"PARAM=val ...">  <command-to-run-while-flying-or-empty>
# Boots SITL headless (FIFO-controlled). Once ready: applies params (param set + save), then runs
# the command (e.g. mission.py) against the live sim, then shuts the sim down and force-kills gz.
sim_session() {
	local params="$1" cmd="$2"
	local fifo outlog
	fifo="$(mktemp -u).pxh"; mkfifo "$fifo"
	outlog="$(mktemp)"
	( HEADLESS=1 GZ_VERBOSE=0 $PX4_LAUNCH >"$outlog" 2>&1 <"$fifo" ) &
	local mkpid=$!
	{
		local i
		for i in $(seq 1 180); do
			grep -q "Startup script returned successfully\|Ready for takeoff" "$outlog" 2>/dev/null && break
			kill -0 "$mkpid" 2>/dev/null || break
			sleep 1
		done
		sleep 3
		if [ -n "$params" ]; then
			local p
			for p in $params; do echo "param set ${p%%=*} ${p##*=}"; sleep 0.3; done
			echo "param save"; sleep 2
		fi
		if [ -n "$cmd" ]; then
			eval "$cmd" >&2   # mission.py output -> terminal (stderr), not the pxh stdin fifo
		fi
		sleep 1; echo "shutdown"; sleep 2
	} > "$fifo"
	local i
	for i in $(seq 1 40); do
		grep -q "Exiting NOW\|Shutting down" "$outlog" 2>/dev/null && break
		kill -0 "$mkpid" 2>/dev/null || break
		sleep 1
	done
	sleep 2
	cleanup_gz
	kill "$mkpid" 2>/dev/null
	rm -f "$fifo" "$outlog"
}

echo "=== A-vs-B sweep: $N seeds (base $BASE), laps=$LAPS — this takes a while ==="
cleanup_gz
ULGS=()
for i in $(seq 0 $((N - 1))); do
	SEED=$((BASE + i))
	for SCEN in A B; do
		if [ "$SCEN" = A ]; then CTRL=0; GPS=0; else CTRL=1; GPS=1; fi
		echo "--- seed $SEED  scenario $SCEN  (config boot) ---"
		sim_session "SIM_GPS_SEED=$SEED EKF2_UWB_CTRL=$CTRL EKF2_UWB_GPS=$GPS" ""
		echo "--- seed $SEED  scenario $SCEN  (measure boot) ---"
		sim_session "" "python3 '$MISSION' --scenario $SCEN --seed $SEED --laps $LAPS"
		U="$(newest_ulg)"; ULGS+=("$U")
		echo "    ulog: $U"
	done
done

echo; echo "=== ANALYSIS (A vs B) ==="
python3 "$ANALYZE" "${ULGS[@]}" --compare
