#!/usr/bin/env bash
# Self-test for the simulated NEO-M9N GPS noise model (gz_bridge::addGpsNoise).
#
# Runs several HEADLESS SITL sessions (drone idle on the ground), driving the pxh
# shell over a FIFO to set params + clean-shutdown, then validates the resulting
# ulogs with _gps_noise_check.py. No human in the loop.
#
#   Tools/uwb_landing/verify_gps_noise.sh
#
# Sessions:  baseline(seed1,nsc1) -> A, A2  | nsc0 -> Z  | seed2 -> S2  | restore.
set -u
PX4_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PX4_DIR"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
CHECK="$PX4_DIR/Tools/uwb_landing/_gps_noise_check.py"
MODEL="gz_f450-uwb_uwb"
# Launch command (override for self-testing the harness with a fake px4):
: "${PX4_LAUNCH:=make px4_sitl $MODEL}"

cleanup_gz() {
	pkill -f "gz sim" 2>/dev/null
	pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
	pkill -f "make px4_sitl" 2>/dev/null
	sleep 3
}
newest_ulg() { ls -t "$ROOTFS"/log/*/*.ulg 2>/dev/null | head -1; }

# run_session <idle_seconds> <"PARAM=val PARAM=val"...>
# Boots SITL headless; once ready, applies params (param set + save) then idles and shuts down.
run_session() {
	local idle="$1"; shift
	local params="$*"
	[ -n "${PX4_IDLE_OVERRIDE:-}" ] && idle="$PX4_IDLE_OVERRIDE"
	local fifo outlog
	fifo="$(mktemp -u).pxh"; mkfifo "$fifo"
	outlog="$(mktemp)"
	echo "  [session] idle=${idle}s params='${params}'  (boot log: $outlog)"
	( HEADLESS=1 GZ_VERBOSE=0 $PX4_LAUNCH >"$outlog" 2>&1 <"$fifo" ) &
	local mkpid=$!
	{
		# wait for boot
		local i
		for i in $(seq 1 180); do
			grep -q "Startup script returned successfully\|Ready for takeoff" "$outlog" 2>/dev/null && break
			kill -0 "$mkpid" 2>/dev/null || break
			sleep 1
		done
		sleep 3
		if [ -n "$params" ]; then
			local p
			for p in $params; do
				echo "param set ${p%%=*} ${p##*=}"
				sleep 0.4
			done
			echo "param save"
			sleep 2
		fi
		sleep "$idle"
		echo "shutdown"
		sleep 2
	} > "$fifo"
	# px4 'shutdown' closes the ulog and exits, but the gz sim child (started '&' by rcS) and the
	# make wrapper do NOT exit on their own -> never 'wait' on them. Wait briefly for px4 to finish
	# closing the log, then force-kill the lingering gz + make tree.
	local i
	for i in $(seq 1 20); do
		grep -q "Exiting NOW\|Shutting down\|process exited" "$outlog" 2>/dev/null && break
		kill -0 "$mkpid" 2>/dev/null || break
		sleep 1
	done
	sleep 2
	cleanup_gz
	kill "$mkpid" 2>/dev/null
	rm -f "$fifo" "$outlog"
}

echo "=== GPS-noise self-test (this takes several minutes) ==="
cleanup_gz

# Baseline: GPS on, UWB off, realistic noise, seed 1.
echo "--- config baseline (seed1, nsc1, GPS-only) ---"
run_session 0 "EKF2_GPS_CTRL=7 EKF2_UWB_CTRL=0 SIM_GPS_SEED=1 SIM_GPS_NSC=1"

echo "--- measure A (120s) ---"; run_session 120 ""; A="$(newest_ulg)"; echo "  A=$A"
echo "--- measure A2 (120s, repro) ---"; run_session 120 ""; A2="$(newest_ulg)"; echo "  A2=$A2"

echo "--- config nsc0 ---"; run_session 0 "SIM_GPS_NSC=0"
echo "--- measure Z (60s) ---"; run_session 60 ""; Z="$(newest_ulg)"; echo "  Z=$Z"

echo "--- config seed2 ---"; run_session 0 "SIM_GPS_SEED=2 SIM_GPS_NSC=1"
echo "--- measure S2 (120s) ---"; run_session 120 ""; S2="$(newest_ulg)"; echo "  S2=$S2"

echo "--- restore defaults (seed1, nsc1) ---"; run_session 0 "SIM_GPS_SEED=1 SIM_GPS_NSC=1"

echo; echo "=== RESULTS ==="
rc=0
python3 "$CHECK" realism  "$A"      || rc=1
python3 "$CHECK" zero     "$Z"      || rc=1
python3 "$CHECK" repro    "$A" "$A2" || rc=1
python3 "$CHECK" seeddiff "$A" "$S2" || rc=1

echo "================================================"
[ $rc -eq 0 ] && echo "ALL CHECKS PASSED ✅" || echo "SOME CHECKS FAILED ❌"
exit $rc
