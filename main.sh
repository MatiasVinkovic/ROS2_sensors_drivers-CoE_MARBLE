#!/usr/bin/env bash
#
# setup-audioinjector-octo.sh
#
# Provisions a fresh Raspberry Pi 4B for the AudioInjector Octo sound card
# (CS42448 codec). Idempotent: safe to re-run.
#
# Usage:
#   sudo ./setup-audioinjector-octo.sh            # configure + reboot
#   sudo ./setup-audioinjector-octo.sh --no-reboot # configure, skip reboot
#   sudo ./setup-audioinjector-octo.sh --check     # only run post-boot checks
#
set -euo pipefail

CONFIG_TXT="/boot/firmware/config.txt"
MODPROBE_FILE="/etc/modprobe.d/audioinjector-octo.conf"
SERVICE_FILE="/etc/systemd/system/audioinjector-fix.service"
MODE="setup"

for arg in "${@:-}"; do
    case "$arg" in
        --check)     MODE="check" ;;
        --no-reboot) MODE="setup-no-reboot" ;;
    esac
done

log()  { echo -e "\033[1;34m[*]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[OK]\033[0m $*"; }
warn() { echo -e "\033[1;33m[!]\033[0m $*"; }
fail() { echo -e "\033[1;31m[FAIL]\033[0m $*"; }

if [[ $EUID -ne 0 ]]; then
    fail "This script must be run as root (use sudo)."
    exit 1
fi

# ---------------------------------------------------------------------------
# Verification block (used both standalone with --check and at the end of
# a full setup run, though config.txt changes only take effect after reboot)
# ---------------------------------------------------------------------------
run_checks() {
    local all_ok=1

    log "Checking I2C bus for codec at address 0x48 ..."
    if command -v i2cdetect >/dev/null 2>&1; then
        if i2cdetect -y 1 | grep -qE '\b48\b'; then
            ok "Codec responds on I2C bus (0x48)."
        else
            fail "No response at I2C address 0x48."
            all_ok=0
        fi
    else
        warn "i2cdetect not installed, skipping I2C check."
        all_ok=0
    fi

    log "Checking kernel modules ..."
    if lsmod | grep -qE "cs42xx8|audioinjector"; then
        ok "Codec/soundcard kernel modules are loaded."
    else
        fail "Expected kernel modules not loaded."
        all_ok=0
    fi

    log "Checking dmesg for successful codec probe ..."
    if dmesg | grep -q "cs42xx8.*found device"; then
        ok "Kernel log confirms: codec found."
    else
        warn "No 'found device' line in dmesg (may need the fix service to run, or a reboot)."
        all_ok=0
    fi

    log "Checking ALSA card registration ..."
    if aplay -l 2>/dev/null | grep -qi "audioinjector-octo"; then
        ok "Card visible in 'aplay -l'."
    else
        fail "Card NOT visible in 'aplay -l'."
        all_ok=0
    fi

    if grep -qi "audioinjector-o" /proc/asound/cards 2>/dev/null; then
        ok "Card visible in /proc/asound/cards."
    else
        fail "Card NOT visible in /proc/asound/cards."
        all_ok=0
    fi

    echo
    if [[ $all_ok -eq 1 ]]; then
        ok "All checks passed. Card appears ready. Try: speaker-test -c 8 -D hw:<card>,0 -t wav"
    else
        warn "Some checks failed. If this is right after first setup, reboot and re-run with --check."
    fi
}

if [[ "$MODE" == "check" ]]; then
    run_checks
    exit 0
fi

# ---------------------------------------------------------------------------
# 1. Install prerequisites
# ---------------------------------------------------------------------------
log "Installing i2c-tools (needed for i2cdetect) ..."
if ! command -v i2cdetect >/dev/null 2>&1; then
    apt-get update -qq
    apt-get install -y i2c-tools
    ok "i2c-tools installed."
else
    ok "i2c-tools already present."
fi

# ---------------------------------------------------------------------------
# 2. Configure /boot/firmware/config.txt
# ---------------------------------------------------------------------------
log "Backing up $CONFIG_TXT ..."
cp -n "$CONFIG_TXT" "${CONFIG_TXT}.bak.$(date +%Y%m%d%H%M%S)" || true

log "Ensuring dtparam=i2c_arm=on is enabled ..."
if grep -qE '^#\s*dtparam=i2c_arm=on' "$CONFIG_TXT"; then
    sed -i 's/^#\s*dtparam=i2c_arm=on/dtparam=i2c_arm=on/' "$CONFIG_TXT"
    ok "Uncommented dtparam=i2c_arm=on."
elif grep -qE '^dtparam=i2c_arm=on' "$CONFIG_TXT"; then
    ok "dtparam=i2c_arm=on already enabled."
else
    echo "dtparam=i2c_arm=on" >> "$CONFIG_TXT"
    ok "Added dtparam=i2c_arm=on."
fi

log "Removing obsolete dtoverlay=audioinjector-octo (no longer shipped) ..."
if grep -qE '^dtoverlay=audioinjector-octo\s*$' "$CONFIG_TXT"; then
    sed -i '/^dtoverlay=audioinjector-octo\s*$/d' "$CONFIG_TXT"
    warn "Removed obsolete 'dtoverlay=audioinjector-octo' line."
fi

log "Ensuring dtoverlay=audioinjector-addons is present ..."
if grep -qE '^dtoverlay=audioinjector-addons' "$CONFIG_TXT"; then
    ok "dtoverlay=audioinjector-addons already present."
else
    echo "dtoverlay=audioinjector-addons" >> "$CONFIG_TXT"
    ok "Added dtoverlay=audioinjector-addons."
fi

# Sanity check: does the overlay file actually exist on this firmware?
if [[ -f /boot/firmware/overlays/audioinjector-addons.dtbo ]]; then
    ok "Confirmed: audioinjector-addons.dtbo exists on this firmware."
else
    fail "audioinjector-addons.dtbo NOT found in /boot/firmware/overlays/. Check your firmware/OS version."
fi

# ---------------------------------------------------------------------------
# 3. Create /etc/modprobe.d/audioinjector-octo.conf (single line, no wrap)
# ---------------------------------------------------------------------------
log "Writing $MODPROBE_FILE ..."
printf 'softdep snd_soc_audioinjector_octo_soundcard pre: snd_soc_cs42xx8 snd_soc_cs42xx8_i2c\n' > "$MODPROBE_FILE"

LINE_COUNT=$(wc -l < "$MODPROBE_FILE")
if [[ "$LINE_COUNT" -eq 1 ]]; then
    ok "$MODPROBE_FILE written correctly (single line)."
else
    fail "$MODPROBE_FILE has $LINE_COUNT lines instead of 1 — check for corruption."
    exit 1
fi

# ---------------------------------------------------------------------------
# 4. Create the systemd fix service
# ---------------------------------------------------------------------------
log "Writing $SERVICE_FILE ..."
cat > "$SERVICE_FILE" << 'EOF'
[Unit]
Description=Fix AudioInjector Octo probe race condition
After=sound.target

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 5
ExecStart=/sbin/modprobe -r snd_soc_audioinjector_octo_soundcard
ExecStart=/sbin/modprobe -r snd_soc_cs42xx8_i2c
ExecStart=/sbin/modprobe -r snd_soc_cs42xx8
ExecStart=/sbin/modprobe snd_soc_cs42xx8
ExecStart=/sbin/modprobe snd_soc_cs42xx8_i2c
ExecStart=/sbin/modprobe snd_soc_audioinjector_octo_soundcard
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
ok "$SERVICE_FILE written."

# ---------------------------------------------------------------------------
# 5. Enable the service
# ---------------------------------------------------------------------------
log "Enabling audioinjector-fix.service ..."
systemctl daemon-reload
systemctl enable audioinjector-fix.service
systemctl start audioinjector-fix.service || warn "Service start returned non-zero (may be normal before first reboot)."
ok "Service enabled and started."

# ---------------------------------------------------------------------------
# 6. Run checks (best-effort — full success expected only after reboot,
#    since the dtoverlay change in config.txt is not active yet)
# ---------------------------------------------------------------------------
echo
log "Running best-effort checks (config.txt changes need a reboot to fully apply) ..."
run_checks || true

# ---------------------------------------------------------------------------
# 7. Reboot
# ---------------------------------------------------------------------------
echo
if [[ "$MODE" == "setup-no-reboot" ]]; then
    warn "Setup complete. Reboot skipped (--no-reboot given)."
    warn "Reboot manually, then run: sudo $0 --check"
else
    log "Setup complete. Rebooting in 5 seconds to apply config.txt changes ..."
    log "After reboot, verify with: sudo $0 --check"
    sleep 5
    reboot
fi
