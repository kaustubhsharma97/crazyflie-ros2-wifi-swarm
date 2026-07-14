#!/bin/bash
# find_decks.sh — pre-flight AI-deck IP check (no router access needed)
# Finds both AI-decks on the LAN by MAC, prints current IPs, and checks
# that the CPX port (5000) is accepting connections.
# Run before every session; update crazyflies.yaml if an IP changed.

MAC_LEADER="78:21:84:bc:15:1c"    # drone 04 / aideck-BC151C / cf231 (leader) — NEW deck; verify MAC against console
MAC_FOLLOWER="78:21:84:77:92:64"  # drone 03 / aideck-779264 / cf2   (follower)

SCAN=$(sudo arp-scan --localnet 2>/dev/null)

for entry in "LEADER cf231 $MAC_LEADER" "FOLLOWER cf2 $MAC_FOLLOWER"; do
    set -- $entry
    ROLE=$1; NAME=$2; MAC=$3
    IP=$(echo "$SCAN" | grep -i "$MAC" | awk '{print $1}' | head -n1)
    if [ -z "$IP" ]; then
        echo "$ROLE ($NAME, $MAC): NOT FOUND — drone off or not on Wi-Fi yet"
        continue
    fi
    if timeout 2 bash -c "echo > /dev/tcp/$IP/5000" 2>/dev/null; then
        PORT="port 5000 OPEN"
    else
        PORT="port 5000 REFUSED — socket busy or deck not ready (power-cycle drone)"
    fi
    echo "$ROLE ($NAME): $IP   [$PORT]"
    echo "    yaml:  uri: tcp://$IP:5000"
done
