#!/usr/bin/env bash
# Fails if this concept repo grows a command or a live capability claim.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

fail() {
  echo "credibility check failed: $*" >&2
  exit 1
}

[[ -f LICENSE ]] || fail "LICENSE is missing"
grep -q "Apache License" LICENSE || fail "LICENSE is not Apache-2.0"
grep -q "Version 2.0, January 2004" LICENSE || fail "LICENSE is not Apache-2.0"
grep -q "Copyright 2026 OhhO Robotics" LICENSE || fail "LICENSE copyright line changed"

[[ -f README.md ]] || fail "README.md is missing"
grep -q "Concept — not started." README.md || fail "README is missing the concept status"
grep -q "https://github.com/ohho-robotics/OmniBot" README.md || fail "README is missing the OmniBot link"
grep -q "git clone https://github.com/ohho-robotics/ohho-drone.git" README.md || fail "README clone URL is wrong"
grep -q "Apache License 2.0" README.md || fail "README license line does not match LICENSE"

[[ ! -e docker-compose.yml ]] || fail "docker-compose.yml must not return"
[[ ! -e ohho.repos ]] || fail "ohho.repos must not return"

# Live claims this skeleton used to make. Roadmap items stay unscoped.
forbidden=(
  "MAVLink"
  "PX4"
  "swarm"
  "Digital Twin"
  "docker compose"
  "vcs import"
  "OhhO-Drone"
  "ROSBridge"
  "WebRTC"
  "VLA"
  "Isaac"
  "OhhO Fleet"
  "OhhO Mind"
  "spatial mapping"
)
for phrase in "${forbidden[@]}"; do
  if grep -q "$phrase" README.md; then
    fail "README still claims or documents: $phrase"
  fi
done

# The only fenced command is the public clone.
commands="$(awk '
  /^```(bash|sh)$/ { capture=1; next }
  /^```$/ { capture=0; next }
  capture && NF { print }
' README.md)"
expected="git clone https://github.com/ohho-robotics/ohho-drone.git"
[[ "$commands" == "$expected" ]] || fail "unexpected command in README: ${commands:-<none>}"

echo "clone URL resolves:"
ref="$(git ls-remote --heads https://github.com/ohho-robotics/ohho-drone.git refs/heads/main)"
[[ -n "$ref" ]] || fail "clone URL did not resolve refs/heads/main"
echo "$ref"

echo "credibility check: ok"
