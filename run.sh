#!/bin/bash
# lidar2026 — start server
set -e
cd "$(dirname "$0")"

[ -d .venv ] || python3 -m venv .venv
.venv/bin/pip install -q --upgrade pip >/dev/null
.venv/bin/python -c "import ouster.sdk" 2>/dev/null || .venv/bin/pip install -q ouster-sdk scapy

cd server
[ -d node_modules ] || npm install
echo "→ http://localhost:3000"
exec node server.js "$@"
