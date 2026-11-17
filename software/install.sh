#!/bin/bash
# Installs the `ratatron` command: a virtual environment in software/.venv with the tools (editable: they run
# from this repository), linked into ~/.local/bin. `./install.sh --no-sim` leaves the simulator's packages out.
set -e
cd "$(dirname "$0")"
EXTRAS="[sim]"
[ "$1" = "--no-sim" ] && EXTRAS=""
python3 -m venv .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -e ".$EXTRAS"
mkdir -p ~/.local/bin
ln -sf "$PWD/.venv/bin/ratatron" ~/.local/bin/ratatron
echo "ratatron installed: $(command -v ratatron || echo "$HOME/.local/bin/ratatron (add ~/.local/bin to PATH)")"
echo "try: ratatron help"
