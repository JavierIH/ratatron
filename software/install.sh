#!/bin/bash
# Installs the `ratatron` command: a virtual environment in software/.venv with the tools (editable: they run
# from this repository), linked into ~/.local/bin.
set -e
cd "$(dirname "$0")"
python3 -m venv .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -e .
mkdir -p ~/.local/bin
ln -sf "$PWD/.venv/bin/ratatron" ~/.local/bin/ratatron
echo "ratatron installed: $(command -v ratatron || echo "$HOME/.local/bin/ratatron (add ~/.local/bin to PATH)")"
echo "try: ratatron help"
