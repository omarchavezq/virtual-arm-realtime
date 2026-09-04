#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Ejecute con sudo: sudo ./scripts/install.sh" >&2
  exit 1
fi

SOURCE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INSTALL_DIR=/opt/virtual-arm-realtime
CONFIG_DIR=/etc/virtual-arm-realtime

install -d -m 0755 "$INSTALL_DIR"
install -d -m 0750 -o virtual-rtk -g virtual-rtk "$CONFIG_DIR"
cp -a "$SOURCE_DIR/app" "$SOURCE_DIR/frontend" "$SOURCE_DIR/pyproject.toml" "$INSTALL_DIR/"
python3 -m venv "$INSTALL_DIR/.venv"
"$INSTALL_DIR/.venv/bin/pip" install --disable-pip-version-check "$INSTALL_DIR"

if [[ ! -f "$CONFIG_DIR/config.toml" ]]; then
  install -m 0600 -o virtual-rtk -g virtual-rtk \
    "$SOURCE_DIR/config.drill-001.toml" "$CONFIG_DIR/config.toml"
fi
chown virtual-rtk:virtual-rtk "$CONFIG_DIR/config.toml"
chmod 0600 "$CONFIG_DIR/config.toml"

install -m 0644 "$SOURCE_DIR/systemd/virtual-arm-realtime.service" \
  /etc/systemd/system/virtual-arm-realtime.service
systemctl daemon-reload

echo "Instalado sin iniciar. Valide /etc/virtual-arm-realtime/config.toml."
echo "Para cambiar de motor, primero detenga virtual-rtk; no comparten /dev/serial0."
