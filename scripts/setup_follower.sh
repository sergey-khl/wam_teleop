#!/bin/bash

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export BARRETT_CONFIG_FILE="$PACKAGE_DIR/config/follower-zeus-hand/follower.conf"
