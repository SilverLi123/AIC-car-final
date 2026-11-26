#!/bin/bash
TOP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$TOP/ws_rdkx5_fork/start.sh"
