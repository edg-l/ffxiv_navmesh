#!/usr/bin/env bash
# Deploy vnavmesh (fork) to the combined Zhyra plugin repo via the global publish-plugin script.
# Usage: ./publish.sh [repo_root]
set -euo pipefail
exec publish-plugin "${1:-$(dirname "$(readlink -f "$0")")}"