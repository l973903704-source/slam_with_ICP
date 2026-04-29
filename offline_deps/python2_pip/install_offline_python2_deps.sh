#!/bin/bash
set -e

cd "$(dirname "$0")"

if ! command -v python2 >/dev/null 2>&1; then
  echo "ERROR: python2 not found. Please install Python 2.7 first." >&2
  exit 1
fi

python2 -m pip --version >/dev/null 2>&1 || {
  echo "ERROR: python2 pip not found. Install python-pip first, then rerun this script." >&2
  exit 1
}

python2 -m pip install --no-index --find-links packages -r requirements_python2.txt
echo "Python2 offline dependencies installed."
