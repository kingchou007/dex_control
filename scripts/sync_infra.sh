#!/bin/bash

LOCAL_DIR="xxxx"
NUC_HOST="xxxx"
NUC_DIR="xxxx"

# Only sync core files: dex_control module, config, scripts
rsync -avz \
  --include="dex_control/" \
  --include="dex_control/robot/***" \
  --include="dex_control/__init__.py" \
  --include="config/***" \
  --include="setup.py" \
  --include="pyproject.toml" \
  --exclude="*" \
  --exclude="__pycache__" \
  "$LOCAL_DIR/" "$NUC_HOST:$NUC_DIR/"

echo "Done!"
