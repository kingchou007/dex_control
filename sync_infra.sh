#!/bin/bash

LOCAL_DIR="/home/jinzhou/dex-control"
NUC_HOST="labuser@192.168.1.7"
NUC_DIR="/home/labuser"

rsync -avz \
  --exclude=".git" \
  --exclude="data" \
  --exclude="__pycache__" \
  --exclude=".venv" \
  "$LOCAL_DIR" "$NUC_HOST:$NUC_DIR"

echo "Done!"
