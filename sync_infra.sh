#!/bin/bash

LOCAL_DIR="/home/jinzhou/projects/temp"
NUC_HOST="labuser@192.168.1.7"                
NUC_DIR="/home/labuser/temp/temp"

rsync -avz \
  --exclude=".git" \
  --exclude="data" \
  --exclude="__pycache__" \
  --exclude=".venv" \
  --exclude="dependencies" \
  "$LOCAL_DIR" "$NUC_HOST:$NUC_DIR"

echo "Done!"
