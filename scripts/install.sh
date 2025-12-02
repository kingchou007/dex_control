#!/bin/bash
set -e
conda activate dex-control
VERSION=0-18-0
wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_${VERSION}_wheels.zip
unzip libfranka_${VERSION}_wheels.zip
pip install numpy
pip install --no-index --find-links=./dist franky-control