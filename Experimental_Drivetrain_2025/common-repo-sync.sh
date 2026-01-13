#!/bin/sh

git config --global --add safe.directory /home/jeff/work/robot-base/src/main/java/frc/recoil
if [ -d  "src/main/java/frc/recoil" ]; then
  (cd src/main/java/frc/recoil && git pull)
else
  git clone --branch main --depth 1 https://github.com/frc9577/common src/main/java/frc/recoil
fi
