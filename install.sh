#!/usr/bin/env bash

PROGNAME=$(basename $0)

usage() {
  echo "Usage: $PROGNAME [option]"
  echo
  echo "This script installs kivy and imgaug for rosbag_annotation on Ubuntu."
  echo "options:"
  echo "  -h, --help : show this help"
  echo
  exit 1
}

for OPT in "$@"
do
  case "$OPT" in
    '-h'|'--help' )
      usage
      exit 1
      ;;
    '--'|'-' )
      shift 1
      param+=( "$@" )
      break
      ;;
    -*)
      echo -e "\e[31m$PROGNAME: unknown option $(echo $1 | sed 's/^-*//'). Check '$PROGNAME -h'\e[m" 1>&2
      exit 1
      ;;
  esac
done

# preparation
echo -e "\e[1;32mPreparation\e[m"

sudo apt-get install python-setuptools python-pygame python-opengl python-gst0.10 python-enchant gstreamer0.10-plugins-good libgl1-mesa-dev libgles2-mesa-dev zlib1g-dev
sudo pip install cython==0.25.2 # the newest supported version by kivy
sudo pip install pygame
sudo pip install kivy
sudo apt-get install xsel xclip
sudo pip install imgaug
