#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="python-pip
	  python-tables"

PIP_PACKAGES="numpy
	      pandas
              more-itertools 
              toolz
              mock"

sudo apt-get install $PACKAGES

pip install $PIP_PACKAGES
rm -rf build
