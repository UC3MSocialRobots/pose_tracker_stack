#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="python-pip
	  python-tables"

PIP_PACKAGES="numpy
	      pandas
              scikit-learn
              toolz
              mock"

sudo apt-get install $PACKAGES

sudo pip install $PIP_PACKAGES
rm -rf build
