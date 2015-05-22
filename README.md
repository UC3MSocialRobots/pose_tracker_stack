# pose_tracker_stack
Pose Tracker that I developed for my PhD Thesis

## Packages

This metapackage contains a set of packages that enable a robot to learn human poses using a Kinect RGB-D Sensor.

* `pose_tracker`: Nodes that perform the learning
* `pose_labeler`: Labeler for training
* `pose_detector`: Nodes to detect when a user us still or moving
* `pose_instance_builder`: Nodes and utilities to build a dataset from Kinect skeleton data
* `pose_msgs`: Common messages for all the packages in this metapackage

# Published papers:

Please, if you use this work, consider to cite paper:

* Gonzalez-Pacheco, V.; Malfaz, M.; Fernandez, F.; Salichs, M.A.	[Teaching Human Poses Interactively to a Social Robot](http://www.mdpi.com/1424-8220/13/9/12406). Sensors 2013, 13, 12406-12430. 


Bibtex: 

```latex
@Article{gonzalez-pacheco2013,
AUTHOR = {Gonzalez-Pacheco, Victor and Malfaz, Maria and Fernandez, Fernando and Salichs, Miguel A.},
TITLE = {Teaching Human Poses Interactively to a Social Robot},
JOURNAL = {Sensors},
VOLUME = {13},
YEAR = {2013},
NUMBER = {9},
PAGES = {12406--12430},
URL = {http://www.mdpi.com/1424-8220/13/9/12406},
PubMedID = {24048336},
ISSN = {1424-8220},
DOI = {10.3390/s130912406}
}
```
# Quality Metrics
[![Code Health](https://landscape.io/github/UC3MSocialRobots/pose_tracker_stack/master/landscape.svg?style=flat)](https://landscape.io/github/UC3MSocialRobots/pose_tracker_stack/master)
[![Scrutinizer Code Quality](https://scrutinizer-ci.com/g/UC3MSocialRobots/pose_tracker_stack/badges/quality-score.png?b=master)](https://scrutinizer-ci.com/g/UC3MSocialRobots/pose_tracker_stack/?branch=master)
[![Code Climate](https://codeclimate.com/github/UC3MSocialRobots/pose_tracker_stack/badges/gpa.svg)](https://codeclimate.com/github/UC3MSocialRobots/pose_tracker_stack)
[![Codacy Badge](https://www.codacy.com/project/badge/a044ba961d0044199b5435348e28436b)](https://www.codacy.com/app/vgonpa/pose_tracker_stack)
