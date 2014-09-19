#! /usr/bin/env python
''' Locates and installs all rosdep.sh files in a directory tree.
    
    locate function adapted from: 
    http://code.activestate.com/recipes/499305-locating-files-throughout-a-directory-tree/'''
import os
import fnmatch
from colorterm import colorterm
from subprocess import call


def locate(pattern, root=os.curdir):
    ''' Locate all files matching supplied filename pattern in and below
        supplied root directory.'''
    for path, dirs, files in os.walk(os.path.abspath(root)):
        for filename in fnmatch.filter(files, pattern):
            yield (path, filename)


def locate_fp(pattern, root=os.curdir):
    ''' same as locate but returning file with full path '''
    for path, filename in locate(pattern, root):
        yield os.path.join(paht, filename)


if __name__ == '__main__':
    for path, filename in locate('rosdep.sh'):
        msg = "\n\nFound '{}'' file in dir '{}'".format(filename, path)
        print colorterm.blue(msg)
        os.chdir(path)
        call(['sh', filename])

    