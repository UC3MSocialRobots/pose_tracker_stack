#!/usr/bin/env python
import pandas as pd
import 

DEFAULT_NAME = 'pose_learner'

class PoseLearner():
    ''' Class produces a learned model from a entered DataFrame

        @type df: pandas.DataFrame
        @keyword df: 
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', DEFAULT_NAME)
