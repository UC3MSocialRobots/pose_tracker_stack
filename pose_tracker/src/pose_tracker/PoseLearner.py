#!/usr/bin/env python

DEFAULT_NAME = 'pose_learner'


def create_model(df, predictor):
    '''
        @type df: pandas.DataFrame
        @param df the dataframe from which the model will be created
        @param predictor: the algorithm that will be used to learn
    '''
    pass


class PoseLearner():
    ''' Class produces a learned model from a entered DataFrame

        @type df: pandas.DataFrame
        @keyword df: 
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', DEFAULT_NAME)

