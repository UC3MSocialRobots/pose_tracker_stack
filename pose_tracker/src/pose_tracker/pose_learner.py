#!/usr/bin/env python

from __future__ import (print_function, division)

from functools import partial
import numpy as np
import pandas as pd

# import PoseDatasetIO as pdio
from PoseDatasetIO import PoseDatasetIO
import user_data_loader as udl


DEFAULT_NAME = 'pose_learner'
COLS_TO_CLEAN=udl.confidences   

def _clean_prefix(text, prefix): 
    return text.lstrip(prefix)

_rm_stand_pref = partial(_clean_prefix, prefix='STAND_')


def prepare_dataset(filename, group_name):
    ''' Loads the file filename and returns all the tables contained in the 
        group 'group_name' in form of a unified dataset
        prior to returning it, the dataset is grouped by pose, to 
    '''
    with PoseDatasetIO(dataset=filename, columns=udl.index, mode='r') as dataset:
        dataset = {node._v_name: dataset.store.select(node._v_pathname). \
                                              groupby('pose').mean(). \
                                              rename(_rm_stand_pref)
                  for node in dataset.store.get_node(group_name) }
        return pd.concat(dataset)


def drop_columns(dataset, cols=COLS_TO_CLEAN):
    ''' Drops the entered dataset columns. 
        @type dataset: pandas.DataFrame
        @param dataset: the dataset which columns are going to be dropped 
        @param cols: the list of column names to be dropped.
                     Default: L{COLS_TO_CLEAN}
        @return: A copy of the dataset without the columns
    '''
    return dataset.drop(cols, axis=1)


def numerize_y(y):
    ''' Converts vector y to nums'''
    labels = sorted(set(y))
    return np.array(map(labels.index, y))

def df_to_Xy(dataframe):
    ''' Converts a dataframe to scikitlearn's compatible X and y
        @param dataframe: DataFrame to be converted to scikit-learn X,y format
        @type dataframe: pandas.DataFrame
        @return: a tuple (X, y)
    '''
    y = zip(*dataframe.index)[1]
    y_num = numerize_y(y)
    return (dataframe.values, y_num)


def train_clf(X, y, **kwargs):
    '''
        Trains a classifier with the entered data
        @param X: 
        @type X:
        @param y: 
        @type y:
        @keyword model:
        @type model:
        @keyword param_grid: Dictionary of parameters to 
        @type param_grid: dict

        @return: the classifier already fitted to the entered data
    '''
    model = kwargs.get('predictor', 
                        ensemble.RandomForestClassifier(oob_score=True))
    param_grid = kwargs['param_grid']

    classif = GridSearchCV(model, param_grid, cv=3, score_func=sklm.f1_score)
    classif.fit(X, y)
    return classif
