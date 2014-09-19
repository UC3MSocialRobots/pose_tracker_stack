#!/usr/bin/env python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05

Functions to give circular behaviour to pandas.Dataframe 
I.e. A circular dataframe pops first row when new rows are added 
'''
# import roslib; roslib.load_manifest('pose_instance_builder')

import pandas as pd

def _drop_older_rows(df, max_dflen):
    ''' Drops n elements of the header where n = len(df) - max_dflen'''
    if max_dflen <= 0:
        return df
    df_ = df
    drop_counts = len(df_) - max_dflen
    if drop_counts > 0:
        df_ = df.drop(df.head(drop_counts).index)   # Drop drop_counts elems
    return df_


def append_instance(df, ins, max_dflen=50):
    ''' Appends an an instance to the dataset 'df'. 

        @param df: dataset at which the instance is added.
        @type df: pandas.DataFrame
        @param ins: Instance to be added to DataFrame
        @type ins: numpy.ndarray (1D)
        @param max_dflen: (Default 50) Max lenght of the dataframe
                    If len(df) > 1 then drops first elem of df.
        @return: a dataframe with the instance added to the end
        @rtype: pandas.DataFrame
    '''
    if ins.ndim != 1:
        raise ValueError("'ins' is not 1D. Shape: {}".format(ins.shape))
    s = pd.Series(ins)
    df_ = _drop_older_rows(df, max_dflen - 1)
    return df_.append(s, ignore_index=True)
