#!/usr/bin/env python
# import roslib; roslib.load_manifest('pose_tracker')
# import rospy
# import kinect.msg as kin
# import numpy as np
import pandas as pd
import datetime
 
def parse_date(date):
    ''' An utility function to parse the date used in the dataset metadata.
        Atributes:
        :date: The date in a form of a string
                Allowed formats: 
                    "YYYY-MM-DD", "YYYY-MM-DD hh:mm" and "YYYY-MM-DD hh:mm:ss" 
                See ISO 8601 For more information. 
        Return: the entered date in form of "YYYY-MM-DD hh:mm:ss"
        Raises: ValueError if entered date is malformed
        '''
    supported_formats = ('%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M', '%Y-%m-%d')
    for fmt in supported_formats:
        try:
            parsed_date = datetime.datetime.strptime(str(date), fmt)
            return parsed_date.strftime(fmt)
        except ValueError:
            pass # Keep trying formats 

    # If we arrive here it means that an unsupported format was entered
    raise ValueError("Incorrect date: {}. " 
                        "Allowed formats: "
                        "'YYYY-MM-DD', 'YYYY-MM-DD hh:mm' "
                        "and 'YYYY-MM-DD hh:mm:ss'."
                        " See ISO 8601 For more information.".format(date))


class PoseDatasetIO(object):
    """Class that that reads/writes data to a dataset containing poses"""
    def __init__(self, *args, **kwargs):
        ''' Inits the class.
            Arguments:
            :dataset: the name of the file where the dataset will be written
            :dataset_columns: The name of the columns for the dataset_table
            
            Raises KeyError if some argument is missing
            Raises TypeError if dataset is not string or dataset_columns is
            not an iterable.'''

        self.dataset = kwargs['dataset']
        self.dataset_columns = kwargs['columns']
        # print self.dataset
        # print str(self.dataset_columns)
        
        if not isinstance(self.dataset, str):
            raise TypeError("dataset must be a string")
        if not hasattr(self.dataset_columns,'__iter__'):
            raise TypeError("dataset columns must be iterable!")

        # self.create_dataset(self.dataset)

    def create_dataset(self):
        '''Creates de dataset file in HDF5 Format'''
        # print "CWD: " + os.getcwd()
        # print self.dataset
        self.store = pd.HDFStore(self.dataset + '.h5')
      

    # def prepare_dataset(self, **kwargs):
    def fill_metadata(self, **kwargs):
        ''' Fills the metadata of the dataset.
            Can be entered the following params:
            'creator': The creator of the dataset
            'date': Date in "YYYY-mm-ddTHH:MM" (ISO 8601 date format)'
            'descr': a string based description of what the dataset contains
       '''
        creator = kwargs.get('creator', 'Anonymous')
        date = kwargs.get('date', '1900-01-01 00:00')
        user_descr = kwargs.get('descr', 
                                 "Default description. " \
                                 "User forgot to add it when created " \
                                 "the dataset.")
        
        # A dirty hack to use the pandas interface to pytables
        description = pd.Series((creator, date, user_descr), 
                                index=('creator', 'date','descr'))

        self.store.put('description', description)    
    
    def write(self, table_name, chunk, **kwargs):
        ''' Writes a chunk of data to the dataset file
            Arguments:
            :chunk: the pandas.DataFrame to be written to the file
            :table_name: the name of the table on the file
            :kwargs: other arguments to be pased to the method.
                     see 'pandas.io.pytables.HDFStore.put().
                     Typically are bools "table" and "append"
            Returns True if succeeds writing data. False otherwise

            Raises TypeError if table_name is not a string
            Raises ValueError if chunk is empty
            Raises TypeError fi chunk is not a pandas.DataFrame
            '''

        # print table_name
        if not isinstance(table_name, str):
            raise TypeError("Table name must be a string")
        
        if not chunk:
            # rospy.logdebug("Nothing to write to the file")
            raise ValueError("data chunk is empty. Nothing to write")
        
        if not isinstance(chunk, pd.DataFrame):
            raise TypeError("chunk is not a pandas.DataFrame. Could not write")

        self.store.put(table_name, chunk, **kwargs)
        
        
    def read(self, **kwargs):
        raise NotImplementedError
        pass
