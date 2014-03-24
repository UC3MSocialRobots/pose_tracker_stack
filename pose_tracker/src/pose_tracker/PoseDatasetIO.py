#!/usr/bin/env python
# import roslib; roslib.load_manifest('pose_tracker')
# import rospy

# import numpy as np
import pandas as pd
import datetime
 
def parse_date(date):
    ''' An utility function to parse the date used in the dataset metadata.
        Atributes:
        @param date: The date in a form of a string
                Allowed formats: 
                    "YYYY-MM-DD", "YYYY-MM-DD hh:mm" and "YYYY-MM-DD hh:mm:ss" 
                See ISO 8601 For more information. 
        @return: the entered date in form of "YYYY-MM-DD hh:mm:ss"
        @raise ValueError: if entered date is malformed
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
            @param dataset: the name of the file where the dataset will be written
            @param dataset_columns: The name of the columns for the dataset_table
            
            @raise KeyError: if some argument is missing
            @raise TypeError: if dataset is not string or dataset_columns is
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
      
    def close(self):
        '''Closes the dataset file'''
        self.store.close()

    # def prepare_dataset(self, **kwargs):
    def fill_metadata(self, **kwargs):
        ''' Fills the metadata of the dataset.
            
            @keyword creator: The creator of the dataset
            @keyword date: Date in "YYYY-mm-ddTHH:MM" (ISO 8601 date format)'
            @keyword descr: string based description of the dataset content
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

    def get_metadata(self):
        ''' Returns the metadata table from the file'''
        return self.read_table('description')
    
    def write(self, table_name, chunk, **kwargs):
        ''' Writes a chunk of data to the dataset file
            Arguments:
            
            @param table_name: the name of the table on the file
            @param chunk: the pandas.DataFrame to be written to the file
            @keyword: other arguments to be pased to the method.
                     @see 'pandas.io.pytables.HDFStore.put().
                     Typically are bools "table" and "append"
            
            @raise TypeError: if table_name is not a string
            @raise TypeError: if chunk is not a pandas.DataFrame
            @raise ValueError: if chunk is empty
            
            '''

        # print table_name
        if not isinstance(table_name, str):
            raise TypeError("Table name must be a string")
        
        if not isinstance(chunk, pd.DataFrame):
            raise TypeError("chunk is not a pandas.DataFrame. Could not write")

        if chunk.empty:
            # rospy.logdebug("Nothing to write to the file")
            raise ValueError("data chunk is empty. Nothing to write")

        self.store.put(table_name, chunk, **kwargs)
        
        
    def read_table(self, table_name, **kwargs):
        ''' Reads the file and returns a dataframe stored in table table_name
        @type  table: string
        @param table: The table where to read
        @return: a pandas dataframe obtained from table table_name
        @rtype: pandas.DataFrame'''

        return pd.read_hdf(self.dataset + '.h5', table_name)
