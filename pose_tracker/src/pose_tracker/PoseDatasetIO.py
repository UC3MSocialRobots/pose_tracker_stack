#!/usr/bin/env python
# import roslib; roslib.load_manifest('pose_tracker')
# import rospy

# import numpy as np
import pandas as pd
import datetime


def parse_date(date):
    """
    An utility function to parse the date used in the dataset metadata.

    Params
    ------
    date: str
        The date in a form of a string. Must follow ISO8601:
            1. I{YYYY-MM-DD}
            2. I{YYYY-MM-DD hh:mm}
            3. I{YYYY-MM-DD hh:mm:ss}
        :see: ISO 8601 for more information.

    Returns
    -------
    str
        The entered date in form of "YYYY-MM-DD hh:mm:ss"

    Raises
    ------
    ValueError
        If entered date is malformed
    """
    supported_formats = ('%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M', '%Y-%m-%d')
    for fmt in supported_formats:
        try:
            parsed_date = datetime.datetime.strptime(str(date), fmt)
            return parsed_date.strftime(fmt)
        except ValueError:
            pass  # Keep trying formats

    # If we arrive here it means that an unsupported format was entered
    raise ValueError("Incorrect date: {}. "
                     "Allowed formats: "
                     "'YYYY-MM-DD', 'YYYY-MM-DD hh:mm' "
                     "and 'YYYY-MM-DD hh:mm:ss'."
                     " See ISO 8601 For more information.".format(date))


def filename_with_extension(name, extension):
    """
    Ensure filename ends with extension.

    Returns
    -------
    str
        name if name already ends with extension.
        name + etension if name does not end with extension.

    Examples
    --------
    >>> filename1 =  'my_file'
    >>> filename_with_extension(filename1, '.py')
    'my_file.py'
    >>> filename2 =  'my_second_file.py'
    >>> filename_with_extension(filename2, '.py')
    'my_second_file.py'
    """
    return name if name.endswith(extension) else name + extension


class PoseDatasetIO(object):

    """Class that that reads/writes data to a dataset containing poses"""

    def __init__(self, *args, **kwargs):
        """
        Init the class.


        Parameters
        ----------
        dataset : str
            Name of the file where the dataset will be written
        columns : str
            name of the columns for the dataset_table
        mode : {'a', 'w', 'r', 'r+'}
            Default 'a'
            - ``'a'`` Append; an existing file is opened for reading
                      and writing. If the file does not exist it is created.
            - ``'w'`` Write; a new file is created
                      (an existing file with the same name would be deleted)
            - ``'r'`` Read-only; no data can be modified.
            - ``'r+'`` Similar to ``'a'``, but the file must already exist.

        Raises
        ------
        KeyError
            If some argument is missing
        TypeError
            If dataset is not string or columns is not an iterable.
        """

        # self.dataset = kwargs['dataset'] + '.h5'
        self.dataset = filename_with_extension(kwargs['dataset'], '.h5')
        self.dataset_columns = kwargs['columns']
        self._mode = kwargs.get('mode', 'a')

        if not isinstance(self.dataset, str):
            raise TypeError("dataset must be a string")
        if not hasattr(self.dataset_columns, '__iter__'):
            raise TypeError("dataset columns must be iterable!")

    def __enter__(self):
        """:todo: allow open dataset passing it arguments."""
        self.open_dataset(mode=self._mode)
        return self
        # self.create_dataset(self.dataset)

    def __exit__(self, type, value, traceback):
        self.close()
        return

    def create_dataset(self):
        """Create de dataset file in HDF5 Format."""
        self.open_dataset()

    def open_dataset(self, **kwargs):
        """
        Open the dataset file using the pandas.HDFStore API.

        :see: pandas.HDFStore

        Notes
        -----
            All keywords are passed to the pandas.HDFStore API

        Parameters
        ----------
        path : str (Optional)
            File path to HDF5 file
        mode: {'a', 'w', 'r', 'r+'}
            Default 'a'
            - ``'a'`` Append; an existing file is opened for reading
                      and writing. If the file does not exist it is created.
            - ``'w'`` Write; a new file is created
                      (an existing file with the same name would be deleted)
            - ``'r'`` Read-only; no data can be modified.
            - ``'r+'`` Similar to ``'a'``, but the file must already exist.
        """
        self.store = pd.HDFStore(self.dataset, **kwargs)

    def close(self):
        """Close the dataset file."""
        self.store.close()

    # def prepare_dataset(self, **kwargs):
    def fill_metadata(self, **kwargs):
        """
        Fills the metadata of the dataset.

        Parameters
        ----------
        creator : str (Optional)
            The creator of the dataset
        date : str (Optional)
            Date in "YYYY-mm-ddTHH:MM" (ISO 8601 date format)
        descr : str (Optional)
            string based description of the dataset content
       """
        creator = kwargs.get('creator', 'Anonymous')
        date = kwargs.get('date', '1900-01-01 00:00')
        user_descr = kwargs.get('descr',
                                "Default description. "
                                "User forgot to add it when created "
                                "the dataset.")

        # A dirty hack to use the pandas interface to pytables
        description = pd.Series((creator, date, user_descr),
                                index=('creator', 'date', 'descr'))
        self.store.put('description', description)

    def get_metadata(self):
        """
        Return the metadata table from the file.

        :see: py:meth:``fill_metadata`

        Returns
        -------
        pandas.Series
            The metadata contained in the dataset file
        """
        return self.read_table('description')

    def write(self, table_name, chunk, **kwargs):
        """
        Write a chunk of data to the dataset file

        Parameters
        ----------
        table_name : str
            The name of the table on the file
        chunk : pandas.DataFrame
            The pandas.DataFrame to be written to the file
        kwargs:
            Other arguments to be pased to the method.
            Typically are bools "table" and "append"
            :see: 'pandas.io.pytables.HDFStore.put().

        Raises
        ------
        TypeError
            If chunk is not a pandas.DataFrame
        ValueError
            If chunk is empty
        """
        if not isinstance(chunk, pd.DataFrame):
            raise TypeError("chunk is not a pandas.DataFrame. Could not write")

        if chunk.empty:
            # rospy.logdebug("Nothing to write to the file")
            raise ValueError("data chunk is empty. Nothing to write")

        self.store.put(table_name, chunk, **kwargs)

    def read_table(self, table_name):
        """
        Read the file and returns a dataframe stored in table ``table_name``.

        Parameters
        ----------
        table_name : str
            The table where to read

        Returns
        -------
        pandas.DataFrame
            A pandas dataframe obtained from table table_name
        """
        return self.store.get(table_name)

    def read_group(self, group_name):
        """
        Read the file and return a dict with all tables belonging to a group.

        Parameters
        ----------
        group_name : str
            The group to read

        Returns
        -------
        dict
            A dict whose keys refer to pandas.DataFrames
            obtained from ``table_name``
        """
        # return pd.concat([self.store.select(node._v_pathname)
        #                  for node in self.store.get_node(group_name)])
        return {node._v_name: self.store.select(node._v_pathname)
                for node in self.store.get_node(group_name)}
