import time
import os


class Logger:
    '''
    Analogue of ROS logger (same style) with minimum functions.
    '''

    def __init__(self, logging_path):
        '''
        Watch dir existence, and if not create file

        :param logging_path: path, where log will be storage
        '''
        if isinstance(logging_path, str):
            self.__logging_path = logging_path
            temp = logging_path.split("/")
            temp = temp[-1]
            if not os.path.exists(logging_path[:-(len(temp) + 1)]):
                os.mkdir(logging_path[:-(len(temp) + 1)])

    def logInfo(self, string):
        '''
        Write info in log

        :param string: message
        :type str
        '''
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[INFO][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def logWarn(self, string):
        '''
        Write warning in log

        :param string: message
        :type str
        '''
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[WARNING][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def logErr(self, string):
        '''
        Write error in log

        :param string: message
        :type str
        '''
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[ERROR][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def __del__(self):
        self.__writeLog.close()
