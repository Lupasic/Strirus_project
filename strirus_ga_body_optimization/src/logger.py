import time
import os


class Logger:
    def __init__(self, logging_path):
        if isinstance(logging_path, str):
            self.__logging_path = logging_path
            temp = logging_path.split("/")
            temp = temp[-1]
            if not os.path.exists(logging_path[:-(len(temp) + 1)]):
                os.mkdir(logging_path[:-(len(temp) + 1)])


    def logInfo(self, string):
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[INFO][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def logWarn(self, string):
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[WARNING][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def logErr(self, string):
        self.__writeLog = open(self.__logging_path, 'a')
        self.__writeLog.write("[ERROR][" + time.ctime() + "] " + string + "\n")
        self.__writeLog.close()

    def __del__(self):
        self.__writeLog.close()