#!/usr/bin/env python

from read_from_txt import readObjectivesFromTextFile, extractSingleObjective


if __name__=='__main__':

    myfile="/catkin_ws/src/navigationturt/scripts/objective.txt"
    index=1
    mydata=(extractSingleObjective(readObjectivesFromTextFile(myfile),index))

    for c in range(len(mydata)):
        print(mydata[c])