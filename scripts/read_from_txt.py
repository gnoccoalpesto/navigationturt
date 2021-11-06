#!/usr/bin/env python

'''
input:  path to a multiline, comma or space divided values, text file
        noNames if file has no value attribute names in each row
output: list[row][colums] of the values in the text file
'''

def readObjectivesFromTextFile(textfile_address,extraction_mode='noNames'):

    textFile=open(textfile_address)
    objectives = []
	
    for line in textFile.readlines():
        tokens = line.replace(',',' ').split(' ',-1)
        if extraction_mode=='noNames':
            objectives.append((float(tokens[0]), float(tokens[1]),  float(tokens[2])))
        else:
            objectives.append((float(tokens[1]), float(tokens[3]),  float(tokens[5])))

    return objectives

def extractSingleObjective(data,position):
    if position>len(data):
        raise Exception('exceedes list lenght by {}'.format(position-len(data)))
        exit(1)
    else:
        return data[position-1]