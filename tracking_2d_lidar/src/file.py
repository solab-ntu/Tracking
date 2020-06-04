import numpy as np
import csv
        
def write_csv(filename, the_lists):
    with open(filename, 'w') as csvfile:

        writer = csv.writer(csvfile)
        # write a row of data
        for the_list in the_lists:
            writer.writerow(list(the_list))

def read_csv(filename):
    with open(filename) as csvfile:

        rows = csv.reader(csvfile)

        return rows

        # for row in rows:
        #     for i in row:

def read_txt(filename):
    try:
        with open(filename) as f:
            numbers = map(float, f)
            number = numbers[0]
    except:
        print('[file.py] ','Required','\"',filename,'\"',' does not exist.')
    else:
        if number == float('nan'):
            number = np.nan 
        return number
