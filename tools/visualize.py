#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
python2 -m pip install matplotlib==2.2.3
python2 -m pip install pandas

'''
import os
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

path = '/home/airobot/JD_Autopilot/result/data/'
def get_data(csv_file):
    lat_data = pd.read_csv(path + csv_file, usecols=[1], header=None)
    lon_data = pd.read_csv(path + csv_file, usecols=[2], header=None)
    latitude =  np.array(lat_data)
    longitude = np.array(lon_data)
    return latitude, longitude
'''
The script can visual 4 files at most,if less than 4 file,just set it to "".
For example, if you have two file,you need to excute command in terminal:
python2 visualize.py --csv1=file1.csv --csv2=file2.csv --csv3="" --csv4=""
'''

if __name__ == '__main__':
    parse = argparse.ArgumentParser(description='')
    parse.add_argument('--csv1', help='csv file1', nargs='?')
    parse.add_argument('--csv2', help='csv file2', nargs='?')
    parse.add_argument('--csv3', help='csv file3', nargs='?')
    parse.add_argument('--csv4', help='csv file4', nargs='?')
    args = parse.parse_args()
    file1 = args.csv1
    file2 = args.csv2
    file3 = args.csv3
    file4 = args.csv4
    if file2=="" and file3=="" and file4=="":
        lat1, lon1 = get_data(file1)
        plt.plot(lat1, lon1, 'b',linewidth=0.8, linestyle="-", label=file1)
        plt.legend(loc='upper right')
        plt.show()
    if file3=="" and file4=="":
        lat1, lon1 = get_data(file1)
        lat2, lon2 = get_data(file2)
        plt.plot(lat1, lon1, 'b',linewidth=0.8, linestyle="-", label=file1)
        plt.plot(lat2, lon2, 'g',linewidth=0.8, linestyle="-", label=file2)
        plt.legend(loc='upper right')
        plt.show()
    if file4=="":
        lat1, lon1 = get_data(file1)
        lat2, lon2 = get_data(file2)
        lat3, lon3 = get_data(file3)
        plt.plot(lat1, lon1, 'b',linewidth=0.8, linestyle="-", label=file1)
        plt.plot(lat2, lon2, 'g',linewidth=0.8, linestyle="-", label=file2)
        plt.plot(lat3, lon3, 'r',linewidth=0.8, linestyle="-", label=file3)
        plt.legend(loc='upper right')
        plt.show()
    else:
        lat1, lon1 = get_data(file1)
        lat2, lon2 = get_data(file2)
        lat3, lon3 = get_data(file3)
        lat4, lon4 = get_data(file4)
        plt.plot(lat1, lon1, 'b',linewidth=0.8, linestyle="-", label=file1)
        plt.plot(lat2, lon2, 'g',linewidth=0.8, linestyle="-", label=file2)
        plt.plot(lat3, lon3, 'r',linewidth=0.8, linestyle="-", label=file3)
        plt.plot(lat4, lon4, 'y',linewidth=0.8, linestyle="-", label=file4)
        plt.legend(loc='upper right')
        plt.show()
