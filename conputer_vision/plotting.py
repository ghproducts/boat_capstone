import numpy as np
import matplotlib.pyplot as plt
import csv

with open('data.csv', newline=',') as csvfile:
    data = list(csv.reader(csvfile))
