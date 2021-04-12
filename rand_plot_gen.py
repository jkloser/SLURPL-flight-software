# import numpy as np
# import matplotlib.pyplot as plt
import random
import time
import csv

x_val = 0
y1_val = 1000
y2_val = 1000

headers = ['x_val', 'y1_val', 'y2_val']

with open('data.csv', 'w') as csvfile:
	csv_writer = csv.DictWriter(csvfile, fieldnames=headers)
	csv_writer.writeheader()

while True:
	with open('data.csv', 'a') as csvfile:
		csv_writer = csv.DictWriter(csvfile, fieldnames=headers)

		data = {
			'x_val': x_val,
			'y1_val': y1_val,
			'y2_val': y2_val
		}

		csv_writer.writerow(data)

		x_val+=1
		y1_val = y1_val + random.randint(-6,8)
		y2_val = y2_val + random.randint(-5,6)

	time.sleep(2)

	# live plot check x value set x limit to be fixed ifnore previous 