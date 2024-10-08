import csv
import random
import time
from datetime import datetime

# Number of rows to generate
num_rows = 10

# File name for the CSV
csv_filename = "data.csv"

# Create the CSV file and write headers
# with open(csv_filename, mode='r', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(["time", "w", "Battery(%)"])
    
#     # Generate rows of data
#     for _ in range(num_rows):
#         current_time = int(time.time())   # Current time in HH:MM:SS format
#         w_value = random.uniform(0, 20)  # Random value between 0 and 20
#         battery_percentage = random.randint(0, 100)  # Random percentage between 0 and 100
        
#         writer.writerow([current_time, round(w_value, 2), battery_percentage])
#         time.sleep(1)  # Optional: Sleep for 1 second to simulate time intervals
data = {}
    
with open("data.csv", mode='r') as file:
        raw_data = file.readlines()
        print(raw_data)
        for x in raw_data:
            strip_data = x.strip()
            split_data = strip_data.split(',')
            print(split_data)
            data["Batterylevel"] = int(split_data[0][3] + split_data[0][4])
            data["Batterydestroy"]= split_data[1]
            data["Trashlevel"]= int(int(split_data[2])*100/25)
            data["Status"]= split_data[3]
            data["Wifi"]= split_data[4]
            
print(data)