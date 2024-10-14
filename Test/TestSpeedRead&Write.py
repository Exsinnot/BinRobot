import csv
from datetime import datetime, timedelta
import time
# จำนวนแถวที่ต้องการสร้าง
# num_rows = 1440

# # กำหนดเวลาเริ่มต้น
# start_time = datetime(2024, 10, 8, 0, 0, 0)
# starttime = time.time()
# # สร้างไฟล์ CSV
# with open('output.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
    
#     # เขียนหัวข้อของ CSV (ตัวเลือกนี้ไม่จำเป็นถ้าไม่ต้องการหัวข้อ)
#     writer.writerow(["Timestamp", "Value1", "Value2"])
    
#     for i in range(num_rows):
#         # คำนวณ timestamp โดยเพิ่ม 1 นาทีสำหรับแต่ละแถว
#         timestamp = start_time + timedelta(minutes=i)
#         # กำหนดค่า Value1 และ Value2 แบบสุ่มหรือแบบคงที่ตามที่ต้องการ
#         value1 = (i % 20) + 10  # ตัวอย่างการกำหนดค่าแบบคงที่ (ปรับได้ตามต้องการ)
#         value2 = 80 - (i % 10) * 5
        
#         # เขียนข้อมูลลง CSV
#         writer.writerow([timestamp.strftime("%d-%m-%y %H:%M:%S"), value1, value2])
# print(time.time() - starttime)
# starttime = time.time()
# with open("output.csv",'r') as file:
#     data = file.readlines()
# print(time.time() - starttime)

# print("CSV file created successfully with 84,000 rows!")
import glob
timestart = time.time()
timesave = time.time()
while True:
    # if time.time()-timestart >=1:
    #     timestart = time.time()
    #     print(1)
    if time.time()-timesave >= 5:
        timesave = time.time()
        current_date = datetime.now().strftime("%Y-%m-%d")
        folder_path = '/home/user/BinRobot/data/*.csv'  # เลือกเฉพาะไฟล์ .csv
        file_names = glob.glob(folder_path)
        print(file_names)
        if f"/home/user/BinRobot/data/{current_date}.csv" in file_names:
            print(current_date)