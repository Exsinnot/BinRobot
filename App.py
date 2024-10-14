from flask import Flask, request, jsonify
from flask_cors import CORS
import glob
import os

app = Flask(__name__)
CORS(app)

@app.route('/api/getdata', methods=['GET'])
def get_data():
    data = {}
    page = request.args.get('page')
    print(page)
    if page == "Dashboard":
        with open("data.csv", mode='r', encoding='utf-8') as file:
            raw_data = file.readlines()
            for x in raw_data:
                strip_data = x.strip()
                split_data = strip_data.split(',')
                data["Batterylevel"] = split_data[0]
                data["Batterydestroy"]= split_data[1]
                data["Trashlevel"]= int(100-(int(split_data[2])*100/25))
                data["Status"]= split_data[4]
                data["Wifi"]= split_data[3]
        print(data)
    elif page == "Setting":
        with open("Setting.csv", mode='r', encoding='utf-8') as file:
            raw_data = file.readlines()
            for x in raw_data:
                split_data = x.split(',')
                data["name"] = split_data[0]
                data["gohome"] = int(split_data[1])
                data["stopdistance"] = int(split_data[2])
        print(data)
    else:
        return jsonify({"message": "No valid page specified"}), 400
    # data = {"message": "Hello from Flask!"}
    return jsonify(data)

@app.route('/api/getGraph', methods=['GET'])
def get_date():
    date = request.args.get('dateSelect')
    data = {
        "time":[],
        "battery":[],
        "powerUsage":[]
    }
    if date == "":
        folder_path = 'datetime/'
        datetime = []
        hourly_data = {}
        file_names = glob.glob(os.path.join(folder_path, "*.csv")) # ดึงเฉพาะไฟล์ที่มีนามสกุล .csv
        last_file_path = os.path.basename(file_names[-1])
        with open(folder_path+last_file_path, mode='r', newline='') as file:
            reader = file.readlines()
            for x in reader:
                split_data = x.split(',')
                hour = split_data[0][:2]
                battery = (int(split_data[1]))
                powerUsage = (int(split_data[2]))
                if hour not in hourly_data:
                    hourly_data[hour] = {"sum_battery":0,"sum_powerUsage":0,"count":0}
                hourly_data[hour]['sum_battery'] += battery
                hourly_data[hour]['sum_powerUsage'] += powerUsage
                hourly_data[hour]['count'] += 1
                
        avg_data = {}
        
        for hour, values in hourly_data.items():
            avg_battery = values['sum_battery'] / values['count']
            avg_powerUsage = values['sum_powerUsage'] / values['count']
            avg_data[hour] = {'avg_battery': avg_battery, 'avg_powerUsage': avg_powerUsage}
        for hour, avg in avg_data.items():
            data["time"].append(f"{hour}:00")
            data["battery"].append(int(avg["avg_battery"]))
            data["powerUsage"].append(int(avg["avg_powerUsage"]))
        for f in file_names:
            year, month, day = os.path.basename(f).replace(".csv","").split("-")
            datetime.append(f"{day}/{month}/{year}")
        data["datetime"] = datetime 
        return jsonify(data)
    else:
        day,month,year = date.split("/")
        file_name = f"{year}-{month}-{day}.csv"
        folder_path = 'datetime/'
        datetime = []
        hourly_data = {}
        with open(folder_path+file_name, mode='r', newline='') as file:
            reader = file.readlines()
            for x in reader:
                split_data = x.split(',')
                hour = split_data[0][:2]
                battery = (int(split_data[1]))
                powerUsage = (int(split_data[2]))
                if hour not in hourly_data:
                    hourly_data[hour] = {"sum_battery":0,"sum_powerUsage":0,"count":0}
                hourly_data[hour]['sum_battery'] += battery
                hourly_data[hour]['sum_powerUsage'] += powerUsage
                hourly_data[hour]['count'] += 1
                
        avg_data = {}
        
        for hour, values in hourly_data.items():
            avg_battery = values['sum_battery'] / values['count']
            avg_powerUsage = values['sum_powerUsage'] / values['count']
            avg_data[hour] = {'avg_battery': avg_battery, 'avg_powerUsage': avg_powerUsage}
        for hour, avg in avg_data.items():
            data["time"].append(f"{hour}:00")
            data["battery"].append(int(avg["avg_battery"]))
            data["powerUsage"].append(int(avg["avg_powerUsage"]))
        return jsonify(data)
    # return jsonify({"message": "No valid page specified"}), 404

@app.route('/api/update', methods=['POST'])
def get_update():
    data = request.json  
    with open("Setting.csv", mode='w',encoding='utf-8') as file:
        file.write(data["name"]+","+str(data["gohome"])+","+str(data["stopdistance"]))
    print("update info succesfully.")
    return jsonify({'message': 'CSV updated successfully!'})
    
    
if __name__ == '__main__':
    app.run(debug=True,host="0.0.0.0",port=5000)
