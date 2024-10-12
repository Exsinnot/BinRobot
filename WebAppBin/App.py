from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/api/data', methods=['GET'])
def get_data():
    data = {}
    print("GET DATA")
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
            data["Status"]= split_data[4]
            data["Wifi"]= split_data[3]
    print(data)
    # data = {"message": "Hello from Flask!"}
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True,host="0.0.0.0",port=5000)
