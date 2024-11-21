from flask import Flask, request, jsonify, render_template, url_for, send_from_directory
import pandas as pd
import os
import csv
import serial
from datetime import datetime
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('Agg')
import time

app = Flask(__name__)

# Serial setting
ser = serial.Serial('/dev/ttyUSB0', 9600)
ser.flushInput()

# CSV file setting
csv_file = 'data.csv'
csv_header = ['Timestamp', 'Temperature (C)', 'Humidity (%)', 'Moisture (%)']

def update_csv(data):
    file_exists = os.path.isfile(csv_file)
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(csv_header)
        writer.writerow(data)

def calculate_averages_from_csv(csv_file):
    try:
        df = pd.read_csv(csv_file)
        recent_data = df.tail(3)
        temperature_avg = recent_data['Temperature (C)'].mean()
        humidity_avg = recent_data['Humidity (%)'].mean()
        moisture_avg = recent_data['Moisture (%)'].mean()
        return round(temperature_avg, 2), round(humidity_avg, 2), round(moisture_avg, 2)
    except FileNotFoundError:
        return 0, 0, 0

default_temperature, default_humidity, default_moisture = calculate_averages_from_csv(csv_file)

# Define the static directory
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')


# Modify your generate_graph function to use the font
def generate_graph(column_name, label, filename):
    try:
        df = pd.read_csv(csv_file)
        plt.figure(figsize=(10, 5))
        plt.plot(df['Timestamp'], df[column_name], marker='o')
        plt.xlabel('time')
        plt.ylabel(label)
        plt.title(f'{label} change')
        plt.xticks(rotation=45)
        plt.tight_layout()
        filepath = os.path.join(STATIC_DIR, filename)
        plt.savefig(filepath)
        plt.close()
    except FileNotFoundError:
        plt.figure(figsize=(10, 5))
        plt.text(0.5, 0.5, 'CSV file not found.', fontsize=12, ha='center')
        filepath = os.path.join(STATIC_DIR, filename)
        plt.savefig(filepath)
        plt.close()

@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(STATIC_DIR, filename)

@app.route('/index')
@app.route('/')
def index():
    temperature, humidity, moisture = calculate_averages_from_csv(csv_file)

    arduino_data = ser.read(5)
    if arduino_data and arduino_data[0] == 0x02:
        temp, humi, moist, checksum = arduino_data[1], arduino_data[2], arduino_data[3], arduino_data[4]
        if (temp + humi + moist) & 0xFF == checksum:
            temperature, humidity, moisture = temp, humi, moist

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    update_csv([timestamp, temperature, humidity, moisture])
    return render_template('index.html', temperature=temperature, humidity=humidity, moisture=moisture)


@app.route('/page1')
def page1():
    return render_template('page1.html')

@app.route('/box1')
def box1():
    generate_graph('Temperature (C)', 'Temperature (°C)', 'temperature_graph.png') 
    return render_template('box1.html', image_file=url_for('static', filename='temperature_graph.png'))

@app.route('/box2')
def box2():
    generate_graph('Humidity (%)', 'Humidity(%)', 'humidity_graph.png')
    return render_template('box2.html', image_file=url_for('static', filename='humidity_graph.png'))

@app.route('/box3')
def box3():
    generate_graph('Moisture (%)', ' Soil Moisture (%)', 'moisture_graph.png') 
    return render_template('box3.html', image_file=url_for('static', filename='moisture_graph.png'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
