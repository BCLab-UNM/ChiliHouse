import os
import serial
import time
import datetime
import numpy as np
import paramiko
import csv

time_ckpt = time.time()

# open file to store temperature and humidity data
csv_file = "logging.csv"
log = open(csv_file, "a")
writer = csv.writer(log, delimiter=",", escapechar=' ', quoting=csv.QUOTE_NONE)

# define serial port and clear cache
ser = serial.Serial('/dev/ttyACM0')
ser.flushInput()

# ssh client setup 
hostname = 'moons.cs.unm.edu'
username = 'chilihouse'
private_key_path = '/home/hanson/.ssh/id_rsa'
remote_directory = '/nfs/guest/chilihouse/public_html'

# create ssh key
ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
private_key = paramiko.RSAKey.from_private_key_file(private_key_path)
ssh_client.connect(hostname, username=username, pkey=private_key)
sftp_client = ssh_client.open_sftp()

try:

    # try to read data from serial port
    while True:
        # read raw data from port
        raw_bytes = ser.readline()
        raw_bytes = raw_bytes.strip(b'\r\n')
        # decode using "utf-8"
        decoded_bytes = raw_bytes.decode("utf-8")

        current_datetime = datetime.datetime.now()

        # add to csv
        # template: time, temp(C), humidity(%)
        decoded_bytes = decoded_bytes.split(",")
        print(f"{current_datetime}: {decoded_bytes}")

        writer.writerow([current_datetime,decoded_bytes[0],decoded_bytes[1]])
        log.flush()

        # every 10 minutes, upload the updated csv to moon.cs.unm.edu directory
        if time.time()-time_ckpt == 600:
            remote_path = f"{remote_directory}/{csv_file}"
            upload_file(sftp_client, csv_file, remote_path)
            time_ckpt = time.time() # reset time checkpoint
            print("file uploaded") 
        # delay between each while call
        time.sleep(1)

# close serial port and file stream
finally:          
    ser.close()
    log.close()
