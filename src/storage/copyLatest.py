import os
import re
import shutil
from datetime import datetime

# Set the directories
source_directory = "/nfs/research/chilihouse/images/"
link_directory = "/nfs/guest/chilihouse/public_html/"

# Define the file types to look for
file_types = ['RGB', 'Depth', 'IR']

# Initialize a dictionary to store the latest file of each type
latest_files = {file_type: ('', datetime.min) for file_type in file_types}

# Regular expression to match the files and extract date and time
pattern = re.compile(r'(\w+)_(\d{8})_(\d{6})\.png')

# Loop through all files in the source directory
for filename in os.listdir(source_directory):
    match = pattern.match(filename)
    if match:
        file_type, date, time = match.groups()
        if file_type in file_types:
            # Parse the date and time into a datetime object
            file_datetime = datetime.strptime(date + time, '%Y%m%d%H%M%S')
            # If this file is more recent than the current latest, update the record
            if file_datetime > latest_files[file_type][1]:
                latest_files[file_type] = (filename, file_datetime)

# Copy the latest files and write their timestamps to separate text files
for file_type, (filename, file_datetime) in latest_files.items():
    if filename:  # Check if a latest file was found
        # Copy the latest image file
        src = os.path.join(source_directory, filename)
        dst = os.path.join(link_directory, f"{file_type}_Latest.png")
        # Remove the old file if it exists
        if os.path.exists(dst):
            os.remove(dst)
        # Copy the new file
        shutil.copy2(src, dst)  # Use copy2 to preserve file metadata
        print(f"{dst} copied from {src}")

        # Write the timestamp to a new text file
        timestamp_filename = os.path.join(link_directory, f"{file_type}_timestamp.txt")
        with open(timestamp_filename, 'w') as f:
            f.write(f"{file_datetime.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Timestamp for {file_type} saved in {timestamp_filename}")
