import pyrealsense2 as rs
import numpy as np
import cv2
import datetime
import time
import os
import paramiko

# Camera model selection
camera_model = 'L515'  # Change to 'L515', 'D415', or 'D455'

# Configure depth and color streams based on the camera model
pipeline = rs.pipeline()
config = rs.config()
# Add your camera configuration here...

# Start the pipeline
pipeline.start(config)

last_saved_time = time.time()

# Server and login details for SFTP
hostname = 'moons.cs.unm.edu'
username = 'chilihouse'
private_key_path = '/home/swarmie/.ssh/id_rsa'
remote_directory = '/nfs/guest/chilihouse'

# Create an SSH client instance
ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
private_key = paramiko.RSAKey.from_private_key_file(private_key_path)
ssh_client.connect(hostname, username=username, pkey=private_key)
sftp_client = ssh_client.open_sftp()

# Function to upload a file using SFTP
def upload_file(sftp_client, local_path, remote_path):
    sftp_client.put(local_path, remote_path)
    print(f"Transferred {local_path} to {remote_path}")

try:
    while True:
        current_time = time.time()

        if current_time - last_saved_time > 60:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            ir_frame = frames.get_infrared_frame()

            if not depth_frame or not color_frame or not ir_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_image = np.asanyarray(ir_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            file_types = ['RGB', 'IR', 'Depth']
            for file_type in file_types:
                image_filename = f'{file_type}_{timestamp}.png'
                symlink_filename = f'{file_type}_Latest.png'

                if file_type == 'Depth':
                    cv2.imwrite(image_filename, depth_colormap)
                elif file_type == 'IR':
                    cv2.imwrite(image_filename, ir_image)
                else:  # RGB
                    cv2.imwrite(image_filename, color_image)

                if os.path.exists(symlink_filename):
                    os.remove(symlink_filename)
                os.symlink(image_filename, symlink_filename)

                remote_path = f"{remote_directory}/{image_filename}"
                upload_file(sftp_client, image_filename, remote_path)

                os.remove(image_filename)

            last_saved_time = current_time

        time.sleep(1)

finally:
    sftp_client.close()
    ssh_client.close()
    pipeline.stop()

