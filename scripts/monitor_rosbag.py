#!/usr/bin/env python3

import subprocess
import time

def main():
    time.sleep(5)  # Añade un retraso de 5 segundos
    try:
        rosbag_pid = subprocess.check_output(["pgrep", "-f", "rosbag play"]).strip()
        print(f"rosbag play process ID: {rosbag_pid.decode('utf-8')}")
    except subprocess.CalledProcessError:
        print("rosbag play process not found.")
        return

    while True:
        try:
            subprocess.check_output(["ps", "-p", rosbag_pid])
        except subprocess.CalledProcessError:
            print("rosbag play process has finished.")
            break
        time.sleep(1)

if __name__ == "__main__":
    main()
