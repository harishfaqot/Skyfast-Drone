#!/usr/bin/env python3

import rospy
import csv
from nav_msgs.msg import Odometry

class WaypointRecorder:
    def __init__(self):
        self.latest_odom = None
        self.waypoints = []
        
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.latest_odom = msg.pose.pose

    def run(self):
        rospy.loginfo("Sistem Perekam Waypoint Aktif.")
        print("\n" + "="*50)
        print("INSTRUKSI:")
        print("1. Bawa drone ke titik yang diinginkan secara manual.")
        print("2. Tekan tombol [ENTER] untuk merekam titik tersebut.")
        print("3. Ketik 'q' lalu tekan [ENTER] untuk selesai dan menyimpan ke CSV.")
        print("="*50 + "\n")

        while not rospy.is_shutdown():
            try:
                user_input = input("Tekan ENTER untuk rekam, atau 'q' untuk simpan & keluar: ")
                
                if user_input.lower() == 'q':
                    break
                
                if self.latest_odom is not None:
                    x = self.latest_odom.position.x
                    y = self.latest_odom.position.y
                    z = self.latest_odom.position.z
                    
                    self.waypoints.append([x, y, z])
                    print(f"[REKAM] Waypoint ke-{len(self.waypoints)}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}\n")
                else:
                    print("[WARNING] Belum ada data Odometry yang masuk. Pastikan topik benar.\n")
                    
            except EOFError:
                break # Menangani jika di-close paksa

        self.save_csv("waypoints.csv")

    def save_csv(self, filename):
        if len(self.waypoints) == 0:
            print("Tidak ada waypoint yang direkam. File CSV tidak dibuat.")
            return

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z']) # Header kolom
            writer.writerows(self.waypoints)
            
        print(f"\n[SUKSES] {len(self.waypoints)} waypoint berhasil disimpan ke file '{filename}'.")

if __name__ == '__main__':
    rospy.init_node('waypoint_recorder', disable_signals=True) 
    
    recorder = WaypointRecorder()
    try:
        recorder.run()
    except KeyboardInterrupt:
        recorder.save_csv("waypoints.csv")
        