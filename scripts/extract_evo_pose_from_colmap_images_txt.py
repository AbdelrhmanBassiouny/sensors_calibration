#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np

from scipy.spatial.transform import Rotation as Rot

import matplotlib.pyplot as plt
import rosbag

def compute_Twc_from_Tcw(tx,ty,tz,qx,qy,qz,qw):

    tcw = np.array([[tx,ty,tz]]).reshape(3,1)
    qcw = np.array([[qx,qy,qz,qw]]).reshape(4,)

    rot_cw = Rot.from_quat(qcw)

    Rcw = rot_cw.as_dcm()

    Rwc = Rcw.T

    twc = -Rwc.dot(tcw)

    rot_wc = Rot.from_dcm(Rwc)
    qwc = rot_wc.as_quat()

    return twc, qwc

def extract_cam_pose_from_line(line, prev_img_num=0):

    line_els = line.split(" ")

    img_id = int(line_els[0])
    qw = float(line_els[1])
    qx = float(line_els[2])
    qy = float(line_els[3])
    qz = float(line_els[4])
    tx = float(line_els[5])
    ty = float(line_els[6])
    tz = float(line_els[7])
    cam_id = int(line_els[8])
    # img_name = int(line_els[9].split(".")[0].split("_")[-1])
    img_num_str = line_els[9].split(".")[0][5:]
    img_name = int(img_num_str)
    # if img_name < prev_img_num:
    #     img_name = None

    twc, qwc = compute_Twc_from_Tcw(tx,ty,tz,qx,qy,qz,qw)

    tx = twc[0,0]
    ty = twc[1,0]
    tz = twc[2,0]

    qx = qwc[0]
    qy = qwc[1]
    qz = qwc[2]
    qw = qwc[3]

    return img_id, qw, qx, qy, qz, tx, ty, tz, cam_id, img_name

def get_times(bag_in, topic_name):
    lines = []
    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == topic_name:            
            # lines.append(str(msg.header.stamp.to_sec())
            #              if msg._has_header else t)
            lines.append(str(msg.header.stamp.secs) +
                         "." + str(msg.header.stamp.nsecs))
    return lines


if __name__ == "__main__":
    print("Colmap Traj Exporter script!")
    
    # if len(sys.argv) < 3:
    #     print("Usage: python extract_evo_pose_from_colmap.py colmap_images_file.txt out_pose_file.txt img_sampling_in_sec")
    #     exit(-1)
    # sampling_sec = float(sys.argv[3])
    
    bag = None
    topic = None
    times = None
    if len(sys.argv) < 5:
        print("Usage: python extract_evo_pose_from_colmap.py colmap_images_file.txt out_pose_file.txt img_sampling_in_sec times_bag times_topic")
        exit(-1)
    bag = sys.argv[3]
    topic = sys.argv[4]
    if (bag is not None) and (topic is not None):
        times = get_times(bag, topic)
        # print(times)

    print("Going to extract colmap pose from file : " + sys.argv[1])

    in_file = open(sys.argv[1], 'r')

    print("Results to be stored in file : " + sys.argv[2])

    evo_tum_file = open("evo_enu_" + sys.argv[2], 'w')

    print("Provided image sampling : " + sys.argv[3] + " secs")

    header_passed = False

    cam_pose_line = True

    out_lines = []
    i = 0
    img_name = 0
    for line in in_file:

        if header_passed is not True:
            if line[0] == "#":
                continue
            else:
                header_passed = True


        if cam_pose_line is True:
            img_id, qw, qx, qy, qz, tx, ty, tz, cam_id, new_img_name \
                = extract_cam_pose_from_line(line, img_name)
            cam_pose_line = False
            if new_img_name is None:
                continue
            else:
                img_name = new_img_name
            # out_line = str(img_name * sampling_sec) + " " \
            #         + str(tx) + " " + str(ty) + " " + str(tz) + " " \
            #         + str(qx) + " " + str(qy) + " " + str(qz) + " " + str(qw) + "\n"
            out_line = str(times[i]) + " " \
                    + str(tx) + " " + str(ty) + " " + str(tz) + " " \
                    + str(qx) + " " + str(qy) + " " + str(qz) + " " + str(qw) + "\n"
            i += 1

            # Store pose lines before writting because they are unordered in
            # colmap files
            # print(img_name)
            out_lines.append((img_name, out_line))
        else:
            cam_pose_line = True
            continue

    # Sort poses from img_name
    out_lines.sort(key=lambda tup : tup[0])
    i = 0
    for line in out_lines:
        line_list = line[1].split(" ")
        line_list[0] = times[i]
        i += 1
        # line_list.insert(0, str(line[0]))
        new_line = " ".join(line_list)
        evo_tum_file.write(new_line)
        evo_tum_file.flush()

    evo_tum_file.close()
