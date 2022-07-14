#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import sqlite3
import collections
import struct
from time import time
import tqdm

import numpy as np

from scipy.spatial.transform import Rotation as Rot

import matplotlib.pyplot as plt

import rosbag

BaseImage = collections.namedtuple(
    "Image", ["id", "qvec", "tvec", "camera_id", "name", "xys", "point3D_ids"])


class Image(BaseImage):
    def qvec2rotmat(self):
        return qvec2rotmat(self.qvec)


def qvec2rotmat(qvec):
    return np.array([
        [1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
         2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
         2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]],
        [2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
         1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
         2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]],
        [2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
         2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
         1 - 2 * qvec[1]**2 - 2 * qvec[2]**2]])


def read_next_bytes(fid, num_bytes, format_char_sequence, endian_character="<"):
    """Read and unpack the next bytes from a binary file.
    :param fid:
    :param num_bytes: Sum of combination of {2, 4, 8}, e.g. 2, 6, 16, 30, etc.
    :param format_char_sequence: List of {c, e, f, d, h, H, i, I, l, L, q, Q}.
    :param endian_character: Any of {@, =, <, >, !}
    :return: Tuple of read and unpacked values.
    """
    data = fid.read(num_bytes)
    return struct.unpack(endian_character + format_char_sequence, data)


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


def read_images_binary(path_to_model_file, cam_idx=1):
    """
    see: src/base/reconstruction.cc
        void Reconstruction::ReadImagesBinary(const std::string& path)
        void Reconstruction::WriteImagesBinary(const std::string& path)
    """
    images = {}
    with open(path_to_model_file, "rb") as fid:
        num_reg_images = read_next_bytes(fid, 8, "Q")[0]
        for _ in range(num_reg_images):
            binary_image_properties = read_next_bytes(
                fid, num_bytes=64, format_char_sequence="idddddddi")
            image_id = binary_image_properties[0]
            qvec = np.array(binary_image_properties[1:5])
            tvec = np.array(binary_image_properties[5:8])
            camera_id = binary_image_properties[8]

            image_name = ""
            current_char = read_next_bytes(fid, 1, "c")[0]
            while current_char != b"\x00":   # look for the ASCII 0 entry
                image_name += current_char.decode("utf-8")
                current_char = read_next_bytes(fid, 1, "c")[0]
            num_points2D = read_next_bytes(fid, num_bytes=8,
                                           format_char_sequence="Q")[0]
            x_y_id_s = read_next_bytes(fid, num_bytes=24*num_points2D,
                                       format_char_sequence="ddq"*num_points2D)
            
            # Done here to make sure that line is fully read!
            if camera_id != cam_idx:
                continue


            xys = np.column_stack([tuple(map(float, x_y_id_s[0::3])),
                                   tuple(map(float, x_y_id_s[1::3]))])
            point3D_ids = np.array(tuple(map(int, x_y_id_s[2::3])))
            images[image_id] = Image(
                id=image_id, qvec=qvec, tvec=tvec,
                camera_id=camera_id, name=image_name,
                xys=xys, point3D_ids=point3D_ids)
    return images


def extract_cam_pose_from_line(line):

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
    img_name = int(line_els[9].split(".")[0].split("_")[-1])

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
            lines.append(str(msg.header.stamp.to_sec())
                         if msg._has_header else t)
    return lines


if __name__ == "__main__":
    print("Colmap Traj Exporter script!")
    wc = False
    bag = None
    topic = None
    times = None
    if len(sys.argv) < 4 or len(sys.argv) == 5:
        print("Usage: python extract_evo_pose_from_colmap_images_bin.py colmap_images_file.bin out_pose_file.txt camera_to_world(wc) [times_bag times_topic]")
        exit(-1)
    elif len(sys.argv) == 6:
        bag = sys.argv[4]
        topic = sys.argv[5]

    print("Going to extract colmap pose from file : " + sys.argv[1])

    colmap_images = read_images_binary(sys.argv[1])

    print("Results to be stored in file : " + sys.argv[2])

    evo_tum_file = open("evo_enu_" + sys.argv[2], 'w')
    
    wc = bool(sys.argv[3])
    
    if (bag is not None) and (topic is not None):
        times = get_times(bag, topic)
    
    out_lines = []
    i = 0
    for _, colmap_image in tqdm.tqdm(colmap_images.items()):
        
        if wc:  # Twc (cam -> world)
            tx = colmap_image.tvec[0]
            ty = colmap_image.tvec[1]
            tz = colmap_image.tvec[2]
            
            qw = colmap_image.qvec[0]
            qx = colmap_image.qvec[1]
            qy = colmap_image.qvec[2]
            qz = colmap_image.qvec[3]
        else:
            # Extract Tcw (world -> cam)
            Rcw = colmap_image.qvec2rotmat().reshape(3,3)
            tcw = colmap_image.tvec.reshape(3,1)

            Rwc = Rcw.T
            twc = -Rwc.dot(tcw)

            qwc = Rot.from_dcm(Rwc).as_quat()

            tx = twc[0,0]
            ty = twc[1,0]
            tz = twc[2,0]

            qx = qwc[0]
            qy = qwc[1]
            qz = qwc[2]
            qw = qwc[3]
        
        if times is not None:
            timestamp = times[i]
        else:
            timestamp = colmap_image.id
        img_name = colmap_image.name
        # Extract Times from Ifremer Data Images if needed
        # (format: yyyymmddThhmmss.msZ.jpg)
        if img_name[8] == 'T' and img_name[-5] == 'Z' and img_name[-3:] == "jpg":
            img_name = colmap_image.name.split(".")
            img_time_els = ".".join(img_name[-3:-1])
            img_time = img_time_els.split("T")[-1][:-1]

            timestamp = float(img_time[0:2]) * 3600. + float(img_time[2:4]) * 60. + float(img_time[4:])

        # Store pose lines before writting because they are unordered in
        # colmap files
        out_line = str(timestamp) + " " \
                    + str(tx) + " " + str(ty) + " " + str(tz) + " " \
                    + str(qx) + " " + str(qy) + " " + str(qz) + " " + str(qw) + "\n"

        out_lines.append((timestamp, out_line))
        i += 1

    # Sort poses from timestamp
    out_lines.sort(key=lambda tup : tup[0])

    for line in out_lines:
        evo_tum_file.write(line[1])
        evo_tum_file.flush()

    evo_tum_file.close()
