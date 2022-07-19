import rosbag
import sys
from cv_bridge import CvBridge
import cv2
import numpy as np


def read_poses(in_file):
  all_poses = []
  with open(in_file) as f:
    for i, line in enumerate(f):
      listRes = list(line.strip().split(" "))
      if listRes[-1][:5] == "frame":
        listRes = listRes[1:-2]
        # listRes = list(map(lambda x: float(x), listRes))
        newlistRes = listRes[4:-1]
        newlistRes.extend(listRes[0:4])
        
        all_poses.append(newlistRes)

def get_times(bag_in, topic_name):
    times = []
    i = 0
    bag = rosbag.Bag(bag_in)
    for k, b in enumerate(bag.read_messages(topic_name)):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
        cv_image.astype(np.uint8)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite("frame" + str(i) + '.jpg', cv_image)
        image_time = b.message.header.stamp.to_sec() if b.message._has_header else b.timestamp
        print('saved: ' + "frame" + str(i) + '.jpg' +
              " with time " + str(image_time))
        times.append(str(image_time))
        i += 1
    return times
        
def write_gt(times, outfile):
    with open(outfile+'.txt', 'w') as f:
        f.writelines('\n'.join(times))

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("usage: generate_gt_tum.py times_bag times_topic outfile")
    else:
        # poses = read_poses(sys.argv[1])
        times = get_times(sys.argv[1], sys.argv[2])
        write_gt(times, sys.argv[3])
