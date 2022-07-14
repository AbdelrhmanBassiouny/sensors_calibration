import rosbag
import sys


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
    lines = []
    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == topic_name:
            lines.append(str(msg.header.stamp.to_sec()) if msg._has_header else t)
    return lines
        
def write_gt(poses, times, outfile):
    lines = []
    with open(outfile+'.txt', 'w') as f:
        for t, pose in zip(times, poses):
            lines.append(t + pose)
            f.write('\n'.join(times))

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("usage: generate_gt_tum.py poses_file times_bag times_topic outfile")
    else:
        poses = read_poses(sys.argv[1])
        times = get_times(sys.argv[2], sys.argv[3])
        write_gt(poses, times, sys.argv[4])
