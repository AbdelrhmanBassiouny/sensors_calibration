import rosbag
import sys


def get_lines(bag_in, pose_topic):
    lines = []
    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        outbag.write(
            topic, msg, msg.header.stamp if msg._has_header else t)
        lines.append(' '.join([str(msg.header.stamp), str()]))
        
def write_tum(lines):
    with open('tum.txt', 'w') as f:
        f.write('\n'.join(lines))

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: my_node.py arg1 arg2")
    else:
        get_lines(sys.argv[1], sys.argv[2])