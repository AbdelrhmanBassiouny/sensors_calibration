import rosbag
import sys


def my_node(bag_in, bag_out):
  with rosbag.Bag(bag_out, 'w') as outbag:
      for topic, msg, t in rosbag.Bag(bag_in).read_messages():
          # This also replaces tf timestamps under the assumption
          # that all transforms in the message share the same timestamp
          if topic == "/tf" and msg.transforms:
              outbag.write(topic, msg, msg.transforms[0].header.stamp)
          else:
              outbag.write(
                  topic, msg, msg.header.stamp if msg._has_header else t)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: my_node.py arg1 arg2")
    else:
        my_node(sys.argv[1], sys.argv[2])