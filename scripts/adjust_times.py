import sys


def read_poses(in_file, out_file=None):
  new_lines = []
  with open(in_file) as f:
    for i, line in enumerate(f):
      listRes = list(line.strip().split(" "))
      time_str = listRes[0].split(".")[0]
      time_str = time_str[:10] + '.' + time_str[10:]
      # print(time_str)
      listRes[0] = time_str
      new_lines.append(' '.join(listRes) + "\n")
  with open(out_file, 'w') as f:
    f.writelines(new_lines)

def add_times(in_file, out_file=None):
  times = []
  with open(in_file) as f:
    for i, line in enumerate(f):
      line.strip('\n')

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("usage: read_camera_poses.py in_file [out_file]")
  elif len(sys.argv) == 2:
    read_poses(sys.argv[1])
  elif len(sys.argv) == 3:
    read_poses(sys.argv[1], out_file=sys.argv[2])
