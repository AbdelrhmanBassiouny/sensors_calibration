import sys


def read_poses(in_file, out_file=None):
  all_poses = []
  with open(in_file) as f:
    for i, line in enumerate(f):
      listRes = list(line.strip().split(" "))
      if listRes[-1][:5] == "frame":
        listRes = listRes[1:-2]
        listRes = list(map(lambda x: float(x), listRes))
        newlistRes = listRes[4:-1]
        newlistRes.extend(listRes[0:4])
        all_poses.append(newlistRes)
        # print(listRes)

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
