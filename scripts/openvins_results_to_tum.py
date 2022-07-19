import sys


def get_lines(filename):
  lines = []
  with open(filename) as f:
    for i, line in enumerate(f):
      if i == 0: # skip first line
        continue
      listRes = list(line.strip().split(" "))
      listRes = listRes[0:8]
      lines.append(' '.join(listRes))
      # print(lines[-1])
  return lines

def write_tum(lines, out_filename):
    with open(out_filename + '.txt', 'w') as f:
        f.write('\n'.join(lines))

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: openvins_results_to_tum.py results_filename out_filename")
    else:
        lines = get_lines(sys.argv[1])
        write_tum(lines, sys.argv[2])
