import os
import sys

DESTDIR="archive/outs"

if len(sys.argv) > 1:
  DESTDIR = os.path.join(DESTDIR, sys.argv[1])

N = 10  
for i in range(1, N + 1):
  os.system("make nox")
  source_folder = "out/"
  destination_folder = os.path.join(DESTDIR, str(i))
  if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)
  os.rename(source_folder, destination_folder)
