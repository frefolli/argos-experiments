import os

N = 10  
for i in range(1, N + 1):
    os.system("make nox")
    source_folder = "out/"
    destination_folder = f"archive/outs/{i}"
    if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)
    os.rename(source_folder, destination_folder)
