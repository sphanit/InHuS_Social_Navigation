import os
import glob

dir_path = "png_data"
if not os.path.exists(dir_path):
    os.makedirs(dir_path)

imgs = []

for file in glob.glob("*.pgm"):
    png_name = file;
    png_name = png_name.replace("pgm","png")
    os.system("convert "+ file + " -scale 150x " + dir_path + "/" + png_name)
    os.system("rm -rf "+file)
    imgs.append(file)

print("Converted all",len(imgs), "images to PNG")
