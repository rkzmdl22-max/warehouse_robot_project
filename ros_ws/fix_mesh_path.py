import re

src = "/ros_ws/tiago.urdf"
dst = "/ros_ws/tiago_fixed.urdf"

with open(src, "r") as f:
    data = f.read()


replacements = {
    "package://pmb2_description":
    "/ros_ws/src/pmb2_robot/pmb2_description",

    "package://pal_urdf_utils":
    "/ros_ws/src/pal_urdf_utils",

    "package://pal_hey5_description":
    "/ros_ws/src/pal_hey5/pal_hey5_description",

    "package://tiago_description":
    "/ros_ws/src/tiago_robot/tiago_description",
}


for old, new in replacements.items():
    data = data.replace(old, new)


with open(dst, "w") as f:
    f.write(data)


print("created:", dst)