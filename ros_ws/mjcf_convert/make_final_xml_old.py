from pathlib import Path
import re

base = Path("output/tiago")


def extract_inner(text, tag):
    pattern = rf"<{tag}[^>]*>(.*?)</{tag}>"
    result = re.search(pattern, text, re.S)

    if result:
        return result.group(1)

    return text


assets = extract_inner(
    (base / "assets.xml").read_text(),
    "asset"
)

bodies = extract_inner(
    (base / "bodies.xml").read_text(),
    "worldbody"
)

actuators = extract_inner(
    (base / "actuators.xml").read_text(),
    "actuator"
)

sensors = extract_inner(
    (base / "sensors.xml").read_text(),
    "sensor"
)


xml = f"""<mujoco model="tiago">

    <compiler
        angle="radian"
        meshdir="meshes"
        autolimits="true"
    />

    <option
        timestep="0.001"
        gravity="0 0 -9.81"
        iterations="50"
    />


    <asset>
{assets}
    </asset>


    <worldbody>
{bodies}
    </worldbody>


    <actuator>
{actuators}
    </actuator>


    <sensor>
{sensors}
    </sensor>


</mujoco>
"""


out = base / "tiago_final.xml"
out.write_text(xml)

print("generated:", out)
