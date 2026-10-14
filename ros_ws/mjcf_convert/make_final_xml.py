from pathlib import Path

base = Path("output/tiago")


def remove_wrapper(text, tag):
    text = text.replace(f"<{tag}>", "")
    text = text.replace(f"</{tag}>", "")
    return text


assets = remove_wrapper(
    (base / "assets.xml").read_text(),
    "asset"
)

bodies = remove_wrapper(
    (base / "bodies.xml").read_text(),
    "worldbody"
)

actuators = remove_wrapper(
    (base / "actuators.xml").read_text(),
    "actuator"
)

sensors = remove_wrapper(
    (base / "sensors.xml").read_text(),
    "sensor"
)


xml = f"""
<mujoco model="tiago">

<compiler
    angle="radian"
    meshdir="meshes"
    autolimits="true"
/>


<option
    timestep="0.001"
    gravity="0 0 -9.81"
/>


<default>

    <joint
        limited="true"
        damping="0.01"
        armature="0.01"
        frictionloss="0.01"
    />


    <geom
        condim="4"
        contype="1"
        conaffinity="15"
    />


    <motor
        ctrllimited="true"
    />


    <default class="visualgeom">

        <geom
            contype="0"
            conaffinity="0"
        />

    </default>

</default>


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


(base / "tiago_final.xml").write_text(xml)

print("created output/tiago/tiago_final.xml")
