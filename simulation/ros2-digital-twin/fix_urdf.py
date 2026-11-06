#!/usr/bin/env python3
"""
fix_urdf.py -- Post-process a raw Onshape URDF export for the JS-06 gantry.

Usage:
    python3 fix_urdf.py <input.urdf> <output.urdf>

What this script does (in order):
  1. Fix mesh package URI  -- replaces package://main_assembly/ with
                              package://main_gantry_assembly/
  2. Collapse spurious movable joints to fixed  -- fix_support*, fix_end_piece*
  3. Fix Y-axis (bridge) -- dof_linear_bearing, dof_linear_bearing_1:
                             shift origin Z by -0.84 m to home near motor end,
                             set limits lower=0 upper=1.06 m
  4. Fix Z-axis lead nut  -- collapse dof_lead_nut and dof_lead_nut_1 to fixed,
                             reparent dof_lead_nut to z_attachment with correct
                             transform so the lead screw stays on the carriage
  5. Add joint_z  -- a clean prismatic joint from z_attachment to nozzle_part
                     with the correct origin, axis, and limits (0 to 0.25 m)
  6. Remove fix_end_piece_3  -- was the old continuous joint from fix_end_piece
                                to nozzle_part; replaced by joint_z
  7. Carriage home position  -- joint_z origin adjusted so the carriage rests
                                just below the Z end piece at slider=0
  8. Anchor root to world  -- the exported root link is free-floating with no
                              joint and no inertial; add a fixed world->root
                              joint so the assembly doesn't free-fall in Gazebo
  9. Disambiguate joint/link name collisions -- Onshape's exporter gives a
     zero-geometry placeholder link the *same name* as its own joint for
     several mate connectors (dof_lead_nut, fix_support*, fix_end_piece*).
     URDF keeps links and joints in separate namespaces so this is legal
     there and invisible to RViz/robot_state_publisher, but SDF (which
     Gazebo converts to internally) uses one shared namespace, so these
     collisions corrupt Gazebo's pose/frame graph (seen as "already exists"
     and "cycle detected" errors). Renaming just the joint (not the link,
     which nothing else needs to reference) resolves it.
  10. Collapse dof_linear_bearing_1 to fixed, remove its mimic tag --
      verified empirically (headless Gazebo spawn) that:
      (a) the physics engine here (DART) does not support mimic
          constraints at all -- gz-sim logs "chosen physics engine does
          not support mimic constraints" and applies none, so
          dof_linear_bearing would never track dof_linear_bearing_1
          in simulation regardless;
      (b) tracing the actual kinematic chain shows dof_linear_bearing_1's
          child (default_13) is a dead end (nothing attaches downstream),
          while dof_linear_bearing's child (default_1) is what actually
          carries the X-bridge/carriage/Z-axis chain -- i.e. the URDF's
          own <mimic> labeling has the "real" and "cosmetic" Y-bearing
          joints backwards relative to what the kinematic tree needs.
      dof_linear_bearing_1 is therefore the one collapsed to fixed (it
      drives nothing), and dof_linear_bearing is the one that carries an
      effort command interface below.
  11. Inject <ros2_control> and <gazebo> plugin blocks for the 3 true DOF
     joints (dof_linear_bearing_2, dof_linear_bearing, joint_z), so the
     gz_ros2_control effort interface is available for closed-loop control.
  12. Colour the uncoloured crop meshes green -- Onshape assigned a material
      to some crop meshes (the radish root bulb: reddish-brown, one radish
      leaf/stem mesh: purple) but not others (BokChoy, Lettuce, and the
      radish's other leaf mesh), which render as default grey with no
      <material> at all. Adds a green <material> to every <visual> using
      one of those three uncoloured mesh files, matched by mesh filename
      (not link name) so it's robust to however many numbered instance
      copies exist or how they're renamed in a fresh export, and leaves
      the already-coloured radish meshes untouched.
"""

import re
import sys
import xml.etree.ElementTree as ET


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def set_joint_type(content: str, joint_name: str, new_type: str) -> str:
    return re.sub(
        rf'(<joint\s+name="{re.escape(joint_name)}"\s+type=")[^"]*(")',
        rf'\g<1>{new_type}\2',
        content,
    )


def set_joint_origin(content: str, joint_name: str, xyz: str, rpy: str) -> str:
    return re.sub(
        rf'(<joint\s+name="{re.escape(joint_name)}".*?<origin\s+)xyz="[^"]*"\s+rpy="[^"]*"',
        rf'\g<1>xyz="{xyz}" rpy="{rpy}"',
        content,
        flags=re.DOTALL,
    )


def set_joint_parent(content: str, joint_name: str, new_parent: str) -> str:
    # Replace only the first <parent link="..."> inside the named joint block
    jstart = content.find(f'<joint name="{joint_name}"')
    jend   = content.find('</joint>', jstart) + len('</joint>')
    block  = content[jstart:jend]
    block  = re.sub(
        r'(<parent\s+link=")[^"]*(")',
        rf'\g<1>{new_parent}\2',
        block,
        count=1,
    )
    return content[:jstart] + block + content[jend:]


def set_limit(content: str, joint_name: str, lower: float, upper: float) -> str:
    jstart = content.find(f'<joint name="{joint_name}"')
    jend   = content.find('</joint>', jstart) + len('</joint>')
    block  = content[jstart:jend]
    block  = re.sub(r'(<limit[^/]*)lower="[^"]*"', rf'\g<1>lower="{lower}"', block)
    block  = re.sub(r'(<limit[^/]*)upper="[^"]*"', rf'\g<1>upper="{upper}"', block)
    return content[:jstart] + block + content[jend:]


def remove_joint(content: str, joint_name: str) -> str:
    pattern = (
        rf'\s*<joint\s+name="{re.escape(joint_name)}".*?</joint>'
    )
    return re.sub(pattern, '', content, flags=re.DOTALL)


def rename_joint(content: str, old_name: str, new_name: str) -> str:
    """Rename a <joint>'s own name attribute only -- does not touch any
    <link> of the same name, or any <parent>/<child> tag (which reference
    links, not joints, so they are unaffected by a joint rename)."""
    return re.sub(
        rf'(<joint\s+name=")({re.escape(old_name)})(")',
        rf'\g<1>{new_name}\g<3>',
        content,
        count=1,
    )


def add_material_by_mesh(content: str, mesh_filename: str, rgba: str) -> str:
    """Add a <material><color rgba="..."/></material> to every <visual>
    block that references mesh_filename and doesn't already have one.
    Matched by mesh filename (not link name), so it applies uniformly no
    matter how many numbered instance copies of a part exist or how
    they're named -- and never touches a <visual> whose mesh already has
    an assigned colour."""
    material_name = rgba.replace(' ', '_').rsplit('_', 1)[0] + '_0.000000_0.000000'

    def repl(m: 're.Match') -> str:
        block = m.group(0)
        if '<material' in block or f'meshes/{mesh_filename}"' not in block:
            return block
        return re.sub(
            r'</geometry>\s*</visual>',
            (
                f'</geometry>\n'
                f'            <material name="{material_name}">\n'
                f'                <color rgba="{rgba}" />\n'
                f'            </material>\n'
                f'        </visual>'
            ),
            block,
        )

    return re.sub(r'<visual>.*?</visual>', repl, content, flags=re.DOTALL)


# ---------------------------------------------------------------------------
# Main transform
# ---------------------------------------------------------------------------

def transform(content: str) -> str:

    # ------------------------------------------------------------------
    # 1. Fix mesh package URI
    # ------------------------------------------------------------------
    content = content.replace(
        'package://main_assembly/',
        'package://main_gantry_assembly/',
    )

    # ------------------------------------------------------------------
    # 2. Collapse spurious movable joints to fixed
    # ------------------------------------------------------------------
    spurious_fixed = [
        'fix_support', 'fix_support_1', 'fix_support_2', 'fix_support_3',
        'fix_end_piece', 'fix_end_piece_1', 'fix_end_piece_2',
    ]
    for name in spurious_fixed:
        content = set_joint_type(content, name, 'fixed')

    # ------------------------------------------------------------------
    # 3. Fix Y-axis (bridge) origin and limits
    #
    #    The Onshape export centres the bridge mid-bed with symmetric
    #    ±0.5 m limits.  The physical home position is near the Y motor,
    #    which is 0.84 m from the export origin along the bed depth (Z in
    #    URDF world frame).  Shifting origin Z by -0.84 m places slider=0
    #    at the motor end; upper=1.06 m covers the full bed travel.
    #
    #    dof_linear_bearing   original xyz: 0.626  0  0.895974
    #    dof_linear_bearing_1 original xyz: 0      0  0.895974
    #    After shift Z -= 0.84:
    #    dof_linear_bearing   new xyz:      0.626  0  0.055974
    #    dof_linear_bearing_1 new xyz:      0      0  0.055974
    # ------------------------------------------------------------------
    def set_joint_origin_xyz(c, name, new_xyz):
        jstart = c.find(f'<joint name="{name}"')
        jend   = c.find('</joint>', jstart) + len('</joint>')
        block  = c[jstart:jend]
        block  = re.sub(r'(<origin\s+)xyz="[^"]*"', rf'\g<1>xyz="{new_xyz}"', block)
        return c[:jstart] + block + c[jend:]

    content = set_joint_origin_xyz(
        content, 'dof_linear_bearing',   '0.626 0.0 0.055974')
    content = set_joint_origin_xyz(
        content, 'dof_linear_bearing_1', '0.0 0.0 0.055974')

    for name in ['dof_linear_bearing', 'dof_linear_bearing_1']:
        content = set_limit(content, name, lower=0.0, upper=1.06)

    # ------------------------------------------------------------------
    # 4. Fix Z-axis lead nut
    #    - Collapse both dof_lead_nut and dof_lead_nut_1 to fixed so the
    #      screw does not move with the nozzle carriage.
    #    - Reparent dof_lead_nut from default_18 to z_attachment and give
    #      it the correct transform so the screw stays on the motor mount.
    #
    #    Origin computed by composing the original chain:
    #      z_attachment -> z_axis_end_piece (fix_z_axis)
    #      -> fix_end_piece_2 chain (xyz=-0.023,0.000156,-0.01555 rpy=0,0,pi)
    #      -> nozzle_part (fix_end_piece_3, xyz=0)
    #      -> default_18 (fix_nut, xyz=-0.012,0.012,0.1315 rpy=0,pi/2,-pi)
    #      -> dof_lead_nut link (dof_lead_nut, xyz=-0.157373,0,0 rpy=pi/2,1.406,-pi/2)
    #    Result: xyz="-0.278324 -0.011844 -0.031800"
    # ------------------------------------------------------------------
    content = set_joint_type(content, 'dof_lead_nut',   'fixed')
    content = set_joint_type(content, 'dof_lead_nut_1', 'fixed')
    content = set_joint_parent(content, 'dof_lead_nut', 'z_attachment')
    content = set_joint_origin(
        content, 'dof_lead_nut',
        xyz='-0.278324 -0.011844 -0.031800',
        rpy='1.570800 1.405720 -1.570800',
    )

    # ------------------------------------------------------------------
    # 5. Remove fix_end_piece_3 (old continuous joint to nozzle_part)
    #    It is replaced by the clean joint_z below.
    # ------------------------------------------------------------------
    content = remove_joint(content, 'fix_end_piece_3')

    # ------------------------------------------------------------------
    # 6. Add joint_z -- clean prismatic Z axis
    #
    #    Parent: z_attachment  Child: nozzle_part
    #    Axis:   0 0 -1  (positive slider = nozzle moves down into bed)
    #    Limits: 0.0 to 0.25 m
    #
    #    Origin xyz computed from the chain:
    #      z_attachment -> z_axis_end_piece: xyz=(-0.005, 0, -0.0208) rpy=(0,-pi/2,0)
    #      -> fix_end_piece_2: xyz=(-0.023, 0.000156, -0.01555) rpy=(0,0,pi)
    #    Composed result: xyz=(0.01055, 0.000156, -0.0438) rpy=(-pi, pi/2, 0)
    #
    #    Carriage home offset: shift -0.1385 along local X (= +138.5 mm up
    #    in world Z) so the carriage rests just below the Z end piece at
    #    slider = 0.  Determined empirically against the RViz model.
    #    Final origin xyz: (-0.138450, 0.000156, -0.043800)
    # ------------------------------------------------------------------
    new_joint = """
  <joint name="joint_z" type="prismatic">
    <parent link="z_attachment"/>
    <child link="nozzle_part"/>
    <origin xyz="-0.138450 0.000156 -0.043800" rpy="-3.141593 1.570796 0.000000"/>
    <axis xyz="0 0 -1"/>
    <limit lower="0.0" upper="0.25" effort="100" velocity="0.1"/>
  </joint>
"""
    content = content.replace('</robot>', new_joint + '\n</robot>')

    # ------------------------------------------------------------------
    # 8. Anchor root to world
    #
    #    The raw export's "root" link has no parent joint and no
    #    <inertial>, so in Gazebo it is a free-floating body that will
    #    fall under gravity. Add a fixed joint from a new "world" link
    #    to anchor it in place.
    # ------------------------------------------------------------------
    world_block = """
  <link name="world"/>
  <joint name="world_to_root" type="fixed">
    <parent link="world"/>
    <child link="root"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
"""
    content = content.replace('</robot>', world_block + '\n</robot>')

    # ------------------------------------------------------------------
    # 9. Disambiguate joint/link name collisions
    #
    #    Onshape's exporter names a zero-geometry placeholder link the same
    #    as its own joint for these 7 mate connectors. Legal in URDF
    #    (separate link/joint namespaces), illegal in SDF (Gazebo converts
    #    to internally) -- causes "already exists" / pose-graph cycle
    #    errors. Only the joint is renamed; the link (and everything that
    #    references the link by name) is untouched.
    # ------------------------------------------------------------------
    for name in ['dof_lead_nut', 'fix_support', 'fix_support_1', 'fix_support_2',
                 'fix_end_piece', 'fix_end_piece_1', 'fix_end_piece_2']:
        content = rename_joint(content, name, f'{name}_joint')

    # ------------------------------------------------------------------
    # 10. Collapse dof_linear_bearing_1 to fixed; drop the (unusable) mimic
    #     tag on dof_linear_bearing -- see module docstring point 10.
    # ------------------------------------------------------------------
    content = set_joint_type(content, 'dof_linear_bearing_1', 'fixed')
    content = re.sub(
        r'\s*<mimic[^/]*joint="dof_linear_bearing_1"[^/]*/>',
        '',
        content,
    )

    # ------------------------------------------------------------------
    # 11. ros2_control + gazebo plugin blocks for closed-loop effort control
    #
    #    Only the 3 true independent DOF joints get command interfaces:
    #    dof_linear_bearing_2 (X), dof_linear_bearing (Y -- see point 10
    #    for why this one, not dof_linear_bearing_1), joint_z (Z).
    #
    #    Effort min/max below mirror each joint's current URDF <limit
    #    effort="..."> value. These are Onshape-derived placeholders, not
    #    real motor specs -- replace them once real motor limits are known.
    #
    #    The controller YAML path is left as a literal placeholder token;
    #    the launch file resolves it with a plain string .replace() before
    #    publishing robot_description (this repo has no xacro toolchain).
    # ------------------------------------------------------------------
    ros2_control_block = """
  <ros2_control name="GantryEffortSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="dof_linear_bearing_2">
      <command_interface name="effort">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="dof_linear_bearing">
      <command_interface name="effort">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint_z">
      <command_interface name="effort">
        <param name="min">-100</param>
        <param name="max">100</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>__GANTRY_CONTROLLERS_YAML_PATH__</parameters>
    </plugin>
  </gazebo>
"""
    content = content.replace('</robot>', ros2_control_block + '\n</robot>')

    # ------------------------------------------------------------------
    # 12. Colour the uncoloured crop meshes green
    # ------------------------------------------------------------------
    GREEN_RGBA = '0.133333 0.545098 0.133333 1'
    for mesh in ['BokChoy__12_.stl', 'Lettuce__11_.stl', 'Radish__4_.stl']:
        content = add_material_by_mesh(content, mesh, GREEN_RGBA)

    return content


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate(content: str) -> bool:
    tree = ET.parse(__import__('io').StringIO(content))
    root = tree.getroot()
    links    = {l.get('name') for l in root.findall('link')}
    joints   = root.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    roots    = links - children

    errors = []
    for j in joints:
        p = j.find('parent').get('link')
        c = j.find('child').get('link')
        if p not in links:
            errors.append(f'Missing parent link "{p}" in joint "{j.get("name")}"')
        if c not in links:
            errors.append(f'Missing child link "{c}" in joint "{j.get("name")}"')

    print(f'Links: {len(links)}  Joints: {len(joints)}  Root links: {roots}')

    if errors:
        for e in errors:
            print(f'ERROR: {e}')
        return False

    if len(roots) != 1:
        print(f'ERROR: expected 1 root link, found {len(roots)}: {roots}')
        return False

    print('Validation passed.')
    print('\nNon-fixed joints after transform:')
    for j in joints:
        if j.get('type') != 'fixed':
            p = j.find('parent').get('link')
            c = j.find('child').get('link')
            limit = j.find('limit')
            lim = (f"lower={limit.get('lower')} upper={limit.get('upper')}"
                   if limit is not None else 'no limits')
            print(f'  {j.get("type"):12s}  {j.get("name"):40s}  {p} -> {c}  {lim}')

    return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f'Usage: python3 {sys.argv[0]} <input.urdf> <output.urdf>')
        sys.exit(1)

    input_path  = sys.argv[1]
    output_path = sys.argv[2]

    with open(input_path, 'r') as f:
        content = f.read()

    print(f'Input:  {input_path}')
    content = transform(content)

    ok = validate(content)

    with open(output_path, 'w') as f:
        f.write(content)

    print(f'Output: {output_path}')
    sys.exit(0 if ok else 1)
