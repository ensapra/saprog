Generic shared-mesh URDF
========================
All four x1/hip-mount links now share ONE STL: meshes/hip_mount.stl.
This is the former front_right_11.STL, exactly as requested. The back-right,
back-left and front-left hip mounts have appropriate local mesh rotations.

Left and right assemblies share meshes between front/back and use generic
part-based filenames. Eight unique STLs remain:
  chassis.stl, hip_mount.stl,
  left_linkage.stl, left_crank.stl, left_leg.stl,
  right_linkage.stl, right_crank.stl, right_leg.stl

URDF link/joint names remain unchanged to preserve compatibility with robot
state publishers, controller configs, and existing launch files. Joint axes,
limits, inertial properties, pivots and mimic settings are unchanged.

IMPORTANT GEOMETRY LIMITATION: The original back_right_21.STL was not an exact
geometric match to front_right_11.STL. Most vertices match after a 180 degree
X rotation, but the back_right original includes some different geometry.
Per request, all four now use the front_right_11-derived geometry, so slight
visible differences are expected. Recheck the robot in RViz or Gazebo.
