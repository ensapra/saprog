Bearing-aligned URDF (geometry-based correction)

The circular upper bearing of each grey leg is aligned to the circular cover hole using 2D circular-feature fits on the supplied STL meshes. The measured CAD-plane corrections are approximately 15.7 mm in Y. Joint axes are along the CAD mesh Z axis. Both visual and collision geometry are updated.

The supplied STEP file and screenshots were used for interpretation. The blue highlighted SolidWorks component was not introduced.

IMPORTANT: The crank and leg are separate revolute joints in this URDF. The closed-loop crank-to-leg bearing is not enforced by standard URDF; a solver/closed-loop constraint is still required to drive the leg from crank motion. Bearing fits are approximate and should be checked in the viewer.

Right crank limits: [-0.57, 0] radians. The original STEP experimental URDF is preserved as URDF_previous.urdf.
