STEP-informed experimental URDF variant

This is NOT a validated closed-loop simulation.

The uploaded BaseAssembly.STEP confirms the CAD components LegCover, LegA, LegB, JointMotor and BoltBearing. STEP AP214 does not supply the SolidWorks motion mates needed to solve the closed-loop kinematics.

This variant replaces the prior rigid crank-to-lower-leg attachment with an independent revolute leg hinge at the original exporter-defined cover-relative frame. It preserves all link mesh positions at joint angle zero. The red crank remains independent and its right-side limits remain [-0.57, 0] radians.

A standard URDF tree cannot enforce the second bolt-bearing closure. Moving the crank alone will NOT correctly move the leg; use a simulator-specific closed-loop constraint or a custom controller solving the linkage geometry. The cover hinge location has NOT been validated against exact CAD bearing axes.

Changes:
front_right_leg: reparented to front_right_12 at the original SolidWorks-exported hinge frame; initial mesh placement preserved; independently actuated.
back_right_leg: reparented to back_right_22 at the original SolidWorks-exported hinge frame; initial mesh placement preserved; independently actuated.
front_left_leg: reparented to front_left_32 at the original SolidWorks-exported hinge frame; initial mesh placement preserved; independently actuated.
back_left_leg: reparented to back_left_42 at the original SolidWorks-exported hinge frame; initial mesh placement preserved; independently actuated.

Original last-working URDF is included as URDF/urdf/URDF_previous.urdf for rollback.
