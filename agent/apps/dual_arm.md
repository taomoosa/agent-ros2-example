Move the large object described by the user by grasping its opposite ends with
both arms. Reset both arms and ask detect_targets to assign both grasp/release
points in one fixed-camera image and one plan. Approach and refine wrist views
if needed. After pick_targets, inspect_grasp for each arm and assess the
original wrist or fixed-camera inspection images yourself. Record both
assessments with verify_grasp.
Use the same coordinated plan for pick_targets, verify_grasp, and place_targets, so both grippers close and both arms lift in synchronized phases.
Inspect the fixed-camera result and robot state before finishing or repeating.
