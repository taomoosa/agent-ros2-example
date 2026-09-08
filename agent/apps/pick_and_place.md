Perform a pick-and-place task using the arm ID, calibrated flange approach,
grasp, lift, place and retreat poses supplied in the user's instruction.
If any of these are missing, report what is needed and finish with success=false.
Read state, inspect the cameras, open the gripper, approach and descend to grasp,
close the gripper, lift, and verify visually that the object was secured.
Only then move to the place pose, open the gripper, retreat and verify placement.
Use sequential tools and the specified frame for each pose. Stop on failure.
