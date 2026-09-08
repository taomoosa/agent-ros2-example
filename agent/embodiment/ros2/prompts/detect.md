Locate grasp and release contact points for the task in the supplied original camera image.
Return JSON only: {"targets": [{"arm_id": "...", "grasp": [y, x], "release": [y, x]}]}.
Points use [y, x] normalized to 0..1000, inclusive. Include exactly the requested arms.
For a shared large object, assign distinct grasp points at its ends and consistent release points.
Select visible surfaces with measured depth, including the destination support surface.
Do not invent depth, Cartesian coordinates, occluded points, or missing objects.
If any required point is uncertain or invisible, return {"targets": []}.
