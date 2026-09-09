Locate grasp and release contact points for the task in the supplied original camera image.
Return JSON only: {"targets": [{"arm_id": "...", "grasp": [y, x], "release": [y, x]}]}.
Points use [y, x] normalized to 0..1000, inclusive. Include exactly the requested arms.
For a shared large object, assign distinct grasp points at its ends and consistent release points.
Select visible contact points compatible with the supplied projection context, including the destination support surface.
Depth-mode points require valid measured depth on the server; plane-mode points must lie on the calibrated plane.
Do not invent depth, Cartesian coordinates, occluded points, or missing objects.
If any required point is uncertain or invisible, return {"targets": []}.
