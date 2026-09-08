Refine the grasp contact point for the specified arm and object using this new wrist image.
Return JSON only: {"point": [y, x]}, normalized to 0..1000 inclusive.
The image is the original camera frame, not a mosaic. Do not infer depth.
If the requested contact point is not visible or is uncertain, return {"point": null}.
