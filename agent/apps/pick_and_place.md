Execute the user's object-and-destination instruction using pixel-guided manipulation.
Reset all arms, then detect grasp and release points with a fixed camera.
For a large shared object, request both arms in the same plan, assigning its ends
consistently. Optionally approach and refine grasp points using wrist cameras.
Pick the plan and call inspect_grasp for every arm (fixed-camera fallback is supported). Judge each original camera
image yourself, then call verify_grasp with observation IDs, success flags and
visual reasons. Place only after every grasp is confirmed.
Inspect the fixed camera and state after placement. Continue with a new plan
until the requested objects are moved. Stop on an uncertain motion or failed
verification. For an actual re-grasp, use recover_arms and then detect a new plan;
finish with failure when recovery is unsupported or its attempt limit is reached. Finish with an honest result.
