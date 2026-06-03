  What the Quest is sending you
  
  Looking at the HTML: it uses
  requestReferenceSpace("local"). In WebXR, "local" space
  means:

  ▎ Origin = where your head was at the exact moment you 
  ▎ pressed Start. Y-up, -Z forward.

  Every wrist position and every joint xyz in that
  75-float array is an absolute coordinate in that space.
  So right_wrist.position = (0.35, 1.1, -0.4) means
  "35cm right, 110cm up, 40cm in front of where my head
  was when I clicked the button."

  ---
  What Isaac Lab has

  Two arms floating at (-0.5, 0, 0) and (0.5, 0, 0) in
  simulation world space. No humanoid body. No torso. No
  shared physical reference with the user's body.

  ---
  Why coordinate frame flipping alone is NOT enough
  
  You'd need two things for that to work:
  1. A shared origin — some point that means the same
  thing in both spaces 
  2. A shared scale/workspace — your arm's reach in
  meatspace maps to something meaningful in sim
  
  Neither exists here. The Quest "local" origin is your
  head position at session start. Isaac Lab's origin is
  wherever the simulation puts world zero. There's no
  geometric relationship between the two. Even if you
  perfectly convert axes, (0.35, 1.1, -0.4) in Quest
  space maps to a completely arbitrary point relative to
  the sim arms.

  ---
  What actually works: relative / delta control

  This is how virtually every teleoperation system
  handles this exact problem. You never map absolute
  positions — you map changes from a reference pose.

  The idea:
  1. Zero/calibration step — when the user is ready, you
  snapshot the current Quest wrist poses as
  wrist_home_quest. You also define a neutral
  configuration for the sim arms (e.g., arms hanging at
  rest = some known joint config).
  2. Every frame — compute delta = current_wrist_quest - 
  wrist_home_quest. This delta is a displacement in Quest
  space.
  3. Convert axes (WebXR Y-up -Z-forward → Isaac Lab
  Z-up, etc.) — now the delta is in sim-compatible
  coordinates.
  4. Add to neutral end-effector position — ee_target = 
  ee_neutral + delta. Feed that target to IK.
  
  Now the origin problem disappears entirely. You don't
  care where the Quest origin is. You only care about how
  much the hand moved from where it started. And since
  the arms are fixed in sim, the neutral ee position is
  just wherever they are at rest — well-defined.

  The axis conversion still matters, but only for mapping
  directions correctly — "moving your hand forward in
  the real world should move the sim arm in the right
  direction." That's a fixed 3×3 rotation matrix between
  WebXR frame and Isaac Lab frame, applied to the delta
  vector.

  ---
  Summary of what needs to happen for arm IK

  ┌────────────────────┬─────────────────────────────┐
  │      Problem       │          Solution           │
  ├────────────────────┼─────────────────────────────┤
  │                    │ Delta control — zero at     │
  │ Origin mismatch    │ session start, drive with   │
  │                    │ offsets                     │
  ├────────────────────┼─────────────────────────────┤
  │ Axis convention    │ Fixed rotation matrix on    │
  │ (WebXR Y-up vs     │ the delta vector            │
  │ Isaac Z-up)        │                             │
  ├────────────────────┼─────────────────────────────┤
  │                    │ Optionally a scalar gain    │
  │ Workspace scale    │ (if 10cm real → 10cm sim is │
  │                    │  too slow/fast)             │
  ├────────────────────┼─────────────────────────────┤
  │                    │ Isaac Lab's differential    │
  │ IK solver          │ IK, fed the target ee pose  │
  │                    │ each frame                  │
  └────────────────────┴─────────────────────────────┘

  The finger curl approach sidesteps all of this because
  you're computing joint angles from relative bone 
  positions within the hand — all in the same local
  frame. The origin never matters. That's exactly why
  it's easy. Arm control is harder because you're trying
  to control something's position in world space, not
  just its internal configuration.

  Does this match your mental model? If so, we can talk
  about what the calibration/zeroing step should look
  like and how to define the "neutral" arm pose in sim.