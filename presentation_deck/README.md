# Vision Arm Teleop Presentation Deck

Artifacts in this folder:

- `vision_arm_teleop_prototype_review.js`: editable PptxGenJS source
- `vision_arm_teleop_prototype_review.pptx`: generated PowerPoint deck

Build command:

```powershell
cd presentation_deck
node vision_arm_teleop_prototype_review.js
```

Notes:

- The deck is based on the current source tree and checked-in calibration profile.
- It treats the Python teleop loop plus the Uno/ESP32 firmware as the implemented system.
- It explicitly does **not** describe the runtime as inverse kinematics or pinch-driven gripper control, because those are not the active paths in the checked-in source.
