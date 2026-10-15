# Webots R2025a Physics, Modeling & Sensor Rules Reference
**Source:** Official Cyberbotics Webots R2025a User Guide (`https://cyberbotics.com/doc/guide/index`) & Project Findings (`FINDINGS.md`)

> **Status:** drafted from an external review, not yet checked line by line
> against the Webots R2025a docs. In particular 1.1 (unjointed child Solids
> detach) is contradicted by Phase 8 in FINDINGS.md, where unjointed sphere
> casters measurably improved delivery; 3.1 (single-value coulombFriction
> zero-fill) and 5.1 (Lidar needs a GPU context) are unverified. Treat these
> as hypotheses until a run confirms them.

---

## 1. Solid Hierarchy, Joints & Mass Rules (`physics.md`, `jointparameters.md`)
1. **No Rigid Weld Joint Exists:** Webots only supports `HingeJoint`, `Hinge2Joint`, `BallJoint`, `SliderJoint`, and `Track`. Every child `Solid` with a `Physics` node that is not inside a Joint behaves as a **completely detached, independent rigid body** and will fall off the robot at `t = 0`.
2. **Merging Static Parts:** All non-moving structural components (chassis deck, bin walls, mast, ramps, retention ledges) MUST be placed as visual `Transform`/`Shape` children inside a single parent `Solid` (`BODY_CHASSIS`), and their collision primitives must be mirrored inside that same `Solid`'s `boundingObject Group`.
3. **Joint Parent Mass (`m0` Rule):** The anchor reaction mass `m0` of any Joint is taken from the **closest upper `Solid` with a `Physics` node**. Never place wheel or intake joints directly under a token-mass `Robot` node (`mass 0.05`); always nest all `HingeJoint`s and `Track`s inside the main `12 kg` `BODY_CHASSIS` `Solid`.
4. **Joint Anchor Frame Rule:** `HingeJointParameters.anchor` and `BallJointParameters.anchor` are defined in the **parent `Solid`'s coordinate frame**, NOT the child `endPoint`'s frame. The `anchor` vector MUST exactly match the `translation` vector of the `endPoint Solid` (e.g., if `endPoint` is at `-0.10 0.185 0.09`, `anchor` must be `-0.10 0.185 0.09`, never `0 0 0`).
5. **Explicit Mass vs. Density:** In a `Physics` node, `density` defaults to `1000 kg/m³` and silently overrides `mass`. To specify an exact `mass` (e.g., to keep front intake components light so the Center of Mass does not shift), you MUST set `density -1`.
6. **Preventing Reaction-Torque Flipping:** In velocity control mode (`setPosition(inf)`, `setVelocity(w)`), Webots motors apply up to `maxTorque` in a single `basicTimeStep` to close velocity errors. High `maxTorque` on low-inertia rotors (wheels or intake fans) transfers an equal and opposite reaction torque to the chassis. Keep `maxTorque` physically proportional (`<= 4.0 N·m` for drive wheels, `<= 1.0 N·m` for intake fans) and ramp velocities in software.

---

## 2. Bounding Objects & Collision Geometry Rules (`bounding-objects.md`)
1. **No Nested Pose/Transform in `boundingObject`:** A `Group` inside a `boundingObject` may contain `Transform` (or `Pose`) nodes, but **a `Transform` inside a `boundingObject` CANNOT contain another `Transform`/`Pose`**. Compound rotations (e.g., yaw + pitch on a funnel guide or conveyor) must be pre-multiplied into a single axis-angle `rotation x y z angle` on a single `Transform`.
2. **Primitive Orientation (`Cylinder` & `Capsule`):** Webots `Cylinder` and `Capsule` primitives are always aligned along their local **Z-axis** and have **no `rotation` or `translation` fields**. To orient a wheel or roller along the Y-axis, wrap it in `Transform { rotation 1 0 0 1.5708 children [ Cylinder { ... } ] }`.
3. **Floor Clearance at `t = 0`:** Static chassis collision boxes (`boundingObject`) must never touch or penetrate `z <= 0` at spawn, or the ODE solver's initial penetration-recovery impulse will launch the robot upward. Keep static floor plates at `z >= 0.002 m`.
4. **Visual vs. Collision Radius Match:** Adding visual tire lugs or paddle vanes that extend beyond the `boundingObject` radius causes visual ground clipping (if `r_visual > r_collision` on wheels) or phantom non-collisions (if `r_visual > r_collision` on paddles). For a bladed fan wheel, either match the `Cylinder` bounding radius to the blade tip or use a `Group` of `Box` primitives in the `boundingObject` for true blade-tooth contact.

---

## 3. Contact Properties & Friction Rules (`contact-properties.md`)
1. **`coulombFriction` 4-Value Rule:** `ContactProperties.coulombFriction` is an `MFFloat` taking **1, 2, or 4 values**, but in Webots R2025a asymmetric/directional modes, supplying a single value `[ 1.0 ]` when secondary friction directions are active can zero-fill remaining axes (`[ 1.0 0.0 0.0 0.0 ]`), eliminating longitudinal traction. Always write **four explicit values**: `coulombFriction [ mu mu mu mu ]`.
2. **`rollingFriction` 3-Value Rule:** `rollingFriction` is an `SFVec3f` (three floats, **no square brackets**): `rollingFriction r1 r2 r3`. Supplying 4 values or brackets raises a parser error.
3. **No Asymmetric Friction on `Sphere`:** ODE cannot compute a principal friction direction on a `Sphere` collider and emits warnings if asymmetric friction is defined. Use low isotropic friction (`[ 0.03 0.03 0.03 0.03 ]`) for spherical ball casters, or use a `HingeJoint` + `Cylinder` for rolling castor wheels.

---

## 4. Conveyor Belts & Tracks (`conveyor-belts.md`, `track.md`)
1. **Use `Track` Instead of Hinged Chain Links:** To simulate a conveyor belt in Webots without dozens of colliding rigid links, use a `Track` node inside the parent `Solid`.
2. **`Track` Actuation:** A `Track` node contains a `LinearMotor` inside its `device [ ... ]` field (controlled via `motor.setPosition(float('inf'))` and `motor.setVelocity(v_belt)` in m/s). ODE simulates the belt motion by applying `surfaceVelocity` at every contact point on the `Track`'s `boundingObject`.
3. **Intake Speed Ratio:** For a side-pinching conveyor belt to capture a rolling ball without bulldozing it, the linear belt speed $v_{\text{belt}}$ must exceed the robot's forward approach speed $v_{\text{robot}}$:
   $$v_{\text{belt}} \ge 1.5 \times v_{\text{robot}}$$

---

## 5. Sensors & Perception Rules (`sensors.md`, `lidar.md`, `distance-sensor.md`)
1. **`Lidar` Requires GPU Context:** Webots `Lidar` nodes are ray-cast via the OpenGL GPU pipeline. Running headless without an active display/GPU context hangs or breaks lidar initialization.
2. **Sensor Mounting Frame & Standoff:** Every range reading from a `Lidar` is measured from the sensor's local `translation` origin, NOT the robot's front bumper. Transform all lidar hit points into the **chassis frame (`BODY_CHASSIS`)** once at the point of measurement (`x_chassis = x_sensor + r * cos(theta)`, `y_chassis = y_sensor + r * sin(theta)`), and never subtract longitudinal standoff offsets from lateral/flank beams.
3. **Buried Lidar Beams (`r < minRange`):** When a bumper lidar presses directly into a wall, beams return `< minRange` or `NaN`. Never map "all frontal beams filtered out" to `inf` (open space); distinguish true `inf` (no obstacle within `maxRange`) from buried/occluded rays (`0.0 m` contact).
4. **`DistanceSensor` vs. `Lidar` Fields:** `DistanceSensor` uses `aperture`, `numberOfRays`, and `lookupTable` (NOT `fieldOfView` or `maxRange`).
