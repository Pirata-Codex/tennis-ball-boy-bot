# Webots Design Rules Based on Findings & Documentation

## General Simulation Notes (Derived from Cyberbotics Docs)
* **References:** Comprehensive guides and documentation can be found on the official Cyberbotics website.
* **Core Setup:** All physics simulations should use `--batch --mode=fast --no-rendering` for reliable, repeatable runs.
* **World Setup:** Use clear definitions and ensure consistent `contactMaterial` properties across all worlds (reference: `tennis_court.wbt` contact properties).

## Core Physics & Geometry Rules (From Findings)

### 1. Body Structure and Rigid Bodies (§5.1, §5.12)
* **No Weld Rule:** All static geometry that should function as a single unit **must** be merged into a single `Solid` body. Do not use nested child `Solid`s for fixed assemblies.
* **Bounding Objects:** Ensure all physical parts (including complex shapes like fan blades) are explicitly defined in the `boundingObject`'s `children` list, along with their collision geometries, to ensure ODE physics engine registers the full shape.
* **Part Definition:** Complex mechanics (skids, wheels, arms) must be attached using `HingeJoint`s or `BallJoint`s with an `anchor` explicitly set to the correct endpoint translation, preventing bodies from orbiting the chassis origin.

### 2. Forces and Materials (§5.7, §5.13, §5.15)
* **Motor Torque and Acceleration:** Be cautious of reaction torques; small, continuous accelerations are often safer than sharp, high-torque commands, as transient torque can tip the robot. Use software rate limiting for movement/pickup phases.
* **Friction:** Collision material coefficient lists (`coulombFriction`) must match the required fixed-point data type (e.g., 4 coefficients for longitudinal/lateral). Providing an incomplete list causes Webots to zero-fill, eliminating necessary grip.
* **Mass and Physics:** For complex assemblies, explicitly define the physics properties (`density`, `mass`) inside the primary `Solid` body to ensure the physics engine uses the correct inertial mass (`m0`).

### 3. Perception, State, and State Machine Reliability (§5.18, §5.19, §5.20)
* **Sensor Frame of Reference:** All range data used for decision-making (e.g., distance to wall) must be expressed in a single, correct frame (ideally the closest sensor's origin). Corrections (like `LIDAR_STANDOFF`) must be applied *once* at the point of measurement, not inconsistently across various call sites.
* **Blind Spots:** Verify sensor placement (e.g., Lidar/Bumper) such that they cover the functional area of the robot (e.g., bumper must be at the front).
* **Diagnostic Readings:** Interpreting one type of sensor reading (e.g., `gps_pose_h` as a heading) based on its raw output is dangerous; always verify its true physical meaning (e.g., GPS bearing of the error).

## Intake Redesign (Feature Specific)
* **Tracking vs. Rolling:** For continuous side-pinching intake, use the `Track` node (`LinearMotor`) with a smooth, continuous `Box` collider over discrete cylindrical rollers to eliminate interleaving and dead zones.
* **Active Capture:** The final capture mechanism requires a dedicated fan/roller (`intakeRoller`) with a correctly inverted axis (`axis 0 -1 0`) and active motion to pull the ball through the channel and over the retention ledge.
* **Collision Geometry:** Always model the physical vanes/blades of a complex mechanism as distinct geometry children within a `boundingObject Group` to ensure they interact with ODE physics.