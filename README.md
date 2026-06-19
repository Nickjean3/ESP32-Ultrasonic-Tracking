# ESP32 Ultrasonic Radar Tracker

Real-time object detection and tracking using an ESP32, HC-SR04 ultrasonic sensor, and servo motor. The servo sweeps a 180° arc, detects objects, and locks onto them to follow their movement.

Built with Arduino IDE using the ESP32Servo library.

---

## Sketches

### v1 — State Machine Tracker
A two-state FSM (SWEEP / TRACK) that steps the servo across the arc degree by degree, switches to track mode on detection, then nudges left or right based on which side reads a closer distance. Simple and functional but locks onto the edge of objects rather than their centre, and has no sense of how large the error is — every correction is the same fixed step.

### v2 — PID Controller
Replaces the binary direction decision with a PID loop. The difference in distance between left and right samples becomes the error signal, and the servo response scales proportionally to how far off-centre the object is. Smoother tracking but still limited by the sensor's wide beam geometry and prone to false triggering on background objects within range.

### v3 — Background Subtraction + Centroid Tracking
On startup, the sensor maps the empty environment across all 180°. During sweep, a reading only counts as a detection if it is significantly closer than the stored background at that angle — eliminating false triggers from static objects entirely. On detection, a centroid scan finds the true angular centre of the object before handing off to PID. A rolling background update keeps the map current over time without recalibration.

---

## Tuning

| Parameter | Effect |
|---|---|
| `INTRUSION_THRESH_CM` | Raise if false triggering, lower if missing objects |
| `RESCAN_INTERVAL_MS` | Lower = more responsive, higher = more stable |
| `Kp / Kd` | Kp raises tracking speed, Kd reduces overshoot |
| `RECOVERY_DEGREES` | How far back to search when object is lost |
