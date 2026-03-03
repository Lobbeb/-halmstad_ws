# Practical Requirements + To‑Do (UAV-side only)

This is a simplified, copy/paste runbook for *our* scope. The UGV stack (SLAM/Nav2/path planning/GNN) is handled by other students.

## 1. Scope and hard constraints

### 1.1 What we own
We implement the UAV-side perception + coordination interface, including failure handling and (optional) comm-aware hooks.

### 1.2 What we do **not** implement
We do not implement UGV SLAM/Nav2/path planning. We treat the UGV as a black box.

### 1.3 Constraint: no UGV state broadcast
The UAV side must not rely on UGV self-reported state as a dependency (no “UGV publishes its pose to UAV” assumption). Any following/support must be driven by UAV observation and the agreed interface.

## 2. UAV-side requirements

### 2.1 Sensing
Use UAV cameras for environment observation (image + camera info). Depth is optional.

### 2.2 Perception
Run a YOLO-family detector to produce detections relevant to the task (leader/UGV proxy, obstacles/clear paths, etc.).

### 2.3 Estimation output (coordination-grade)
Publish a compact estimate suitable for coordination (pose estimate + confidence/health), plus a diagnostic status stream. Avoid exposing raw detections as a public “contract” topic (keep detections internal unless explicitly needed).

### 2.4 Prediction / smoothing
Add a Kalman filter (or similar) layer on top of detections/estimates to stabilize outputs and provide short-horizon prediction during noise/dropouts.

### 2.5 Comms-aware hooks (in scope)
Expose a comm-quality signal (real later, synthetic first) and support a comm-aware leash / fallback decision (thresholds + hysteresis).

## 3. Links (what each is for)

### 3.1 Integration
- `ros_gz_bridge` docs: bridging Gazebo camera topics into ROS 2
- Gazebo ↔ ROS 2 integration docs: general simulation integration patterns

### 3.2 ROS 2 behavior
- ROS 2 QoS docs/demos: reliability/history/durability settings for your streams

### 3.3 Clearpath API (reference only)
- Clearpath ROS API overview: to know which UGV topics exist (UGV team implements their side)

Nav2/SLAM links are background only for us.

## 4. Practical To‑Do (our scope)

### 4.1 Track upstream simulation updates
Keep compatibility with the latest lab repo updates (new UAV model and control path).

### 4.2 Freeze the interface contract early
Lock what *we* publish and what we require as inputs, so the rest of the project can integrate around it.
- Inputs: UAV camera topics (+ optional depth)
- Outputs: leader estimate + status (+ optional comm metric)
- Explicitly: do not require UGV pose/odom as a dependency

### 4.3 Implement perception → estimate pipeline
1. Run YOLO inference on the UAV camera stream  
2. Select the relevant detection(s) for “leader/UGV proxy”  
3. Convert detection(s) into a usable estimate (relative pose proxy or leader estimate)  
4. Apply Kalman filtering / smoothing + short prediction  
5. Failure handling:
   - stale input handling (hold / debounce / reacquire)
   - publish status continuously
   - only publish valid estimates when requirements are satisfied

### 4.4 Generate UAV motion intent from estimate
Compute follow-point / standoff behavior and publish into the existing follower/controller interface.
Avoid “manual drive”; publish planned/commanded intent.

### 4.5 Add comm-aware behavior hooks (stub OK)
1. Define a comm-quality signal you can expose (RSSI/PRR/latency proxy)  
2. Add simple thresholds + hysteresis for comm-based leash switching  
3. Start with a synthetic ROS signal; later replace with OMNeT++ / emulation outputs

## 5. Minimal run plan (defensible results)

### 5.1 Baseline (no perception)
Follow/leash baseline without YOLO estimate driving control.

### 5.2 Perception-only (smoke mode)
YOLO + filter runs and publishes estimate/status, but does not control the follower.

### 5.3 Estimate-driven follow
Use the published leader estimate as the leader input for following.

### 5.4 Optional: comm-leash disturbance
Inject a repeatable comm-quality dip (synthetic first) and show that fallback/hysteresis prevents unstable behavior.

## 6. What “done” means (acceptance checks)

### 6.1 Interface checks
- Topics exist and have stable message types
- Status topic clearly indicates: OK / stale / disabled / reacquire / etc.

### 6.2 Runtime checks
- Runs are repeatable with the same harness
- Rosbag + metadata logs exist for each run directory
- Switching between modes does not change the logging pipeline
