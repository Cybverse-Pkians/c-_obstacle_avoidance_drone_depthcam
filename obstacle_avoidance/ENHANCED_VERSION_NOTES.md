# Obstacle Avoidance System — ENHANCED VERSION

## Summary of Changes

This document outlines all the improvements made to the obstacle avoidance system as per your requirements.

---

## 1. ✅ SIMPLE RIGHT AVOIDANCE (No Smart Logic)

**What Changed:**
- Removed complex `chooseBestDirection()` method that selected LEFT or RIGHT based on multiple factors
- Implemented **always prefer RIGHT** strategy
- Removed unused methods: `bandAvg()`, `corridorWidth()`, gradient analysis, slant detection

**Code Changes:**
- In `processAvoidance()` CRUISING state: Direction is now always `int dir = 1;` (RIGHT)
- In nested obstacle detection: Nested obstacles also always choose RIGHT
- Clean, simple decision tree

**Benefits:**
- No mistakes due to complex logic
- Predictable behavior
- Faster execution

---

## 2. ✅ OBSTACLE MEASUREMENT & RELATIVE COORDINATES

**What Changed:**
- Added `logMeasurements()` method that runs every 500ms
- Extracts obstacle dimensions (L×W×H) in real-time using voxel data
- Calculates **relative coordinates** from drone position to obstacle center
- Prints to console with obstacle ID, dimensions, voxel count, and confidence

**Code Changes:**
- New timer: `meas_tmr_ = create_wall_timer(500ms, ...logMeasurements...)`
- Calculates:
  ```cpp
  float rel_x = m.center_x - dx_;
  float rel_y = m.center_y - dy_;  
  float rel_z = m.center_z - alt_;
  ```
- Output format: `📊 OBS <id> | Rel: X=<m> Y=<m> Z=<m> | Dims: L=<m> W=<m> H=<m>`

**Log Output Example:**
```
📊 OBS 0 | Rel: X=2.45m Y=-1.23m Z=-0.15m | Dims: L=0.80m W=1.50m H=2.10m | Voxels: 145 | Conf: 95.2% | Phase: WALL_FOLLOW
```

---

## 3. ✅ CSV LOGGING WITH OBSTACLE DATA

**What Changed:**
- Created new `CSVLogger` class that logs every obstacle measurement
- File saved to: `/home/avengers/obstacle_data.csv`
- Logs measurements at each avoidance phase AND when obstacle is completed

**CSV Columns:**
```
timestamp_ms, obstacle_id, rel_x_m, rel_y_m, rel_z_m,
length_m, breadth_m, height_m,
center_x_m, center_y_m, center_z_m,
confidence, voxel_count, avoidance_direction, phase
```

**CSV Log Entries:**
- Continuously logged during avoidance phases (SHIFT, WALL_FOLLOW, POST_WALL, VOX_EXTRA, EXTRA_5M, RETURN)
- Final entry with phase="COMPLETED" when obstacle is successfully avoided
- Allows post-mission analysis of all obstacles encountered

**Example CSV Row:**
```
1234567890,0,2.45,-1.23,-0.15,0.80,1.50,2.10,12.45,15.77,1.85,0.95,145,RIGHT,"WALL_FOLLOW"
```

---

## 4. ✅ 5 METER EXTRA OFFSET AFTER AVOIDANCE

**What Changed:**
- Updated tuning parameter: `kExtraStraight = 5.0f` (was 3.0m before)
- Renamed state from `EXTRA_STRAIGHT` to `EXTRA_STRAIGHT_5M` for clarity
- Added new dedicated avoidance phase

**State Machine:**
```
POST_WALL_FORWARD → VOXEL_EXTRA_FWD → EXTRA_STRAIGHT_5M (NEW!) → RETURNING
                     (ensures wall gone)  (5m safety buffer)  
```

**Behavior:**
- After voxel memory confirms wall is gone (~4.5m forward in POST_WALL + voxel check)
- Drone flies exactly 5.0 meters straight forward as extra offset buffer
- This ensures maximum clearance from any residual obstacles
- Then proceeds to lateral RETURNING phase

**Code Changes:**
```cpp
case AvoidanceMode::EXTRA_STRAIGHT_5M:
{
  cmd.linear.x = 0.f;
  cmd.linear.y = kFwdSpd;
  ctx.extra_straight_dist += kFwdSpd * dt;
  
  bool done = (ctx.extra_straight_dist >= kExtraStraight);  // 5.0m
  if (done) {
    ctx.mode = AvoidanceMode::RETURNING;
  }
}
```

---

## 5. ✅ WALL MEMORY & COLLISION PREVENTION

**What Changed:**
- Enhanced voxel persistence with `is_wall_marker` flag
- Wall markers decay 2× slower than regular obstacle voxels
- Wall markers persist even after main avoidance stack is cleared
- Voxels tagged as walls influence behavior even after returning to cruise

**Problem Fixed:**
- **Before**: After returning and popping from stack, drone would immediately go LEFT if it detected a wall, causing collision
- **After**: Voxelization remembers wall location. If wall still exists nearby, drone recognizes it and avoids re-entering

**Voxel Decay Times:**
- Regular voxels: 60 seconds base decay (1.5× for high-confidence hits)
- Wall markers: 60 seconds × 2.0 = 120 seconds base decay

**Code Changes:**
```cpp
struct Voxel {
  bool is_wall_marker{false};  // NEW
};

float confidence(...) {
  if (it->second.is_wall_marker) eff_dt *= 2.0f;  // decay slower
}

void cleanup(...) {
  float max_age = it->second.is_wall_marker ? dt_*5.0f : dt_*3.5f;
}
```

---

## 6. ✅ REAL-TIME DATA PRINTING DURING AVOIDANCE

**What Changed:**
- Every 500ms, obstacles are analyzed and printed with full details
- Output includes current phase, relative coordinates, dimensions
- Confidence level and voxel count for monitoring accuracy

**Output Format:**
```
📊 OBS 0 | Rel: X=2.45m Y=-1.23m Z=-0.15m | Dims: L=0.80m W=1.50m H=2.10m | Voxels: 145 | Conf: 95.2% | Phase: WALL_FOLLOW
```

**Phase Strings:**
- `SHIFT`: Lateral avoidance phase
- `WALL_FOLLOW`: Following wall while going forward
- `POST_WALL`: Post-wall forward distance
- `VOX_EXTRA`: Voxel-gated extra forward
- `EXTRA_5M`: 5-meter safety offset
- `RETURN`: Returning to original path

---

## 7. ✅ ERROR CHECKS & WALL COLLISION FIX

**Triple-Source Guard System:**

The code now uses three sources to prevent collision during return:

1. **Camera Obstacle Detection** → if front_min < kTriggerDist
2. **Voxel Side Wall Memory** → if sideVoxelOccupied() 
3. **Return Path Occupancy** → if returnPathClear()

All three must be clear before returning. If ANY is blocked:
- Drone holds position or moves slowly forward
- Prevents rushing back into walls/obstacles
- Debounces for kClearFrames (10 frames) before confirming clear

**Specific Wall Handling:**
```cpp
// Guard 2a: wall on avoidance side (camera)
if (!cam_ok) {
  cmd.linear.y = kFwdSpd*0.7f;
  ctx.clear_cnt = 0;  // reset debounce
}

// Guard 2b: wall on avoidance side (voxels!)
if (!vox_side_ok) {
  cmd.linear.y = kFwdSpd*0.6f;
  ctx.clear_cnt = 0;  // reset debounce
}
```

---

## Configuration Parameters

All tuning parameters are in one place (private section of class):

```cpp
static constexpr float kShiftDist         = 2.5f;   // lateral shift distance
static constexpr float kPostWallDist      = 4.5f;   // distance after wall gone
static constexpr float kVoxelExtraOffset  = 5.0f;   // max voxel extra distance
static constexpr float kExtraStraight     = 5.0f;   // ← 5M EXTRA OFFSET (NEW!)
static constexpr float kTriggerDist       = 3.5f;   // obstacle detection range

static constexpr float kVoxelWallDist     = 3.0f;   // synthetic wall distance
static constexpr int   kNoWallFrames      = 20;     // debounce frames
static constexpr int   kClearFrames       = 10;     // return debounce
```

Easy to adjust these values for tuning behavior.

---

## CSV Output Location

**File Path:** `/home/avengers/obstacle_data.csv`

To view the data after a mission:
```bash
cat /home/avengers/obstacle_data.csv
```

Or in a spreadsheet:
```bash
libreoffice /home/avengers/obstacle_data.csv
```

---

## State Machine Flow

```
CRUISING
   ↓
[Obstacle Detected]
   ↓
LATERAL_SHIFT (shift 2.5m RIGHT)
   ↓
WALL_FOLLOWING (go forward while wall exists)
   ↓
POST_WALL_FORWARD (go 4.5m forward after wall gone)
   ↓
VOXEL_EXTRA_FWD (go forward, gated by voxel memory)
   ↓
EXTRA_STRAIGHT_5M (fly 5m straight - NEW!)
   ↓
RETURNING (lateral + forward to return to original path)
   ↓
CRUISING (resume mission)
```

---

## Key Features Summary

✅ **Always RIGHT**: No complex logic, predictable avoidance  
✅ **Real-Time Dimensions**: L×W×H printed every 500ms  
✅ **Relative Coordinates**: Obstacle position relative to drone  
✅ **CSV Logging**: All measurements saved to file  
✅ **5M Safety Offset**: Extra buffer after obstacle  
✅ **Wall Memory**: Voxels prevent re-collision  
✅ **Triple Guard**: Camera + voxels + path check  
✅ **Error Resistant**: Debounced decisions  

---

## Compilation

To compile this enhanced version:

```bash
cd /home/avengers/ros2_ws
colcon build --packages-select obstacle_avoidance
```

Then source and run:
```bash
source install/setup.bash
ros2 launch obstacle_avoidance avoidance.launch.py
```

---

## Testing Checklist

- [ ] Drone takes RIGHT turn on obstacle detection
- [ ] CSV file is created in `/home/avengers/obstacle_data.csv`
- [ ] Obstacle dimensions printed to console
- [ ] Relative coordinates are accurate
- [ ] Flies 5 meters extra after obstacle
- [ ] No collision when returning to path
- [ ] Voxel memory prevents re-entry to cleared walls

---

End of Document
