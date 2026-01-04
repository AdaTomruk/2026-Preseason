# Vision System Configuration Guide

## Overview

This guide covers the configuration and tuning of the vision system for optimal performance.

## Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Software Configuration](#software-configuration)
3. [Tuning Parameters](#tuning-parameters)
4. [Pipeline Configuration](#pipeline-configuration)
5. [Field Layout Setup](#field-layout-setup)

---

## Hardware Setup

### Limelight Installation

#### Physical Mounting

**Requirements:**
- Rigid mount to robot frame
- No vibration or flex
- Clear view of AprilTags
- Protected from impacts

**Optimal Placement:**
```
Height: 0.5m - 1.0m from ground
Angle: 15-30 degrees upward tilt
Position: Front center of robot
FOV: Should cover scoring zones and tags
```

**Current Configuration:**
```java
// In Constants.VisionConstants
new LimelightConstants(
    "FrontLL",           // Camera name
    LIMELIGHT_3G,        // Model (90 FPS, better range)
    11,                  // Static IP: 10.TE.AM.11
    LOCALIZATION         // Primary use: robot localization
)
```

#### Network Configuration

**Static IP Setup:**

1. Connect to Limelight web interface:
   - URL: `http://limelight-frontll.local:5801`
   - Or: `http://10.TE.AM.11:5801` (after configuration)

2. Navigate to **Settings** → **Networking**

3. Configure:
   ```
   Team Number: [Your team number]
   Static IP: 11  (results in 10.TE.AM.11)
   Hostname: limelight-frontll
   ```

4. Verify connection:
   ```bash
   ping 10.TE.AM.11
   ping limelight-frontll.local
   ```

#### Power Distribution

**Important:**
- Connect to dedicated PDP/PDH port
- Use 12V Passive PoE injector OR direct power
- Ensure stable voltage (11-13V)
- Monitor for brownouts during high power use

---

## Software Configuration

### Constants.java Configuration

#### VisionConstants

```java
public static final class VisionConstants {
    // Camera Configuration
    public static final LimelightConstants kLimelights[] = {
        new LimelightConstants(
            "FrontLL",              // Name (used for NetworkTables)
            LimelightModel.LIMELIGHT_3G,  // Model type
            11,                     // IP: 10.TE.AM.11
            LimelightRole.LOCALIZATION    // Primary purpose
        )
        // Add more cameras here as needed
    };
    
    // Distance Thresholds (meters)
    public static final double kMaxDistance = 8.0;
    public static final double kMaxDistanceForMegaTag1 = 3.75;
    
    // Speed Thresholds (m/s)
    public static final double kMaxSpeedForMegaTag1 = 0.5;
    public static final double kMaxAngularSpeed = 720; // deg/s
    
    // Timing
    public static final Time newMegaTag1ReadingThreshold = Seconds.of(10);
    
    // Features
    public static final boolean kVisionDiagnostics = true;
    
    // ⚠️ FIX REQUIRED: AprilTag Regions
    public static final class AprilTagRegions {
        // Current (INCORRECT - both alliances same):
        public static final int[] RED_ALLIANCE = { 1, 2, 3, 4, 5, 6 };
        public static final int[] BLUE_ALLIANCE = { 1, 2, 3, 4, 5, 6 };
        
        // Should be (for 2025 Reefscape - verify exact layout):
        // Example with 11 tags total:
        // public static final int[] RED_ALLIANCE = { 9, 10, 11 };
        // public static final int[] BLUE_ALLIANCE = { 1, 2, 3, 4, 5, 6, 7, 8 };
    }
}
```

#### Standard Deviation Constants

```java
public static final class StdDevConstants {
    public static final class MegaTag1 {
        public static final double kInitialValue = 0.3;
        public static final double kTagCountReward = 0.15;
        public static final double kAverageDistancePunishment = 0.1;
        public static final double kRobotSpeedPunishment = 0.15;
        public static final double kSingleTagPunishment = 0.3;
    }
    
    public static final class MegaTag2 {
        public static final double kInitialValue = 0.2;
        public static final double kAverageDistancePunishment = 0.075;
        public static final double kRobotSpeedPunishment = 0.25;
        public static final double kMultipleTagsBonus = 0.05;
    }
}
```

### RobotContainer.java Configuration

#### Field Layout Loading

**Current (needs custom_field.json):**
```java
static {
    try {
        String path = Filesystem.getDeployDirectory()
            .toPath()
            .resolve("custom_field.json")
            .toString();
        fieldLayout = new AprilTagFieldLayout(path);
    } catch (IOException e) {
        DriverStation.reportError("Failed to load custom field", 
            e.getStackTrace());
        try {
            fieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2025ReefscapeWelded);
        } catch (Exception ex) {
            fieldLayout = null;
        }
    }
}
```

**Recommended (use default field):**
```java
static {
    try {
        fieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2025ReefscapeWelded);
    } catch (IOException e) {
        DriverStation.reportError("Failed to load field layout", 
            e.getStackTrace());
        fieldLayout = null;
    }
}
```

---

## Tuning Parameters

### When to Tune

Tune vision parameters when:
- Robot localization is unstable
- Too many measurements rejected
- Position jumps/drifts significantly
- Different field conditions (lighting, distance)

### MegaTag1 vs MegaTag2 Threshold

**Goal:** Balance between getting heading from vision (MT1) vs. trusting gyro (MT2)

**MegaTag1 Conditions:**
```java
tagCount >= 2
avgTagDist < kMaxDistanceForMegaTag1  // default: 3.75m
robotSpeed < kMaxSpeedForMegaTag1      // default: 0.5 m/s
```

**Tuning Scenarios:**

| Scenario | Adjustment | Reason |
|----------|------------|--------|
| MT1 rarely used | Increase thresholds | Too restrictive |
| Position jumps with MT1 | Decrease thresholds | Bad heading estimates |
| Good tags, still MT2 | Increase speed limit | Robot moves too fast |
| Far tags work well | Increase distance | Field specific |

**Example Adjustments:**

```java
// More conservative (use MT1 less often)
kMaxDistanceForMegaTag1 = 3.0;   // was 3.75
kMaxSpeedForMegaTag1 = 0.4;      // was 0.5

// More aggressive (use MT1 more often)
kMaxDistanceForMegaTag1 = 4.5;   // was 3.75
kMaxSpeedForMegaTag1 = 0.75;     // was 0.5
```

### Standard Deviation Tuning

**Purpose:** Tell the Kalman filter how much to trust vision vs. odometry

**Lower stddev = Trust vision more**
**Higher stddev = Trust odometry more**

#### MegaTag1 Tuning

**Formula:**
```
stddev = kInitialValue
       - min(tagCount, 4) × kTagCountReward
       + avgTagDist × kAverageDistancePunishment
       + robotSpeed × kRobotSpeedPunishment
       + (singleTag ? kSingleTagPunishment : 0)

stddev = max(stddev, 0.05)  // minimum threshold
```

**Tuning Guide:**

```java
// Trust vision more (smaller stddev)
kInitialValue = 0.2;              // was 0.3
kTagCountReward = 0.20;           // was 0.15
kAverageDistancePunishment = 0.08; // was 0.1
kRobotSpeedPunishment = 0.10;     // was 0.15
kSingleTagPunishment = 0.25;      // was 0.3

// Trust vision less (larger stddev)
kInitialValue = 0.4;              // was 0.3
kTagCountReward = 0.10;           // was 0.15
kAverageDistancePunishment = 0.15; // was 0.1
kRobotSpeedPunishment = 0.20;     // was 0.15
kSingleTagPunishment = 0.4;       // was 0.3
```

#### MegaTag2 Tuning

**Formula:**
```
stddev = kInitialValue
       - (tagCount > 1 ? kMultipleTagsBonus : 0)
       + avgTagDist × kAverageDistancePunishment
       + robotSpeed × kRobotSpeedPunishment

stddev = max(stddev, 0.05)  // minimum threshold
```

**Typical Adjustments:**

```java
// Trust more (use when MT2 is very reliable)
kInitialValue = 0.15;             // was 0.2
kMultipleTagsBonus = 0.075;       // was 0.05
kAverageDistancePunishment = 0.05; // was 0.075
kRobotSpeedPunishment = 0.20;     // was 0.25

// Trust less (use if MT2 causes drift)
kInitialValue = 0.3;              // was 0.2
kMultipleTagsBonus = 0.03;        // was 0.05
kAverageDistancePunishment = 0.1;  // was 0.075
kRobotSpeedPunishment = 0.3;      // was 0.25
```

### Single Tag Filtering

**Location:** LimelightDevice.java line 154

**Current:**
```java
if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
    return Optional.empty();
}
```

**Tuning:**

```java
// More strict (fewer single tag measurements)
if (singleTag.ambiguity > 0.6 || singleTag.distToCamera > 4) {

// More lenient (more single tag measurements)
if (singleTag.ambiguity > 0.85 || singleTag.distToCamera > 6) {
```

**Ambiguity Meaning:**
- 0.0 = Perfect detection, no ambiguity
- 0.5 = Some ambiguity, usually okay
- 0.7 = High ambiguity, questionable
- 1.0+ = Very ambiguous, likely wrong

---

## Pipeline Configuration

### Limelight Web Interface

Access: `http://limelight-frontll.local:5801`

### AprilTag Pipeline Settings

**Pipeline Tab:**

```
Pipeline Type: AprilTag (Fiducials)

Detection Settings:
├─ Tag Family: 36h11 (FRC standard)
├─ Decimation: Normal (best accuracy)
│   └─ Use 2x for faster FPS if needed
├─ Blur: 0.0 (unless noisy image)
├─ Threads: 4 (use all cores)
├─ Quad Decimate: 1.0
└─ Refine Edges: Enabled (better accuracy)

Filtering:
├─ Min Area: 0.01% (adjust based on tag distance)
├─ Sort Mode: Largest (prioritize big/close tags)
└─ Max Targets: 8 (track many tags simultaneously)

Output:
├─ Targeting: AprilTag
├─ Stream: Standard (for debugging)
└─ Snapshot Mode: Off
```

### Camera Settings

**Camera Tab:**

```
Exposure:
├─ Mode: Auto (or manual 5-20ms)
├─ Gain: Auto (or manual 0-40)
└─ Note: Adjust if tags hard to detect

Color:
├─ White Balance: Auto
├─ Black Level Offset: 0
└─ Saturation: Default

Image:
├─ Resolution: 960×720 (default for LL3/3G)
├─ Orientation: Normal
└─ Crop: None
```

### 3D Settings

**3D Tab:**

```
Robot Transform:
├─ Forward: Distance from robot center to camera (meters)
├─ Right: Lateral offset (positive = right)
├─ Up: Height of camera above ground (meters)
├─ Roll: Camera roll angle (degrees)
├─ Pitch: Camera tilt angle (15-30° up typical)
└─ Yaw: Camera rotation (0° = forward)

Example for front-mounted camera:
Forward: 0.25m
Right: 0.0m
Up: 0.6m
Roll: 0°
Pitch: 20° (angled up)
Yaw: 0°
```

**Important:** These offsets are relative to robot center, not camera mounting point!

### LED Configuration

**Settings Tab → LED:**

```
LED Mode: Pipeline (controlled by active pipeline)

Default LED:
├─ During operation: Off (to save power)
├─ For debugging: On (to see in bright light)
└─ Brightness: 50-100% (adjust for conditions)

Note: LEDs can help in very dark conditions but drain power
```

---

## Field Layout Setup

### Using Default Field Layout

**Recommended for 2025 Reefscape:**

```java
// In RobotContainer.java
fieldLayout = AprilTagFieldLayout.loadField(
    AprilTagFields.k2025ReefscapeWelded);
```

**Pros:**
- Always up to date with WPILib
- No need to manage custom file
- Automatically correct for official field

**Cons:**
- Cannot customize for practice field differences

### Using Custom Field Layout

**When to use:**
- Practice field has different tag positions
- Testing with non-standard layout
- Need to add custom tags

**Setup:**

1. **Create custom_field.json:**

```json
{
  "field": {
    "length": 16.54175,
    "width": 8.21055
  },
  "tags": [
    {
      "ID": 1,
      "pose": {
        "translation": {
          "x": 1.5,
          "y": 0.5,
          "z": 1.35
        },
        "rotation": {
          "quaternion": {
            "W": 1.0,
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
          }
        }
      }
    },
    // ... more tags ...
  ]
}
```

2. **Save to:** `src/main/deploy/custom_field.json`

3. **Deploy:** File will be copied to robot during deployment

4. **Verify:** Check console for "Failed to load" messages

### Tag Coordinate System

**FRC Field Coordinate System:**
```
Origin: Blue alliance wall, right corner (looking from driver station)
+X: Toward red alliance wall
+Y: Toward left (from blue driver station perspective)
+Z: Up

Rotation: 0° = facing red alliance wall
```

**Tag Pose Format:**
```
Translation: (x, y, z) in meters
Rotation: Quaternion (W, X, Y, Z)
  - Most tags face outward from field perimeter
  - Use online calculator to convert from euler angles
```

---

## Validation and Testing

### After Configuration

**Checklist:**

1. ✅ **Network connectivity**
   ```bash
   ping limelight-frontll.local
   # Should respond in <5ms
   ```

2. ✅ **Camera feed visible**
   - Open web interface
   - See live camera feed
   - AprilTags highlighted with green boxes

3. ✅ **Pipeline active**
   - Correct pipeline selected
   - "tv" shows 1 when tags visible

4. ✅ **Robot code detects camera**
   - Deploy code
   - Check SmartDashboard for vision values
   - Should see non-zero readings

5. ✅ **Pose estimation working**
   - Enable robot
   - "Initial Pose Set?" becomes true
   - Field2d shows robot position

### Field Testing Procedure

**Step 1: Stationary Tests**
```
1. Place robot at known position
2. Point at AprilTags (2+ visible)
3. Enable robot
4. Wait for "Initial Pose Set?"
5. Verify pose on Field2d matches expected
6. Repeat at different positions
```

**Step 2: Movement Tests**
```
1. Start at known position
2. Drive slowly in straight line (0.5 m/s)
3. Observe pose tracking
4. Should smoothly follow robot
5. Check for jumps or drift
```

**Step 3: Method Switching**
```
1. Drive slowly with 2+ tags visible
   → Should use MegaTag1
   → Check "method" in SmartDashboard
2. Drive faster (> 0.5 m/s)
   → Should switch to MegaTag2
3. Move far from tags (> 3.75m)
   → Should switch to MegaTag2
```

**Step 4: Edge Cases**
```
1. Single tag visible
   → Check if measurement accepted
   → Verify ambiguity in diagnostics
2. No tags visible
   → Should use odometry only
   → No vision measurements
3. Spinning fast
   → MegaTag2 should reject
   → Check angular velocity limit
```

### Tuning Workflow

```
1. Start with default values
2. Collect data during field testing
3. Identify issues:
   - Too many rejections?
   - Position jumping?
   - Drift over time?
4. Adjust ONE parameter at a time
5. Re-test and compare
6. Document changes and results
7. Repeat until satisfied
```

---

## Common Configurations

### Conservative (Trust odometry more)

**When to use:** New field, uncertain conditions, testing

```java
kMaxDistanceForMegaTag1 = 3.0;
kMaxSpeedForMegaTag1 = 0.4;
kMaxDistance = 6.0;

MegaTag1.kInitialValue = 0.4;
MegaTag2.kInitialValue = 0.3;
```

### Aggressive (Trust vision more)

**When to use:** Well-tuned system, good tags, reliable field

```java
kMaxDistanceForMegaTag1 = 4.5;
kMaxSpeedForMegaTag1 = 0.75;
kMaxDistance = 10.0;

MegaTag1.kInitialValue = 0.2;
MegaTag2.kInitialValue = 0.15;
```

### Balanced (Recommended starting point)

**When to use:** Normal operation, competition

```java
kMaxDistanceForMegaTag1 = 3.75;  // default
kMaxSpeedForMegaTag1 = 0.5;      // default
kMaxDistance = 8.0;              // default

// Use default StdDev constants
```

---

## Maintenance

### Regular Checks

**Before each match:**
- [ ] Clean camera lens
- [ ] Verify network connection
- [ ] Check mounting tightness
- [ ] Test camera feed
- [ ] Verify tag detection

**Weekly:**
- [ ] Review diagnostic data
- [ ] Check for loose connections
- [ ] Update firmware if available
- [ ] Backup configuration

**Seasonal:**
- [ ] Full recalibration
- [ ] Review and tune parameters
- [ ] Update documentation
- [ ] Test with current year's field

### Backup Configuration

**Save settings:**
1. Limelight web interface → Settings → Export
2. Save Constants.java values
3. Document custom field layout
4. Record tuning parameters

**Restore:**
1. Import settings to Limelight
2. Restore Constants.java
3. Copy field layout file
4. Re-deploy code

---

## Reference

### Default Values Summary

```java
// Distance thresholds
kMaxDistance = 8.0 m
kMaxDistanceForMegaTag1 = 3.75 m

// Speed thresholds  
kMaxSpeedForMegaTag1 = 0.5 m/s
kMaxAngularSpeed = 720 deg/s

// MegaTag1 StdDev
kInitialValue = 0.3
kTagCountReward = 0.15
kAverageDistancePunishment = 0.1
kRobotSpeedPunishment = 0.15
kSingleTagPunishment = 0.3

// MegaTag2 StdDev
kInitialValue = 0.2
kAverageDistancePunishment = 0.075
kRobotSpeedPunishment = 0.25
kMultipleTagsBonus = 0.05

// Single tag filtering
ambiguity threshold = 0.7
distance threshold = 5.0 m

// Timing
newMegaTag1ReadingThreshold = 10 seconds
```

---

*Last Updated: 2026-01-04*  
*Configuration Version: 1.0*
