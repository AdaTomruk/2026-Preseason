# Vision System Troubleshooting Guide

## Quick Diagnostics Checklist

Before diving into detailed troubleshooting, run through this quick checklist:

- [ ] Limelight has power (green LED on)
- [ ] Limelight is connected to network (check LED pattern)
- [ ] Robot code is deployed and running
- [ ] Vision diagnostics enabled (`kVisionDiagnostics = true`)
- [ ] SmartDashboard/Shuffleboard showing vision data
- [ ] AprilTags visible in camera view
- [ ] Correct alliance selected in Driver Station

## Common Issues and Solutions

### Issue 1: No Vision Measurements

**Symptoms:**
- "Initial Pose Set?" remains false
- No vision data in SmartDashboard
- Robot localization relies only on wheel odometry

**Diagnostic Steps:**

1. **Check Limelight Connection**
   ```
   SmartDashboard → VisionDiagnostics → limelight-FrontLL
   Check if values are updating (not stuck at -1)
   ```

2. **Verify Camera Feed**
   - Open web browser: `http://limelight-frontll.local:5801`
   - Check if AprilTags are visible
   - Verify green bounding boxes around detected tags

3. **Check NetworkTables**
   ```
   OutlineViewer → NetworkTables → limelight-FrontLL
   Verify "tv" (has valid target) = 1
   Check "botpose_wpiblue" is updating
   ```

**Common Causes:**

| Cause | Check | Solution |
|-------|-------|----------|
| Camera obscured | Visual inspection | Clean lens, check mount |
| Network disconnected | LED pattern | Check ethernet cable |
| Wrong pipeline | Limelight web UI | Select AprilTag pipeline |
| Tags out of range | Distance > 8m | Move closer to tags |
| Too dark/bright | Camera exposure | Adjust in Limelight settings |

---

### Issue 2: Robot Position Jumps/Drifts

**Symptoms:**
- Robot pose on Field2d widget jumps around
- Odometry unstable
- Path following erratic

**Diagnostic Steps:**

1. **Check Standard Deviations**
   ```
   SmartDashboard → VisionDiagnostics → limelight-FrontLL → stddev
   
   Expected ranges:
   - MegaTag1, multiple tags, close: 0.05 - 0.30
   - MegaTag2, multiple tags: 0.15 - 0.50
   - Single tag: 0.60 - 0.90 (or rejected)
   ```

2. **Verify Measurement Quality**
   ```
   Check in SmartDashboard:
   - count (tag count): Should be ≥ 2 for best results
   - distance: Should be < 4m for MegaTag1, < 8m for MegaTag2
   - speed: Should be < 0.5 m/s for MegaTag1
   ```

3. **Check for Outliers**
   - Compare vision pose vs. odometry pose
   - Large differences (>1m) indicate bad measurement
   - Check "method" - ensure switching between MT1/MT2 appropriately

**Common Causes:**

| Cause | Symptom | Solution |
|-------|---------|----------|
| Single tag detection | High stddev, jumping | Move to see multiple tags |
| Fast movement | Frequent MT2, higher stddev | Slow down during localization |
| Far from tags | High distance value | Move closer to tags |
| Tag ambiguity | Single tag rejected | Approach tag from different angle |
| Incorrect field layout | Consistent offset | Verify AprilTag positions match field |

**Fixes:**

```java
// Increase trust requirement (more conservative)
kMaxDistanceForMegaTag1 = 3.0;  // was 3.75
kMaxSpeedForMegaTag1 = 0.4;     // was 0.5

// Or adjust std dev calculation (require more tags)
transStdDev -= min(tagCount, 4) * 0.20;  // was 0.15
```

---

### Issue 3: Initial Pose Never Set

**Symptoms:**
- "Initial Pose Set?" remains false
- Auto doesn't start at correct position
- Robot doesn't know where it is

**Diagnostic Steps:**

1. **Check Timer Status**
   ```
   ⚠️ KNOWN BUG: TELEOP_TIMER started in testInit instead of teleopInit
   
   Check code in Robot.java:
   - Auto timer should restart in autonomousInit() ✓
   - Teleop timer should restart in teleopInit() ✗ (currently in testInit)
   ```

2. **Verify BEFORE_MATCH Logic**
   ```java
   // In LimelightDevice.java line 85
   final boolean BEFORE_MATCH = 
       !Robot.AUTO_TIMER.hasElapsed(0.01) && 
       !Robot.TELEOP_TIMER.hasElapsed(0.01);
   ```
   
   If timers never start, BEFORE_MATCH is always true, requiring high-quality measurements.

3. **Check Measurement Quality Requirements**
   - Need 2+ tags visible
   - Distance < 3.75m
   - Speed < 0.5 m/s
   - All conditions must be met simultaneously

**Solutions:**

**Fix 1: Correct Timer Bug** (CRITICAL)
```java
// In Robot.java, move TELEOP_TIMER.restart() from testInit to:
@Override
public void teleopInit() {
    if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
    }
    TELEOP_TIMER.restart();  // ← Add this line
}
```

**Fix 2: Relax Initial Requirements** (if needed)
```java
// In VisionConstants
kMaxDistanceForMegaTag1 = 4.5;  // More forgiving
kMaxSpeedForMegaTag1 = 0.75;
```

**Fix 3: Manual Initial Pose**
```java
// In RobotContainer.autonomousInit() or teleopInit()
if (!visionSubsystem.isInitialPoseSet()) {
    // Set known starting position
    drivetrain.seedFieldRelative(new Pose2d(1.5, 5.5, new Rotation2d()));
}
```

---

### Issue 4: Wrong Alliance Tag Filtering

**Symptoms:**
- Robot sees opponent alliance tags
- Localization confused near opponent side
- Unexpected pose estimates

**Diagnostic Steps:**

1. **Check Alliance Selection**
   ```
   Driver Station → Setup → Team Station
   Verify correct alliance (Red/Blue) selected
   ```

2. **Verify Tag Filter Applied**
   ```
   OutlineViewer → limelight-FrontLL → fiducial_id_filters_set
   Should show array of tag IDs
   ```

3. **Check Tag IDs Detected**
   ```
   SmartDashboard → VisionDiagnostics → visible tags
   All detected tag IDs should be from your alliance
   ```

**Known Issue:**

```java
// In Constants.java - BOTH ALLIANCES HAVE SAME TAGS! ❌
public static final int[] RED_ALLIANCE = { 1, 2, 3, 4, 5, 6 };
public static final int[] BLUE_ALLIANCE = { 1, 2, 3, 4, 5, 6 };
```

**Solution:**

```java
// Fix alliance tag regions (2025 Reefscape field)
public static final int[] RED_ALLIANCE = { 9, 10, 11, 12, 13, 14, 15, 16 };
public static final int[] BLUE_ALLIANCE = { 1, 2, 3, 4, 5, 6, 7, 8 };
```

---

### Issue 5: Field Layout Not Loading

**Symptoms:**
- Error in console: "Failed to load custom field"
- NullPointerException in getVisibleTagPoses()
- Field2d shows no tag positions

**Diagnostic Steps:**

1. **Check Console Output**
   ```
   Look for: "Failed to load custom field: ..."
   ```

2. **Verify File Exists**
   ```bash
   ls src/main/deploy/custom_field.json
   ```

3. **Check Fallback Loaded**
   ```
   If custom fails, should load k2025ReefscapeWelded
   If that fails too, fieldLayout will be null
   ```

**Solutions:**

**Option 1: Create Custom Field Layout**
```bash
# Download from game manual or create from scratch
# Save to: src/main/deploy/custom_field.json
```

**Option 2: Remove Custom Field Reference**
```java
// In RobotContainer.java, replace custom loading with:
try {
    fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded
    );
} catch (IOException e) {
    DriverStation.reportError("Failed to load field layout", 
        e.getStackTrace());
    fieldLayout = null;  // Will crash if used!
}
```

**Option 3: Add Null Safety**
```java
// In LimelightVisionSubsystem.java, line 144:
Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

// Change to:
if (fieldLayout == null) {
    Logger.getLogger(this.getClass().getName())
        .warning("Field layout is null, cannot get tag poses");
    return new ArrayList<>();
}
Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);
```

---

### Issue 6: MegaTag Selection Not Working

**Symptoms:**
- Always using MegaTag2 (or always MegaTag1)
- "method" in SmartDashboard doesn't change
- Rotation estimation not working (or too sensitive)

**Diagnostic Steps:**

1. **Check Conditions**
   ```
   SmartDashboard values:
   - count ≥ 2
   - distance < 3.75
   - speed < 0.5
   All must be true for MegaTag1
   ```

2. **Verify Override Flag**
   ```java
   SmartDashboard.putBoolean("MT1 Override", 
       LimelightVisionSubsystem.getMegaTag1Override());
   
   Should be false in normal operation
   ```

3. **Check Angular Velocity (for MT2)**
   ```
   If angular velocity > 720 deg/s, MT2 rejects measurements
   ```

**Common Scenarios:**

| Scenario | Expected Method | Why |
|----------|----------------|-----|
| Stationary, 2 tags close | MegaTag1 | All conditions met |
| Moving fast | MegaTag2 | Speed too high for MT1 |
| Far from tags | MegaTag2 | Distance too far for MT1 |
| Single tag | MegaTag2 | Need 2+ for MT1 |
| Spinning fast | Rejected | Angular velocity limit |

**Testing:**

```java
// Add debugging in LimelightDevice.getVisionMeasurement()
System.out.println(String.format(
    "Vision: tags=%d, dist=%.2f, speed=%.2f, method=%s",
    poseEstimate.tagCount,
    poseEstimate.avgTagDist,
    robotSpeed,
    method
));
```

---

### Issue 7: Vision Measurements Rejected

**Symptoms:**
- Very few or no measurements accepted
- High rejection rate
- Robot doesn't use vision

**Diagnostic Steps:**

1. **Add Rejection Logging**
   ```java
   // In LimelightDevice, add logging for each rejection:
   
   if (!LimelightHelpers.validPoseEstimate(poseEstimate)) {
       System.out.println("Rejected: invalid pose estimate");
       return Optional.empty();
   }
   
   if (!BEFORE_MATCH && poseEstimate.avgTagDist > kMaxDistance) {
       System.out.println("Rejected: distance " + 
           poseEstimate.avgTagDist + " > " + kMaxDistance);
       return Optional.empty();
   }
   
   // Add similar logging for other rejection points
   ```

2. **Check Rejection Reasons**
   | Rejection Point | Reason | Threshold |
   |----------------|---------|-----------|
   | Invalid estimate | No tags detected | tv == 0 |
   | Distance (before match) | > 8m | kMaxDistance |
   | Single tag ambiguity | > 0.7 | Hard-coded |
   | Single tag distance | > 5m | Hard-coded |
   | MT2 angular velocity | > 720 deg/s | kMaxAngularSpeed |
   | MT2 distance | > 8m | Hard-coded |

3. **Adjust Thresholds** (if too strict)

**Solutions:**

```java
// Less conservative thresholds:
kMaxDistance = 10;  // was 8
kMaxDistanceForMegaTag1 = 4.5;  // was 3.75
kMaxSpeedForMegaTag1 = 0.75;  // was 0.5

// In LimelightDevice line 154:
if (singleTag.ambiguity > 0.85 || singleTag.distToCamera > 6) {
    // was 0.7 and 5
```

---

## Diagnostic Commands

### SmartDashboard Keys to Monitor

```
Essential:
- Initial Pose Set?
- VisionDiagnostics/Want New MT1 Reading?
- VisionDiagnostics/limelight-FrontLL/count
- VisionDiagnostics/limelight-FrontLL/distance
- VisionDiagnostics/limelight-FrontLL/speed
- VisionDiagnostics/limelight-FrontLL/method
- VisionDiagnostics/limelight-FrontLL/stddev

Timing:
- Since Last Megatag1 Reading
- Since Last Megatag2 Reading

Robot State:
- Match Time
- Alliance (from DriverStation)
```

### Limelight Web Interface

Access at: `http://limelight-frontll.local:5801`

**Useful Pages:**
- **Input**: Live camera feed with detections
- **Pipeline**: Configure AprilTag detection
- **Settings**: Network, LED, camera settings
- **Diagnostics**: FPS, latency, temperature

**Key Settings to Check:**
```
Pipeline:
- Type: AprilTag (Fiducials)
- Tag Family: 36h11 (FRC 2025)
- Decimation: Normal (not aggressive)

Camera:
- Exposure: Auto or 5-20ms
- Black Level: 0
- Gain: Auto or 0-40
```

### NetworkTables Inspection

Using OutlineViewer or AdvantageScope:

```
/limelight-FrontLL/
├─ tx: Target X offset (degrees)
├─ ty: Target Y offset (degrees)
├─ tv: Has valid target (0 or 1)
├─ botpose_wpiblue: [x, y, z, roll, pitch, yaw, latency]
├─ botpose_orb_wpiblue_megatag2: [x, y, z, roll, pitch, yaw, latency]
└─ json: Full JSON dump (advanced debugging)
```

---

## Performance Optimization

### Reducing Latency

1. **Pipeline Decimation**
   - Normal: Best accuracy, ~11ms latency
   - 2X: Faster, ~8ms latency, slight accuracy loss
   - 3X: Fastest, ~6ms latency, more accuracy loss

2. **Camera Resolution**
   - Default: 1280×960 or 960×720
   - Lower resolution = faster processing

3. **Multiple Cameras**
   - Different angles provide redundancy
   - Can use best measurement from all cameras

### Improving Accuracy

1. **Standard Deviation Tuning**
   - Lower values = more trust in vision
   - Higher values = more trust in odometry
   - Tune based on field testing

2. **Camera Placement**
   - Multiple cameras at different heights/angles
   - Cover all areas of field
   - Minimize occlusions

3. **Lighting Conditions**
   - Test in various lighting (bright, dark)
   - Adjust exposure accordingly
   - Use consistent LED ring brightness

---

## Advanced Debugging

### Enable Verbose Logging

```java
// In LimelightDevice and LimelightVisionSubsystem
private static final Logger LOGGER = 
    Logger.getLogger(LimelightDevice.class.getName());

// Add throughout code:
LOGGER.info("Vision measurement: " + measurement);
LOGGER.fine("Tag count: " + tagCount);  // Detailed debug
LOGGER.warning("Rejected: " + reason);
```

### Log to File

```java
// In Robot.java or RobotContainer
try {
    FileHandler fh = new FileHandler("/home/lvuser/vision.log");
    Logger.getLogger("frc.robot.subsystems.vision").addHandler(fh);
} catch (IOException e) {
    e.printStackTrace();
}
```

### Data Logging

```java
// Log all measurements to CSV
PrintWriter writer = new PrintWriter(
    new FileWriter("/home/lvuser/vision_data.csv", true));
writer.printf("%f,%f,%f,%d,%f,%f,%s\n",
    timestamp, pose.getX(), pose.getY(),
    tagCount, distance, speed, method);
writer.close();
```

---

## Testing Procedures

### Pre-Match Checklist

1. ✅ Verify Limelight power and network
2. ✅ Check camera feed shows tags clearly
3. ✅ Verify correct alliance selected
4. ✅ Position robot at known location
5. ✅ Enable robot and wait for "Initial Pose Set?" = true
6. ✅ Verify Field2d shows robot in correct position
7. ✅ Drive slowly and watch pose tracking
8. ✅ Check for smooth transitions between MegaTag1/2

### Field Testing

1. **Stationary Positions**
   - Place robot at known positions
   - Verify vision matches expected pose
   - Test all areas of field

2. **Moving Tests**
   - Drive slowly in straight lines
   - Make gentle turns
   - Fast movements (test rejection)

3. **Edge Cases**
   - Single tag visible
   - Far from all tags
   - Rapidly spinning
   - Tags at extreme angles

### Simulator Testing

Currently not implemented. Future enhancement:
- PhotonVision simulation
- Simulated AprilTag field
- Test logic without hardware

---

## Contact and Support

### Resources

- **Limelight Docs**: https://docs.limelightvision.io/
- **WPILib Docs**: https://docs.wpilib.org/
- **FRC AprilTag Info**: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/
- **Team Documentation**: See Referance_Project folder

### Common Web Interfaces

- Limelight: http://limelight-frontll.local:5801
- RoboRIO: http://roborio-TEAM-frc.local (or roborio-4915-frc.local)
- Radio: http://10.TE.AM.1 (e.g., 10.49.15.1)

### Getting Help

1. Check this troubleshooting guide
2. Review vision system documentation
3. Check console output for errors
4. Verify SmartDashboard values
5. Test with known good configuration
6. Ask team mentor or lead programmer

---

## Appendix: Error Messages

### Common Errors and Meanings

| Error Message | Meaning | Solution |
|--------------|---------|----------|
| "Failed to load custom field" | custom_field.json missing | Create file or fix path |
| "Tag ID X not found in field layout" | Invalid tag ID or wrong field | Check field layout matches tags |
| NullPointerException in getTagPose | fieldLayout is null | Fix field layout loading |
| "Limelight not responding" | Network issue | Check ethernet, IP address |
| "No valid pose estimate" | No tags detected | Point camera at tags |

### LED Patterns

Limelight LED colors indicate status:

| Pattern | Meaning |
|---------|---------|
| Solid green | Normal operation |
| Blinking green | No target detected |
| Orange/yellow | Booting up |
| Red | Error condition |
| No light | No power |

---

*Last Updated: 2026-01-04*  
*For Issues or Updates: Contact programming team lead*
