# Vision System Documentation

## Overview

This directory contains comprehensive documentation for the AdaTomruk 2026 Preseason robot vision system. The system uses Limelight cameras with AprilTag detection for robot localization and pose estimation.

## Documentation Files

### 📊 [EXECUTIVE_SUMMARY.md](EXECUTIVE_SUMMARY.md)
**High-Level Overview for Leadership**

Quick reference for team leadership, mentors, and stakeholders:
- Current system status and capabilities
- Critical issues requiring attention
- Resource requirements and timeline
- Risk assessment and mitigation
- Team recommendations by role

**Start here** for a high-level understanding.

---

### 📋 [VISION_SYSTEM_ANALYSIS.md](VISION_SYSTEM_ANALYSIS.md)
**Critical Issues and Missing Components**

A comprehensive analysis identifying:
- **Critical bugs** that must be fixed before competition
- Missing components and features
- Configuration issues
- Code quality concerns
- Recommendations for immediate and long-term improvements

**Read this first** to understand what needs to be fixed.

---

### 🏗️ [VISION_SYSTEM_ARCHITECTURE.md](VISION_SYSTEM_ARCHITECTURE.md)
**System Design and Data Flow**

Detailed architectural documentation including:
- Component hierarchy and class diagrams
- Complete data flow from camera to drivetrain
- MegaTag 1 vs MegaTag 2 selection logic
- Standard deviation calculation algorithms
- Periodic execution flow
- Integration points with drivetrain

**Read this** to understand how the system works internally.

---

### 📈 [FLOWCHARTS.md](FLOWCHARTS.md)
**Visual Flowcharts and Decision Trees**

Detailed flowcharts illustrating:
- System initialization sequence
- Periodic update flow (20ms loop)
- MegaTag selection decision tree
- Vision measurement validation pipeline
- Standard deviation calculation examples
- Error handling and recovery
- Alliance tag filter setup

**Use this** to visualize the system's operation.

---

### 🔧 [TROUBLESHOOTING_GUIDE.md](TROUBLESHOOTING_GUIDE.md)
**Debugging and Problem Solving**

Step-by-step troubleshooting for common issues:
- No vision measurements
- Robot position jumps/drifts
- Initial pose not set
- Wrong alliance tag filtering
- Field layout errors
- MegaTag selection issues
- Measurement rejection problems

Includes diagnostic commands, SmartDashboard monitoring, and testing procedures.

**Use this** when things aren't working as expected.

---

### ⚙️ [CONFIGURATION_GUIDE.md](CONFIGURATION_GUIDE.md)
**Setup and Tuning**

Complete configuration reference:
- Hardware setup and mounting
- Network configuration
- Software constants tuning
- Pipeline configuration
- Field layout setup
- Standard deviation tuning
- Performance optimization

**Use this** to configure and tune the system.

---

## Quick Start

### For New Team Members

1. Start with **EXECUTIVE_SUMMARY.md** for a high-level overview
2. Read **VISION_SYSTEM_ANALYSIS.md** to understand current issues
3. Review **VISION_SYSTEM_ARCHITECTURE.md** and **FLOWCHARTS.md** to learn how it works
4. Reference **TROUBLESHOOTING_GUIDE.md** when debugging
5. Use **CONFIGURATION_GUIDE.md** for setup and tuning

### For Debugging Issues

1. Check **TROUBLESHOOTING_GUIDE.md** for your specific issue
2. Follow the diagnostic steps
3. Use SmartDashboard keys listed in the guide
4. Apply solutions as recommended

### For Tuning/Configuration

1. Start with **CONFIGURATION_GUIDE.md**
2. Use default values first
3. Follow the tuning workflow
4. Document your changes

---

## Critical Issues to Fix

Before deploying to competition robot, fix these issues:

### 🔴 Priority 1 (Critical)

1. **Missing Field Layout File** (VISION_SYSTEM_ANALYSIS.md)
   - Create `src/main/deploy/custom_field.json` OR
   - Update RobotContainer.java to use default field layout only

2. **Incorrect AprilTag Regions** (VISION_SYSTEM_ANALYSIS.md)
   - Fix Constants.java lines 15-16
   - Red and blue alliances currently have identical tag filters

3. **Timer Initialization Bug** (VISION_SYSTEM_ANALYSIS.md)
   - Move `TELEOP_TIMER.restart()` from `testInit()` to `teleopInit()`
   - Affects initial pose estimation logic

### 🟡 Priority 2 (Important)

4. **Missing Null Checks** (VISION_SYSTEM_ANALYSIS.md)
   - Add validation for fieldLayout before use
   - Prevents NullPointerException crashes

5. **Code Quality** (VISION_SYSTEM_ANALYSIS.md)
   - Remove unused imports (PublicKey)
   - Fix typo: `initalPoseSet` → `initialPoseSet`
   - Change `Void` return type to `void`

---

## System Status

### ✅ Working Features

- AprilTag detection with Limelight cameras
- MegaTag 1 and MegaTag 2 pose estimation
- Dynamic standard deviation calculation
- Alliance-based tag filtering (once fixed)
- Vision diagnostics via SmartDashboard
- Sensor fusion with swerve odometry

### ⚠️ Issues Requiring Attention

- Field layout file missing
- Tag filtering not configured correctly
- Timer bug affects initial pose
- No simulation support
- Limited test coverage
- Single tag rejection may be too strict

### ❌ Not Implemented

- PhotonVision simulation
- Comprehensive unit tests
- Vision calibration tools
- Performance metrics logging
- Measurement rejection tracking
- Machine learning detection

---

## Architecture Summary

```
┌─────────────────────────────────────────────────────┐
│                 LimelightVisionSubsystem            │
│  - Manages multiple cameras                         │
│  - Aggregates vision measurements                   │
│  - Alliance-based tag filtering                     │
└─────────────────┬───────────────────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼────────┐  ┌───────▼────────┐
│ LimelightDevice│  │ LimelightDevice│
│  (Front LL)    │  │  (Future...)   │
│  - Pose est.   │  │  - Pose est.   │
│  - StdDev calc │  │  - StdDev calc │
└───────┬────────┘  └───────┬────────┘
        │                   │
        └─────────┬─────────┘
                  │
        ┌─────────▼──────────┐
        │  LimelightHelpers  │
        │  - NetworkTables   │
        │  - JSON parsing    │
        └─────────┬──────────┘
                  │
        ┌─────────▼──────────┐
        │  Limelight Camera  │
        │  (Hardware)        │
        └────────────────────┘
```

**Data Flow:**
Camera → NetworkTables → LimelightHelpers → LimelightDevice → LimelightVisionSubsystem → CommandSwerveDrivetrain

---

## Key Concepts

### MegaTag 1 vs MegaTag 2

**MegaTag 1:**
- Estimates both position AND rotation
- Used when: 2+ tags, close (<3.75m), slow (<0.5 m/s)
- More accurate when conditions met
- Rotation stddev: 0.3

**MegaTag 2:**
- Estimates position only, uses gyro for rotation
- Used when: MegaTag 1 conditions not met
- More robust to motion and distance
- Rotation stddev: MAX_VALUE (never trust)

### Standard Deviation

Tells the Kalman filter how much to trust vision vs. odometry:
- Lower stddev → Trust vision more
- Higher stddev → Trust odometry more

Calculated based on:
- Number of tags visible
- Distance to tags
- Robot speed
- Single vs. multiple tags

### Alliance Tag Filtering

Different tags are visible to different alliances:
- **Blue Alliance:** Should see tags 1-8
- **Red Alliance:** Should see tags 9-11 (verify actual layout)

**⚠️ Currently broken** - both alliances see same tags!

Note: 2025 Reefscape field has 11 tags total (IDs 1-11). Exact alliance assignment depends on final field layout.

---

## Configuration Files

### Code Files

```
src/main/java/frc/robot/
├── Constants.java                  # Vision configuration constants
├── RobotContainer.java            # Field layout and subsystem setup
├── subsystems/
│   ├── CommandSwerveDrivetrain.java  # Vision integration
│   └── vision/
│       ├── LimelightVisionSubsystem.java  # Main subsystem
│       └── LimelightDevice.java           # Camera wrapper
└── util/
    └── Structures.java            # VisionMeasurement record
```

### Deploy Files

```
src/main/deploy/
└── custom_field.json              # ⚠️ MISSING - needs to be created
```

### Documentation Files

```
docs/vision/
├── README.md                          # This file - documentation index
├── EXECUTIVE_SUMMARY.md              # High-level overview for leadership
├── VISION_SYSTEM_ANALYSIS.md         # Critical issues and analysis
├── VISION_SYSTEM_ARCHITECTURE.md     # System design and flow
├── FLOWCHARTS.md                     # Visual diagrams and decision trees
├── TROUBLESHOOTING_GUIDE.md          # Debugging guide
└── CONFIGURATION_GUIDE.md            # Setup and tuning guide
```

---

## Diagnostic Tools

### SmartDashboard Keys

Monitor these values during operation:

```
Essential:
- Initial Pose Set?
- VisionDiagnostics/limelight-FrontLL/count
- VisionDiagnostics/limelight-FrontLL/distance
- VisionDiagnostics/limelight-FrontLL/speed
- VisionDiagnostics/limelight-FrontLL/method
- VisionDiagnostics/limelight-FrontLL/stddev

Timing:
- Since Last Megatag1 Reading
- Since Last Megatag2 Reading
```

### Limelight Web Interface

Access at: `http://limelight-frontll.local:5801`

Pages:
- **Input**: Live camera feed
- **Pipeline**: Detection settings
- **Settings**: Configuration
- **Diagnostics**: Performance metrics

---

## Testing Checklist

### Pre-Match

- [ ] Clean camera lens
- [ ] Verify network connection (ping limelight)
- [ ] Check camera feed shows tags
- [ ] Verify correct alliance selected
- [ ] Enable robot and wait for initial pose
- [ ] Verify Field2d shows correct position
- [ ] Test smooth tracking during slow movement

### Field Testing

- [ ] Test stationary at multiple positions
- [ ] Test slow movement (MegaTag1)
- [ ] Test fast movement (MegaTag2)
- [ ] Test single tag scenarios
- [ ] Test far from tags (>4m)
- [ ] Test spinning (angular velocity limit)

---

## Resources

### External Documentation

- **Limelight Docs**: https://docs.limelightvision.io/
- **WPILib Vision**: https://docs.wpilib.org/en/stable/docs/software/vision-processing/
- **AprilTag Info**: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/
- **PathPlanner**: https://pathplanner.dev/

### Web Interfaces

- Limelight: http://limelight-frontll.local:5801
- RoboRIO: http://roborio-TEAM-frc.local
- Radio: http://10.TE.AM.1

### Reference Implementation

The `Referance_Project/` folder contains example documentation from another team:
- Vision system overview
- Limelight subsystem examples
- Target detector interface

---

## Contributing

When making changes to the vision system:

1. **Test thoroughly** - Use field testing checklist
2. **Document changes** - Update relevant documentation files
3. **Tune incrementally** - Change one parameter at a time
4. **Log results** - Keep notes on tuning attempts
5. **Update this README** - Keep status current

### Documentation Standards

- Use clear headings and sections
- Include code examples where helpful
- Add diagrams for complex flows
- Keep troubleshooting practical
- Update the status section

---

## Version History

### Version 1.0 (2026-01-04)

**Initial Documentation Release**

Created comprehensive documentation:
- System analysis identifying critical issues
- Architectural documentation with diagrams
- Troubleshooting guide
- Configuration and tuning guide

**Known Issues:**
- Missing field layout file
- Incorrect AprilTag regions
- Timer initialization bug
- Missing null checks
- No simulation support
- Limited test coverage

---

## Support

For issues or questions:

1. Check relevant documentation file
2. Review console output for errors
3. Check SmartDashboard diagnostics
4. Review architecture documentation
5. Contact team programming lead

---

## License

This documentation is provided for the AdaTomruk FRC team (Team TBD).
Code follows the WPILib BSD license.

---

*Last Updated: 2026-01-04*  
*Documentation Version: 1.0*  
*For 2026 Preseason Robot*
