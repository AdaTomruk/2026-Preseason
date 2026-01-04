# TargetDetectorInterface

## Overview

`TargetDetectorInterface` defines a contract for target detection systems, primarily used for game piece detection (like notes/corals). It provides a simple interface for getting the closest visible target.

## File Location

```
src/main/java/com/spartronics4915/frc2025/subsystems/vision/TargetDetectorInterface.java
```

## Interface Definition

```java
public interface TargetDetectorInterface {
    
    public static record Detection(
        double tx,          // Horizontal angle to target (degrees)
        double ty,          // Vertical angle to target (degrees)  
        double estimatedDistance  // Estimated distance to target (meters)
    ) {};

    public Optional<Detection> getClosestVisibleTarget();
}
```

## Detection Record

The `Detection` record contains information about a detected target:

| Field | Type | Description |
|-------|------|-------------|
| `tx` | `double` | Horizontal angle from camera center to target (degrees) |
| `ty` | `double` | Vertical angle from camera center to target (degrees) |
| `estimatedDistance` | `double` | Estimated distance to the target (meters) |

```
                    DETECTION ANGLES
                    
        ◄────────── tx (horizontal) ──────────►
                         │
                    ┌────┼────┐
                    │    │    │
                    │    ▼    │  ▲
                    │  Target │  │
                    │         │  ty (vertical)
                    └─────────┘  │
                         │       ▼
                    ┌─────────┐
                    │ Camera  │
                    │  View   │
                    └─────────┘
```

## Implementations

### NoteLocatorSim

A simulation implementation for detecting notes (game pieces) in the simulator.

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                        NoteLocatorSim                                          │
│                implements TargetDetectorInterface                              │
├────────────────────────────────────────────────────────────────────────────────┤
│  - swerveDrive: SwerveSubsystem                                                │
│  - noteLocations: ArrayList<Translation2d>  (static, predefined positions)     │
├────────────────────────────────────────────────────────────────────────────────┤
│  + NoteLocatorSim(swerveDrive)                                                 │
│  + getClosestVisibleTarget(): Optional<Detection>                              │
│  + getTx(): OptionalDouble                                                     │
│  + getTy(): double                                                             │
└────────────────────────────────────────────────────────────────────────────────┘
```

## Note Detection Logic (Simulation)

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                    getClosestVisibleTarget() FLOW                              │
└────────────────────────────────────────────────────────────────────────────────┘

        ┌─────────────────────────────────────────────────────┐
        │           For each note location:                   │
        └─────────────────────┬───────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────────────────┐
        │  Calculate vector from robot to note                │
        │  botNoteVec = noteLoc - robotPosition               │
        └─────────────────────┬───────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────────────────┐
        │  Calculate horizontal angle from robot heading      │
        │  viewCenterNoteAngle = robotRotation - noteAngle    │
        └─────────────────────┬───────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────────────────┐
        │  Is note within camera FOV?  (±27°)                  │
        └─────────────────────┬───────────────────────────────┘
                              │
                ┌─────────────┴─────────────┐
               NO                          YES
                │                           │
                ▼                           ▼
        ┌───────────────┐   ┌─────────────────────────────────┐
        │    Skip       │   │  Calculate vertical angle       │
        │               │   │  (based on robot height 0.3m)   │
        └───────────────┘   └─────────────────┬───────────────┘
                                              │
                                              ▼
                            ┌─────────────────────────────────┐
                            │  Is vertical angle valid?       │
                            │  (> -40° threshold)             │
                            └─────────────────┬───────────────┘
                                              │
                                ┌─────────────┴─────────────┐
                               NO                          YES
                                │                           │
                                ▼                           ▼
                        ┌───────────────┐   ┌─────────────────────────────────┐
                        │    Skip       │   │  Is this the closest note?      │
                        └───────────────┘   │  If yes, save as best detection │
                                            └─────────────────────────────────┘
```

## Visibility Constraints

| Constraint | Value | Description |
|------------|-------|-------------|
| `MAX_DEGREES` | ±27° | Horizontal field of view |
| `ROBOT_HEIGHT` | 0.3 m | Camera height for vertical angle calc |
| `VERT_BOT_VISIBILITY_THRESH` | -40° | Minimum vertical angle threshold |

## Predefined Note Locations (Simulation)

```java
final static ArrayList<Translation2d> noteLocations = new ArrayList<>(List.of(
    new Translation2d(2.9, 7),
    new Translation2d(2.9, 5. 5),
    new Translation2d(2.9, 4.1)
));
```

```
        FIELD VIEW (partial)
        
        Y ▲
          │
        7 ┤     ● Note 1
          │
      5.5 ┤     ● Note 2
          │
      4.1 ┤     ● Note 3
          │
          └─────┬─────────────► X
              2.9
```