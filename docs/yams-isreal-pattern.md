# YAMS isReal/Stubbed Pattern

When creating or refactoring a YAMS-based subsystem, follow this pattern to support both real hardware and a lightweight "stubbed" mode for isolated testing.

## Constructor

- Add `boolean isReal` parameter, store as `private final boolean m_isReal`
- Move all TalonFX, SmartMotorControllerConfig, TalonFXWrapper, mechanism config, and mechanism construction from field declarations into the constructor body
- Wrap hardware creation in `if (m_isReal) { ... }` — when false, mechanism fields stay `null`
- NetworkTable entries and lookup tables are always initialized (they have no hardware dependency)

```java
public MySubsystem(boolean isReal) {
    m_isReal = isReal;

    if (m_isReal) {
        TalonFX motor = new TalonFX(CAN_ID);
        SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // ... config chain ...
            ;
        var controller = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig);
        // Build mechanism (FlyWheel, Arm, Pivot, etc.)
        mechanism = new FlyWheel(new FlyWheelConfig(controller)...);
    }

    // NetworkTable entries — always initialized
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("MySubsystem");
    m_targetRPM = table.getDoubleTopic("TargetRPM").getEntry(0.0);
    m_targetRPM.set(0.0);
}
```

## Stub Tracking Fields

- **Velocity-based** (FlyWheel): `private AngularVelocity m_stubbedVelocity = RPM.of(0);`
- **Position-based** (Arm/Pivot): `private Angle m_stubbedAngle = Degrees.of(STARTING_POSITION);`

## Public Methods — Null-Check Dispatch

Every public method checks the mechanism for null to branch between real and stubbed behavior:

### Velocity-based example (FlyWheel)
```java
public AngularVelocity getVelocity() {
    if (mechanism != null) return mechanism.getSpeed();
    return m_stubbedVelocity;
}

public Command setVelocity(AngularVelocity speed) {
    if (mechanism != null) return mechanism.setSpeed(speed);
    return Commands.run(() -> m_stubbedVelocity = speed, this);
}

public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
    if (mechanism != null) return mechanism.setSpeed(speedSupplier);
    return Commands.run(() -> m_stubbedVelocity = speedSupplier.get(), this);
}

public Command set(double dutyCycle) {
    if (mechanism != null) return mechanism.set(dutyCycle);
    return Commands.run(() -> {}, this);  // no-op but holds subsystem
}
```

### Position-based example (Arm/Pivot)
```java
public Angle getAngle() {
    if (mechanism != null) return mechanism.getAngle();
    return m_stubbedAngle;
}

public Command setAngle(Angle angle) {
    if (mechanism != null) return mechanism.setAngle(angle);
    return Commands.run(() -> m_stubbedAngle = angle, this);
}

public Command setAngle(Supplier<Angle> angleSupplier) {
    if (mechanism != null) return mechanism.setAngle(angleSupplier);
    return Commands.run(() -> m_stubbedAngle = angleSupplier.get(), this);
}

public Command setDutyCycle(double dutyCycle) {
    if (mechanism != null) return mechanism.set(dutyCycle);
    return Commands.run(() -> {}, this);
}

public Command sysId() {
    if (mechanism != null) return mechanism.sysId();
    return Commands.none();
}
```

## periodic() and simulationPeriodic()

```java
@Override
public void periodic() {
    Logger.recordOutput("SubsystemName/IsStubbed", !m_isReal);

    if (mechanism != null) {
        mechanism.updateTelemetry();
    }
}

@Override
public void simulationPeriodic() {
    if (mechanism != null) {
        mechanism.simIterate();
    }
}
```

## Coordinator Pattern (Shooter)

The Shooter coordinator accepts `boolean isReal` and passes it to all sub-subsystems:

```java
public Shooter(boolean isReal) {
    m_turret = new Turret(isReal);
    m_hood = new Hood(isReal);
    m_bottomFlyWheel = new BottomFlyWheel(isReal);
    m_kicker = new Kicker(isReal);
    m_spindexer = new Spindexer(isReal);
    // ... rest of init (NetworkTable, default commands, etc.)
}
```

## RobotContainer Wiring

```java
m_shooter = new Shooter(true);   // real mode
m_shooter = new Shooter(false);  // stubbed mode for testing
```

## Implemented Subsystems

| Subsystem | Type | Mechanism | CAN ID(s) | Motor | Notes |
|-----------|------|-----------|-----------|-------|-------|
| Hood | Position (Arm) | Arm | 24 | KrakenX44 | 19:1 reduction, ArmFeedforward, 20A stator |
| Turret | Position (Pivot) | Pivot | 1 | KrakenX60 | 3:4 dual-stage, ±90° soft/±100° hard limits |
| BottomFlyWheel | Velocity (FlyWheel) | FlyWheel | 35, 36 | KrakenX60(2) | Follower on right motor, lookup tables always init |
| Kicker | Velocity (FlyWheel) | FlyWheel | 38 | Falcon500 | 1:1 gearing, COAST, 40A |
| Spindexer | Velocity (FlyWheel) | FlyWheel | 39 | Falcon500 | 1:1 gearing, motor inverted, COAST, 40A |

## Key Lessons
- Stubbed commands must use `Commands.run(() -> ..., this)` (not `Commands.runOnce`) to properly hold the subsystem requirement
- For duty cycle commands in stubbed mode, use `Commands.run(() -> {}, this)` — no state to track but still holds subsystem
- Lookup tables and NetworkTable entries have no hardware dependency — always initialize them
- Follower motor setup (e.g., `rightMotor.setControl(new Follower(...))`) must be inside the `if (m_isReal)` block
- Use `!m_isReal` for telemetry value (key is "IsStubbed", true when stubbed)
- Don't remove existing comments when refactoring
