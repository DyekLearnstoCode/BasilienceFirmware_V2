# BASILIENCE Firmware V3 State Machine

## Overview

BASILIENCE operates using a state-based control system.

Only one primary system state can be active at a time.

The state machine ensures:

- Stable environmental control
- Safe nutrient dosing
- Safe pH correction
- Controlled water refilling
- Predictable automation behavior

---

## State Priority

If multiple conditions occur simultaneously, the firmware processes them in the following order:

| Priority | State |
|-----------|--------|
| 1 | SAFETY_LOCK |
| 2 | REFILLING |
| 3 | DOSING_PH |
| 4 | DOSING_EC |
| 5 | NORMAL |

Example:

Water Level = 10%  
EC = 0.8  
pH = 5.0

↓

REFILLING executes first.

---

## System States

STARTUP
↓
NORMAL
├── REFILLING
├── DOSING_PH
├── MIXING_PH
├── DOSING_EC
├── MIXING_EC
└── SAFETY_LOCK

---

# STARTUP

### Purpose

Allow newly transplanted basil plants to acclimate and establish root growth.

### Sequence

Power ON

↓

Fogger ON  
Blower ON

↓

20 Minutes

↓

Fogger OFF  
Blower ON

↓

10 Minutes

↓

NORMAL

---

# NORMAL

### Purpose

Maintain the growing environment.

---

## Fogger

Adaptive humidity control:

Humidity < 60%

↓

Fogger ON

Humidity > 80%

↓

Fogger OFF

---

## Blower

Always ON

Purpose:

- Distribute fog
- Prevent stagnant zones
- Improve chamber uniformity

---

## Canopy Fan

Adaptive speed control.

| Condition | Speed |
|------------|--------|
| Cold | 30% |
| Normal | 60% |
| Hot | 100% |

Temperature range:

24°C – 27°C

---

## Grow Light

RTC controlled.

16 Hours ON

8 Hours OFF

---

## Water Temperature Control

Water Temperature < 24°C

↓

Water Heater ON

Water Temperature > 27°C

↓

Peltier ON

---

# REFILLING

### Trigger

Water Level < 20%

### Actions

Open Solenoid Valve

Environmental systems continue operating.

### Exit Condition

Water Level ≥ 80%

↓

Return to NORMAL

---

# DOSING_PH

### Trigger

pH < 5.5

or

pH > 6.5

### Actions

If pH is LOW:

Dose pH Up Pump

If pH is HIGH:

Dose pH Down Pump

### Environmental Behavior

| Device | State |
|----------|--------|
| Fogger | OFF |
| Blower | OFF |
| Canopy Fan | ON |
| Grow Light | Schedule |

### Next State

MIXING_PH

---

# MIXING_PH

### Purpose

Allow pH correction to stabilize.

### Actions

Wait 60 Seconds

### Environmental Behavior

| Device | State |
|----------|--------|
| Fogger | OFF |
| Blower | OFF |
| Canopy Fan | ON |
| Grow Light | Schedule |

### Exit Condition

pH within range

↓

NORMAL

Otherwise:

↓

DOSING_PH

---

# DOSING_EC

### Trigger

EC < Target EC

Target EC:

1.2 mS/cm

(adjustable in Config.h)

### Actions

Execute nutrient recipe.

Example:

Grow Pump

Bloom Pump

using configured dosing durations.

### Environmental Behavior

| Device | State |
|----------|--------|
| Fogger | OFF |
| Blower | OFF |
| Canopy Fan | ON |
| Grow Light | Schedule |

### Next State

MIXING_EC

---

# MIXING_EC

### Purpose

Allow nutrient concentration to stabilize.

### Actions

Wait 60 Seconds

### Environmental Behavior

| Device | State |
|----------|--------|
| Fogger | OFF |
| Blower | OFF |
| Canopy Fan | ON |
| Grow Light | Schedule |

### Exit Condition

EC ≥ Target EC

↓

NORMAL

Otherwise:

↓

DOSING_EC

---

# SAFETY_LOCK

### Priority

Highest priority state.

### Triggers

Examples:

- Sensor Failure
- Water Temperature Extreme
- Hardware Fault
- Manual Emergency Stop

### Actions

Fogger OFF

Blower OFF

Canopy Fan OFF

All Pumps OFF

Solenoid OFF

Peltier OFF

Water Heater OFF

### Recovery

Manual intervention required.

---

# Future Expansion

Reserved for:

- Firebase remote control
- Maintenance mode
- Calibration mode
- Advanced nutrient recipes
- Growth analytics

---

# Manual Mode

Manual Mode is NOT a primary state.

Manual Mode is a system flag:

```cpp
systemState.manualMode
```

When enabled:

- Automation decisions are suspended.
- User commands directly control actuators through Firebase.

This prevents the state machine from becoming unnecessarily complex while preserving full manual control capability.