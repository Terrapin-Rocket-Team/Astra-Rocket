# Flight Stages

`RocketState` exposes the current `FlightStage` through
`getFlightStage()`. The current state machine is:

```text
PAD_IDLE → BOOST → COAST → APOGEE
                              ↓
EXPECTING_DROGUE → UNDER_DROGUE → EXPECTING_MAIN → UNDER_MAIN → LANDED
```

| Stage | Meaning |
| --- | --- |
| `PAD_IDLE` | Initialized on the pad and waiting for sustained liftoff acceleration |
| `BOOST` | Powered ascent after liftoff detection |
| `COAST` | Motor burnout detected; vehicle is coasting toward apogee |
| `APOGEE` | Vertical velocity crossed the configured apogee condition |
| `EXPECTING_DROGUE` | Descending and waiting for a sustained drogue-like descent rate |
| `UNDER_DROGUE` | Estimated descent rate is consistent with drogue recovery |
| `EXPECTING_MAIN` | Below the current main-altitude boundary and waiting for a main-like descent rate |
| `UNDER_MAIN` | Estimated descent rate is consistent with main recovery |
| `LANDED` | Low vertical velocity persisted for the configured landing duration |

!!! danger "Do not use these names as deployment confirmation"
    Recovery stages are inferences from state estimates. They do not confirm
    electrical continuity, actuator motion, pyro firing, or parachute
    inflation. Independent deployment logic and validation are required.

Useful state accessors include:

```cpp
RocketState* state = rocket.getRocketState();
FlightStage stage = state->getFlightStage();
double stageTime = state->getTimeInStage();
double flightTime = state->getTimeSinceLaunch();
double altitudeAGL = state->getAltitudeAGL();
Quaternion rocketToEarth = state->getRocketOrientation();
```

`setFlightStage()` exists for tests and explicit overrides. Application code
that overrides the automatic state machine must document why and how safety is
preserved.
