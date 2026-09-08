# Physics realism work

The requested scope covers all five areas below. This is an implementation record,
not a claim of validation against measured engine performance.

| Area | Implementation status | Remaining work |
| --- | --- | --- |
| Gas properties | Existing constant heat capacity and air molecular mass | Mixture transport, reaction mass balance, temperature-dependent energy and flow consistency |
| Thermal behavior | Hohenberg cylinder heat transfer and evolving wall temperature; conservation and engine-run checks passed | Calibration; coolant remains a prescribed reservoir |
| Combustion | Existing simplified flame propagation | Changing burn conditions, mixture/residual effects, calibration against burn histories |
| Pipe dynamics | Existing connected volumes | Distributed conservative pipe solver, boundaries, wave travel and reflection verification, runtime evaluation |
| Mechanical losses | Existing friction model | Temperature-dependent oil/viscous losses and calibration against motoring/coast-down data |

## Cylinder thermal model

The default uses the Hohenberg correlation:

`h = scale * 130 * V^-0.06 * (p/100000)^0.8 * T^-0.4 * (abs(mean_piston_speed)+1.4)^0.8`

Here V is m³, p is Pa, T is K, speed is m/s and h is W/(m² K).
The area includes the bore wall, piston face and head face using the existing
cylindrical chamber geometry. The correlation is empirical, not a universal
description of every chamber. See the correlation and engine comparisons in
[Millo et al., 2021](https://www.mdpi.com/2076-3417/11/13/6035).

Each cylinder has one metal temperature. Gas-to-metal and metal-to-coolant
exchange use a coupled backward-Euler solve. For the current constant-heat-capacity
gas model, this conserves gas + metal + coolant-reservoir energy and avoids
temperature overshoot from a stiff explicit heat-loss step. This solve must be
updated consistently when variable gas heat capacities are introduced.

Engine script inputs (SI values):

| Input | Default | Meaning |
| --- | --- | --- |
| `thermal_model` | `true` | `false` reproduces the old 90°C wall / 100 W/(m² K) heat-loss calculation |
| `wall_temperature` | `363.15` | Initial metal temperature, K |
| `coolant_temperature` | `363.15` | Prescribed coolant temperature, K |
| `wall_heat_capacity` | `4000.0` | Effective metal heat capacity per cylinder, J/K |
| `coolant_conductance` | `40.0` | Metal-to-coolant conductance per cylinder, W/K |
| `heat_transfer_scale` | `1.0` | Nonnegative correlation calibration multiplier |

Defaults represent a warm engine. For a cold metal/coolant start, set both
temperatures to `298.15`. The coolant reservoir is still held at that value; this
is not yet a radiator/thermostat warm-up simulation. Capacities and conductances
are explicit provisional parameters, not measurements for the bundled engines.

The headless summary reports mean `wall_temperature_K` and summed
`coolant_energy_J` (positive into the coolant). Legacy mode has no evolving wall
or reservoir ledger. Focused checks cover thermal equilibrium, energy balance,
and a stiff large time step. Engine comparisons and performance results belong
below as they are collected.

Windows Release observations: the 36 selected gas/simulator checks passed,
including the two focused thermal checks. Invalid zero heat capacity was rejected
with a script source location. Hidden GUI startup, reload, failed-reload
preservation and shutdown also passed.

Hayabusa, 10 simulated seconds, starter engaged for 3 seconds and speed-control
input 0.2 (not a direct throttle-plate angle):

| Model | Wall time (s) | Final RPM | Mean wall temperature (K) | Heat to coolant (J) |
| --- | --- | --- | --- | --- |
| Legacy | 8.757 | 10390 | 363.15 (fixed) | No ledger |
| Hohenberg, warm | 8.915 | 9885 | 381.86 | 13589 |
| Hohenberg, cold metal/coolant | 8.715 | 10056 | 317.64 | 14165 |

These are single-run comparisons, not performance guarantees or measured-engine
validation. They show that the new model affects operation and retains finite
state. Gas thermodynamics, combustion and friction are still the earlier models
at this checkpoint. The existing user-launched playtest was left running with
its previous build while these checks used separate staging folders.
