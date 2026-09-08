# Physics realism work

The requested scope covers all five areas below. This is an implementation record,
not a claim of validation against measured engine performance.

| Area | Implementation status | Remaining work |
| --- | --- | --- |
| Gas properties | Five-species mixture transport, mass-conserving hydrocarbon reaction, NASA temperature-dependent heat capacities, consistent flow and nonlinear thermal exchange | Engine calibration, additional fuels/chemistry, runtime optimization |
| Thermal behavior | Hohenberg cylinder heat transfer and evolving wall temperature; conservation and engine-run checks passed | Calibration; coolant remains a prescribed reservoir |
| Combustion | Evolving flame speed, corrected equivalence ratio and transported residual-gas dilution | Calibration against measured burn histories; chamber geometry remains simplified |
| Pipe dynamics | Distributed conservative runner/primary cells, automatic CFL substeps, wave/reflection conservation check | Higher-order reconstruction and calibrated junction/port losses |
| Mechanical losses | Oil-temperature-dependent viscous friction, oil/wall/coolant heat exchange and friction-work ledger | Calibration against motoring/coast-down data; bearings/accessories remain simplified |

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
temperature overshoot from a stiff explicit heat-loss step. With variable gas
properties a safeguarded nonlinear solve uses the same energy balance and the
temperature derivative of internal energy.

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

## Gas composition and thermodynamics

`variable_gas_properties: true` is the new engine default; set it to `false` to
compare the old constant-heat-capacity/air-mass gas approximation. The standalone
GasSystem API keeps its original default unless explicitly enabled.

Nitrogen, oxygen, carbon dioxide, water vapor and fuel travel with the gas. CO2
and H2O remain included in the old `p_inert` interface but are independently
tracked subsets. Mass, density and sound speed use the mixture molecular weight.
The intake now uses 21% O2 / 79% N2 and respects the configured oxygen/fuel ratio,
removing the old implicit 0.8 enrichment. Argon and humidity are not included.
Exhaust backflow now brings air rather than oxygen-free inert gas into the pipe.
Fresh cylinders, plenums and runners also initialize with ambient air.

The default fuel molecular weight is now 114.232 g/mol, matching C8H18. The
reaction derives an equivalent C_x H_y from configured molecular weight and
oxygen demand, consumes reactants together in stoichiometric proportions, and
creates explicit CO2/H2O while preserving total mass. Incompatible parameter
pairs are rejected. Fuel heat capacity uses isooctane as a surrogate even when a
custom equivalent hydrocarbon is configured; other fuels require their own data.

Heat-capacity coefficients are the NASA7 data for N2, O2, CO2, H2O and isooctane
from [Cantera's NASA gas database](https://github.com/Cantera/cantera/blob/main/data/nasa_gas.yaml).
They are used over 200–6000 K with the original 1000 K breakpoint. Outside that
range, the endpoint heat capacity is held constant. This is a numerical extension,
not validated high-temperature chemistry. Dissociation and emissions chemistry
are not modeled.

Sensible energy is a continuous integral of cv(T), with u(0)=0. Temperature is
recovered from energy using safeguarded Newton iteration. Reaction energy
includes a 298.15 K reference-state correction and the enthalpy-to-internal-energy
conversion; the configured fuel energy density supplies the heat of combustion.
Mixture polynomial coefficients and recovered temperatures are cached. Flow
uses the donor's gamma and molecular weight, transports products, and exchanges
internal/bulk kinetic energy consistently. The reservoir path limits transfers
at pressure equilibrium without using the old constant-cv pressure formula.

Focused checks cover energy/temperature inversion across polynomial breakpoints,
reaction mass and product yields, species/mass/energy transport, and variable-cv
gas/wall/coolant energy conservation. These passed with the existing selected gas
and simulator checks (39 total). A first 5-second Hayabusa run took 20.08 seconds;
after caching and correcting the intake composition, a run took 10.17 seconds
and ended at 8477 RPM. The operating points differ: this is not an isolated
optimization benchmark, and runtime still needs attention.

## Combustion

`dynamic_combustion: true` updates flame speed every physics step. The unburned
charge temperature is estimated from its ignition state and current pressure
using isentropic compression and the mixture gamma. This avoids using the bulk
burned-gas temperature to accelerate the remaining flame. It is still an
approximation, not a fully separate two-zone energy balance or detailed kinetics.

The laminar correlation now uses actual equivalence ratio (stoichiometric
oxygen/fuel divided by actual oxygen/fuel); previously that ratio was inverted.
A passive burned-gas mass fraction is produced by combustion and transported
through the intake/exhaust network. The ignition charge's residual fraction
attenuates laminar speed with `max(0, 1 - 2.1 * residual_fraction)`. This is an
empirical dilution approximation; the literature reports limitations, especially
at high dilution and low load. See [De Bellis et al., 2019](https://doi.org/10.4271/03-12-03-0018).
Setting `dynamic_combustion: false` freezes flame speed at ignition and disables
this dilution attenuation; the equivalence-ratio bug fix remains.

The existing flame geometry, turbulence curve and burning-efficiency parameters
are retained. Knock, flame quenching near surfaces, detailed ignition chemistry
and emissions prediction are not claimed. A focused check verifies the reference
laminar speed and the direction of mixture, temperature, pressure and dilution
effects.

## Oil and mechanical losses

`oil_model: true` evolves each cylinder's share of oil temperature from piston
friction work and exchanges heat with the metal and coolant. The coupled oil/metal
solve balances stored heat, coolant heat and supplied friction work. Friction
power is estimated from the instantaneous piston force and speed after each
mechanical step; this is not an exact work integral of the constraint solver.

Viscous piston friction scales with the oil's viscosity relative to 90°C. Its
speed dependence and the existing load-dependent Coulomb friction are retained.
The breakaway excess is bounded below by zero so it cannot create a negative
friction contribution at high side loads. Oil density variation is neglected.
The two-point Walther viscosity relation is fitted to the provided 40°C and 100°C
viscosities. [ASTM D341](https://store.astm.org/standards/d341) describes the
limited-range nature of viscosity-temperature fitting. Here viscosity evaluation
is held at endpoint values outside 250–450 K; extrapolation outside the measured
40–100°C points is provisional, not cold-cranking oil certification.

| Engine input | Default | Units / role |
| --- | --- | --- |
| `oil_model` | `true` | `false` disables oil heating and viscosity scaling |
| `oil_temperature` | `363.15` | Initial K |
| `oil_viscosity_40` | `80.0` | cSt at 40°C |
| `oil_viscosity_100` | `10.5` | cSt at 100°C |
| `oil_heat_capacity` | `2000.0` | J/K per cylinder |
| `oil_wall_conductance` | `10.0` | W/K per cylinder |
| `oil_coolant_conductance` | `5.0` | W/K per cylinder |
| `friction_scale` | `1.0` | Nonnegative piston-friction calibration multiplier |

The default oil and conductances are provisional. A cold-model comparison should
set wall, coolant **and oil** temperatures. Headless output includes average
`oil_temperature_K` and integrated `piston_friction_energy_J`; coolant energy
includes the oil branch. Full thermal conservation assumes the evolving-wall
model is enabled; legacy wall mode acts as a fixed-temperature reservoir.

## Distributed runners and exhaust primaries

`pipe_cells: 8` is the default per intake runner and exhaust primary. Values 2–64
enable a first-order finite-volume Euler solver with Rusanov fluxes. `1` selects
the previous lumped runner model. Length and cross section come from the engine
geometry. A CFL limit of 0.25 automatically subdivides fluid/coupling steps;
no simulated time is discarded to meet a frame budget.

The interior transports five species, axial momentum, total energy and the
burned-gas marker. Darcy wall friction converts lost bulk kinetic energy into gas
internal energy. `pipe_friction_factor` is the Darcy factor (default 0.02, nonnegative);
it is a provisional constant rather than a Reynolds/roughness correlation.
Existing calibrated port/orifice flow connects the endpoint
cells to the plenum, cylinder and collector using an operator split. Thus the
interior is conservative, while junction conditions retain the original
approximate orifice model; this is not a full multidimensional manifold solver.
Plenums and collectors remain lumped reservoirs.

The numerical flux is intentionally first-order and diffusive. Increase cell
count and compare results when assessing a resonance; eight cells do not resolve
all acoustic frequencies. A 64-cell closed-pipe standing-wave check verifies
the expected half-period phase reversal, reflecting boundaries and mass/energy
conservation. Engine startup and finite-state checks are separate from that
physical reference check. Higher resolution and nonlinear gas properties cost
CPU time, and full real-time speed is not guaranteed on this PC.

## Combined-model verification

All 53 selected gas, simulator, function and audio checks passed with the final
connected models. All 20 bundled engines passed 0.1-second startup checks.
The final video-enabled GUI passed hidden-desktop startup, minimize/restore,
successful reload, failed-reload preservation, recording and shutdown checks;
the produced video decoded without errors.
Five-second runs with two seconds of starter and a 0.2 speed-control input gave:

| Engine | Simulated time | Wall time | Final RPM | Mean wall / oil temperature |
| --- | --- | --- | --- | --- |
| Hayabusa | 5.005 s | 20.21 s | 9666 | 367.89 / 378.29 K |
| Ferrari V12 | 5.005 s | 34.40 s | 12049 | 365.85 / 373.21 K |

Both completed after starter release with finite state. These inputs let the
engines run at high RPM; the observations do not establish idle stability,
correct torque curves, or real-world temperature accuracy. The default full
model currently runs slower than real time on this PC. Resolution/correlation
controls are exposed for comparisons, and CPU profiling remains worthwhile.
