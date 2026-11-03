# Cart-pole cascade PID in Webots

Open **`worlds/Cart Pole PID.wbt`** in Webots R2025a or newer and press **Run**.
The world contains **one cart-pole** using the supplied `protos/CartPole.proto`, with a 1 kg cart, a
0.2 kg passive pole, and the existing arena/background. Python 3 is required;
no extra Python packages or compilation are needed. If Webots cannot find
Python, set its Python command to your Python 3 executable in Preferences.

The pole starts at **0.05 rad (2.86 degrees)** from upright. The controller
catches it and returns the cart to the center of the rail.

Click the 3D view to give it keyboard focus:

- **A:** decrease cart target by **0.10 m per press**.
- **D:** increase cart target by **0.10 m per press**.
- **Space:** return the target to the center.
- **Reset simulation:** restart after a fall or travel cutout.

Holding A or D changes the target only once; release and press again for the
next increment. The target remains limited to ±1.4 m.

For interactive use, open `worlds/Cart Pole PID.wbt` directly. It runs until
you stop it and never closes Webots automatically. `tests/webots_check.py`
is an automated test runner: it deliberately closes its temporary Webots
window after 30 **simulated** seconds, which may take only a few real seconds
in fast mode. Reopening a generated test world manually also keeps Webots open;
automatic exit requires the environment flag set by the test runner.

## Controller

`controllers/cascade_pid/cascade_pid.py` reads `cart_position` and `pole_angle`
and sends force commands to `cart_motor`. The reusable control logic and gains
are in `controllers/cascade_pid/control.py`.

```text
cart target -> position PID (32 ms) -> desired pole angle
                                      |
                                      v
                         angle PID (8 ms) -> cart force
```

| Loop | Kp | Ki | Kd | Output limit |
| --- | ---: | ---: | ---: | --- |
| Cart position | 0.10 | 0.015 | 0.16 | ±0.12 rad |
| Pole angle | 100.0 | 5.0 | 18.0 | ±20 N |

All calculations use metres, radians, seconds, and newtons. With `x` the cart
position and `theta` the pole angle, the control laws are:

```text
e_x       = x_reference - x
theta_ref = clamp(Kp_x*e_x + Ki_x*integral(e_x) - Kd_x*x_velocity)
e_theta   = theta - theta_ref
force     = clamp(Kp_theta*e_theta + Ki_theta*integral(e_theta)
                  + Kd_theta*angular_velocity)
```

Positive pole angle leans toward +X. Positive cart force catches that lean,
so the inner loop deliberately uses **measured angle minus desired angle**.
Derivatives use measured motion to avoid derivative kicks on reference changes.
Both velocity estimates have a 20 ms low-pass filter. Each integral has a
limit and conditional antiwindup; the outer integrator also pauses when the
inner loop saturates. Cart targets are limited to ±1.4 m and move internally
at at most 0.25 m/s.

The motor's `setForce()` selects direct force control, bypassing its built-in
position PID; see the [Webots motor documentation](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/motor.md).
The pole hinge remains passive, and the control loop uses only the two joint
sensors. Supervisor access is used only by the simulator test harness.

This is an upright balancing controller, not a swing-up controller. At a tilt
of 0.6 rad, cart travel of ±1.9 m, or an invalid sensor reading, it latches
the force to zero and prints a message. This cutout does not actively brake
the cart; reset the simulation to restart. The physical rail stops are ±2 m.

## Configuration and recording

Edit the `CartPole` instance's `controllerArgs` in the world to set an initial
target and optionally record a CSV. For example:

```webots
controllerArgs [ "--target" "0.5" "--log" "balance.csv" ]
```

Relative log paths are resolved from `controllers/cascade_pid/`. A log path
is overwritten on each run. CSV columns include simulation time, position,
target, filtered reference, pole angle, angle reference, velocities, force,
and fault state. Live values also appear in the Webots console every 2 seconds.

Keep the world's `basicTimeStep` at **8 ms** for the tuned default. The control
code accepts up to 16 ms and computes its loop intervals from the world step.
The world's physics sleep thresholds are zero so the small balancing motions
remain simulated; see the [WorldInfo reference](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/worldinfo.md).
Changing the masses, pole dimensions, actuator limits, or gravity requires
rechecking the gains.

## Verification

Run the independent nonlinear model and controller regression checks:

```sh
python3 -m unittest discover -s tests -v
```

These checks cover initial tilts in both directions, cart displacement,
positive/negative target changes, a force disturbance, limits, antiwindup,
and fault latching. The model includes the uniform pole's inertia and joint
damping; it excludes collisions and Webots solver effects.

To exercise the actual Webots physics engine:

```sh
python3 tests/webots_check.py
# Optional: run all scenarios sequentially, one cart-pole at a time:
python3 tests/webots_check.py --scenario all
# On other platforms, if webots is not on PATH:
python3 tests/webots_check.py --webots /path/to/webots
```

The harness copies the world environment, PROTO, and production controller to
a temporary project containing exactly one cart-pole. By default it checks
the initial 0.05 rad tilt for 30 simulated seconds. `--scenario all` runs seven
separate simulations sequentially: initial tilts ±0.05 and ±0.15 rad, targets
±0.5 m, and a 5 N cart push lasting 0.2 s. It verifies actual initial hinge readings, bounded force/travel, no
controller faults, and settled position/angle over the final five seconds.
The temporary folder retains CSVs, simulator output, and a JSON summary.

Verified on Webots R2025a: all 11 offline tests and all seven 30-second Webots
scenarios passed. In the default run the peak cart excursion was 0.0506 m and
peak force was 5.01 N. During the final five seconds its position error stayed
below 0.0004 m. Across all seven scenarios, the final five-second position
error stayed below 0.0025 m and pole angle below 0.00001 rad.

The supplied PROTO's pole rotation template was corrected to Webots JavaScript
template syntax. Internal `CART`/`POLE` names support test inspection. The world
now instantiates the robot and frames the complete rail in a larger arena.
