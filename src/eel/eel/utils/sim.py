# UNIT: Meters per second.
# We can alter this one, to speed up simulation.
# NOTE: When increasing the speed, you will probably have to increase the update
# frequency in e. g. GNSS and IMU, so that it can react faster.
# NOTE: we measured 0.858 mps in gröndal 2 march 2025
# NOTE: we measured 0.96 mps outside of rotholmen 18 april 2025. But since it travelled in a squiggly pattern,
# rather than "as the crow flies", we're rounding it to 1.0
LINEAR_VELOCITY = 1.0

# UNIT: Degrees per second.
# Angular velocity depends on speed, since otherwise simulation wouldn't turn
# fast enough and would therefore drift a lot.
# Which is probably also true in real life, since faster water flow would
# make the rudder have a greater impact. We should however change the value here,
# to make it reflect its real life angular velocity. When we have done measurements, that is.
ANGULAR_VELOCITY = 10.0 * LINEAR_VELOCITY
