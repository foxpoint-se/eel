# how much the tank is filled from 0.0 to 1.0
float32 current_level

# target level can be empty if it hasn't received any instructions
float32[] target_level

# target_reached | ceiling_reached | floor_reached | no_target | adjusting
string target_status

# whether the tank is in autocorrecting mode or not
bool is_autocorrecting
