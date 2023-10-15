from depth_control.depth_control_node_rudder import DepthControlNode
d = DepthControlNode()
d.handle_cmd_msg(10.0)
d.current_depth=9.8
d.current_pitch = 25.0
d.compute_and_send()
# output ====================
# current depth 9.8
# current pitch 25.0
# depth target 10.0
# hej -4.999999999999982
# angle pid output 4.999999999999982
# rudder pid output -0.6666666666666673