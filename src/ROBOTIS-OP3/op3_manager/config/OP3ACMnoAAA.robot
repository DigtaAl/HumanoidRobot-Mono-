[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyAAA | 2000000   | OPEN-CR

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
#dynamixel | /dev/ttyAAA | 1   | MX-28          | 2.0      | r_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 2   | MX-28          | 2.0      | l_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 3   | MX-28          | 2.0      | r_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 4   | MX-28          | 2.0      | l_sho_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 5   | MX-28          | 2.0      | r_el           | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 6   | MX-28          | 2.0      | l_el           | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 7   | MX-28          | 2.0      | r_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 8   | MX-28          | 2.0      | l_hip_yaw      | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 9   | MX-64          | 2.0      | r_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 10  | MX-64          | 2.0      | l_hip_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 11  | MX-64          | 2.0      | r_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 12  | MX-64          | 2.0      | l_hip_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 13  | MX-64          | 2.0      | r_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 14  | MX-64          | 2.0      | l_knee         | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 15  | MX-64          | 2.0      | r_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 16  | MX-64          | 2.0      | l_ank_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 17  | MX-64          | 2.0      | r_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 18  | MX-64          | 2.0      | l_ank_roll     | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 19  | MX-28          | 2.0      | head_pan       | present_position, position_p_gain, position_i_gain, position_d_gain
#dynamixel | /dev/ttyAAA | 20  | MX-28          | 2.0      | head_tilt      | present_position, position_p_gain, position_i_gain, position_d_gain
sensor    | /dev/ttyAAA | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw