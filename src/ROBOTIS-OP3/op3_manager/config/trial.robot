[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyACM0 | 4000000   | r_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME       | BULK READ ITEMS
dynamixel | /dev/ttyACM0 | 1   | MX-64          | 2.0      | r_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyACM0 | 2   | MX-64          | 2.0      | l_sho_pitch    | present_position, position_p_gain, position_i_gain, position_d_gain
sensor    | /dev/ttyACM0 | 200 | OPEN-CR        | 2.0      | open-cr        | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw

