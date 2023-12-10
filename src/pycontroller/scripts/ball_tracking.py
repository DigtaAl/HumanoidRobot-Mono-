import math
from simple_pid import PID


x_p = 0.3
y_p = 0.23
x_i = 0.001
y_i = 0.001
x_d = 0.001
y_d = 0.001

flip_x = 1
flip_y = -1

pid_x = PID(0.4, 0.001, 0.01, setpoint=0)
pid_y = PID(0.23, 0.001, 0.01, setpoint=0)

# stall movement

pid_x.output_limits = (-1.0, 1.0)
pid_y.output_limits = (-1.0, 1.0)


errorPitch = 0.0
out_scale_x = 4
out_scale_y = 4

max_pitch = 1
min_pitch = -0.15
max_yaw = 1.5
min_yaw = -1.5

zero_offset_x = 0
zero_offset_y = 0.2

pitch = 0.0
yaw = zero_offset_y

ball_track = None

isEnabled = True

####
search_state = 0
s_max_angle = 0.8
turn_speed_delay = 0.53
last_search_turn = 0

searching = False

def reload():
    pid_x.tunings = (x_p,x_i,x_d)
    pid_y.tunings = (y_p,y_i,y_d)
    print("tunings set")
    pid_x.reset()
    pid_y.reset()
    print("PID reset")

def track(error):
    global pid_x
    global yaw
    global pid_y
    global pitch
    global search_state
    search_state = 0

    out_x = pid_x(error.x)*out_scale_x*flip_x
    out_y = pid_y(error.y)*out_scale_y*flip_y

    pitch += out_y * 0.1
    yaw += out_x * 0.1

    set_py(pitch, yaw)

def set_py(_pitch, _yaw):
    global pitch
    global yaw

    pitch = max(min(_pitch, max_pitch), min_pitch)
    yaw = max(min(_yaw, max_yaw), min_yaw)

scan_bufer = []
last_scan_epoch = 0.1

def set_py_from_buff():
    span_yaw = max_yaw - min_yaw

    size_buff = len(scan_bufer)
    if(size_buff <= 0): 
        set_py(0.3, 0)
        return

    buf_item = scan_bufer[0]

    if(size_buff > 10):
        buf_item = scan_bufer[-10]

    back_yaw = 0
    if(buf_item[2] == 1):
        back_yaw = (buf_item[1]/turn_speed_delay*span_yaw) + min_yaw
    else:
        back_yaw = (buf_item[1]/turn_speed_delay*-span_yaw) + max_yaw

    set_py(buf_item[0], back_yaw)


def search(time):
    global search_state
    global yaw
    global pitch
    global last_search_turn

    delta_last_search = time - last_search_turn
    if(search_state == 1):
        scan_bufer.append((pitch, delta_last_search, search_state))
        if delta_last_search >= turn_speed_delay:
            yaw = max_yaw
            last_search_turn = time
            pitch -= 0.25
            search_state = 2
            if pitch <= min_pitch: search_state = 0

    elif(search_state == 2):
        scan_bufer.append((pitch, delta_last_search, search_state))
        if time - last_search_turn >= turn_speed_delay:
            yaw = min_yaw
            last_search_turn = time
            pitch-= 0.25
            search_state = 1
            if pitch <= min_pitch: search_state = 0
        
    elif search_state == 0:
        yaw = min_yaw+0.3
        pitch = max_pitch
        search_state = 1
