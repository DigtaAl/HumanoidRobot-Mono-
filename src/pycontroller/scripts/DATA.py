# find ball
# chase ball, while do localization (regular goal check) when 
# catch ball, move ball towards goal
# shoot

# convention:
# PRE - > ING -> mas juga

import time
import numpy as np
import math

# repeat

BALL_SEARCHING = 0

PAUSE = 1

AUTO_READY_INIT = 2
AUTO_READY_INIT_STARTING = 3
AUTO_READY_POSITIONING = 4
AUTO_READY_POST = 5
AUTO_READY_TURNING = 6

AUTO_PLAY = 7

WALK_2_SHOOT_INIT_1 = 8
WALK_2_SHOOT_INIT_2 = 9
WALK_2_SHOOT_INIT_3 = 10

GOAL_SCAN_INIT = 11
GOAL_SCANING = 12


WALK_INIT = 13
WALK_INIT_STARTING = 14

WALK_STOP_INIT = 15
WALK_STOPING = 16

SHOOTING_INIT = 17
SHOOTING = 18
SHOOTING_POST = 19

SHOOTING_2_WALK = 20
SHOOTING_2_WALK_POST = 21

GOAL_ALIGN_BY_YAW_INIT = 22
GOAL_ALIGN_BY_YAW_TURNING = 23

BALL_ALIGN_INIT = 24
BALL_ALIGNING = 25

BALL_SEARCHING_TURNING = 26
BALL_SEARCHING_INIT = 27
START_YAW_COMPE = 28
END_YAW_COMPE = 29

WALK_2_STANDUP_INIT = 30
ACTION_2_STANDUP_INIT = 31
STANDING_UP = 32
STANDUP_POST = 33
WALK_2_STANDUP_INIT_2 = 34

ROBOT_AVOID_INIT = 35
ROBOT_AVOID_INIT_AVG = 36
ROBOT_AVOID_EVADE = 37
ROBOT_AVOID_REALIGN = 38
ROBOT_AVOID_POST = 39

TESTING_SPEED_PRE = 40
TESTING_SPEED_INIT = 41
TESTING_SPEED_POST = 42
TESTING_SPEED_STANDBY = 43

states_dict = {
    0 : "BALL_SEARCHING",
    1 : "PAUSE",
    2 : "AUTO_READY_INIT",
    3 : "AUTO_READY_INIT_STARTING",
    4 : "AUTO_READY_POSITIONING",
    5 : "AUTO_READY_POST",
    6 : "AUTO_READY_TURNING",
    7 : "AUTO_PLAY",
    8 : "WALK_2_SHOOT_INIT_1",
    9 : "WALK_2_SHOOT_INIT_2",
    10 : "WALK_2_SHOOT_INIT_3",
    11 : "GOAL_SCAN_INIT",
    12 : "GOAL_SCANING",
    13 : "WALK_INIT",
    14 : "WALK_INIT_STARTING",
    15 : "WALK_STOP_INIT",
    16 : "WALK_STOPING",
    17 : "SHOOTING_INIT",
    18 : "SHOOTING",
    19 : "SHOOTING_POST",
    20 : "SHOOTING_2_WALK",
    21 : "SHOOTING_2_WALK_POST",
    22 : "GOAL_ALIGN_BY_YAW_INIT",
    23 : "GOAL_ALIGN_BY_YAW_TURNING",
    24 : "GOAL_ALIGN_INIT",
    25 : "GOAL_ALIGNING",
    26 : "BALL_SEARCHING_TURNING",
    27 : "BALL_SEARCHING_INIT",
    28 : "START_YAW_COMPE",
    29 : "END_YAW_COMPE",
    30 : "WALK_2_STANDUP_INIT",
    31 : "ACTION_2_STANDUP_INIT",
    32 : "STANDING_UP",
    33 : "STANDUP_POST",
    34 : "WALK_2_STANDUP_INIT_2",
#
    35 : "ROBOT_AVOID_INIT",
    36 : "ROBOT_AVOID_INIT_AVG",
    37 : "ROBOT_AVOID_EVADE",
    38 : "ROBOT_AVOID_REALIGN",
    39 : "ROBOT_AVOID_POST",

    40 : "TESTING_SPEED_PRE",
    41 : "TESTING_SPEED_INIT",
    42 : "TESTING_SPEED_POST",
    43 : "TESTING_SPEED_STANDBY"
}

state = PAUSE

yaw_dead_area = 0.15

yaw_init = -0.75
last_yaw = 0.0
yaw = yaw_init

gt = None
infer = None
bt = None
walk = None

pubMotionIndex = None
isActionRunning = None
pubEnaMod = None
pubEnableOffset = None

enabled = True
goal_align_time = 0.0
start_align_turn = 0

gt_on_ball_search_last = 0
gt_on_ball_search_interval = 10
gt_on_ball_search_dead_yaw = 0.2
gt_on_ball_search_last_head_pos = [0,0]

setwalkparams = None
setwalkcmd = None

interval_checking = 10

goal_align_post_interval = 10
goal_align_post_start = 0
goal_align_angle_start = 0
goal_align_angle_time = 0

show_head_angle = False

timed_start = 0
timed_delay = 0

initialized = False

actionEnabled = False
ready_time = 15
play_delay = 4

turn_yaw_max_rate = 0.65

time_play_start = 0
max_time_from_play = 1000

pitch_ball_tar = -0.67
yaw_ball_tar = -0.14
ball_tar_deviation = 0.1
yaw_ball_dev_multipler = 1
pitch_ball_dev_multipler = 1

ball_search_loss = 0
max_ball_search_loss = 100

odo_10min_dev = 0.0
odo_deviation = 0.0
time_odo_start = 0

odo_min_max = 1.5
grad_to_yaw_gain = 1.0

show_ypr_counter = 0

z_amp_turning = 0.03
z_amp_normal = 0.026

yaw_compe_ball_align = 0.1
time_multi_goal_align_yaw = 4.5
time_multi_goal_align = 5
yaw_x_turning_max = 1.5

enable_goal_det = True
enable_ball_align = False

set_yaw_compe_start_time = 0.0
set_yaw_compe_start = 0.0

ypr = np.array([.0,.0,.0])
ypr_offset = np.array([.0,.0,.0])

standing_up = False

head_speed_idx = 100
robot_avoid_dir = 0.0
robot_dir_acu = []
data_testing = False

def clamp(val, _min, _max):
    return max(min(val, _max), _min)

def plus_or_min(val, out):
    if val > 0: return out
    return -out

def set_compe():
    set_state(START_YAW_COMPE)

def set_ypr(newypr):
    global ypr
    global show_ypr_counter
    global yaw
    ypr[0] = newypr.yaw
    ypr[1] = newypr.pitch
    ypr[2] = newypr.roll
    ypr = ypr - ypr_offset

    if show_ypr_counter >= 30:
        # print("pitch: "+str(ypr[1]))
        # print("yaw: "+str(yaw))
        show_ypr_counter = 0
    show_ypr_counter+=1

    yaw = (ypr[0]/1800*math.pi) + update_odo_dev()

def zero_ypr():
    global ypr_offset
    global time_odo_start
    ypr_offset = ypr + ypr_offset
    time_odo_start = time.time()

def update_odo_dev():
    global odo_deviation
    odo_deviation = (time.time() - time_odo_start) / 600 * odo_10min_dev
    return odo_deviation

def set_state(new_state, _timed_delay = 0):
    global state
    global timed_start
    global timed_delay
    state = new_state
    timed_delay = _timed_delay
    timed_start = time.time()
    print("NEW STATE: "+states_dict[new_state])

def init_action(_pubMotionIndex, _isActionRunning, _pubEnaMod, _pubEnableOffset):
    global pubMotionIndex
    global isActionRunning
    global pubEnaMod
    global pubEnableOffset
    pubMotionIndex = _pubMotionIndex
    isActionRunning = _isActionRunning
    pubEnaMod = _pubEnaMod
    pubEnableOffset = _pubEnableOffset

def enableWalk():
    global actionEnabled
    global standing_up
    standing_up = False
    actionEnabled = False
    pubEnaMod.publish("walking_module")
    pubEnaMod.publish("head_control_module")

def enableAction():
    global actionEnabled
    if not actionEnabled:
        actionEnabled = True
        pubEnaMod.publish("action_module")
        pubEnaMod.publish("head_control_module")

def enableActionOnly():
    global actionEnabled
    # if not actionEnabled:
    actionEnabled = True
    pubEnaMod.publish("action_module")

def playAction(index):
    pubMotionIndex.publish(index)

def stopAction():
    pubMotionIndex.publish(-2)

def init(goal_tracker, inference, ball_tracker, walking, _setwalkparams, _setwalkcmd):
    global gt
    global infer
    global bt
    global walk
    global setwalkparams
    global setwalkcmd
    global initialized

    gt = goal_tracker
    infer = inference
    bt = ball_tracker
    walk = walking
    setwalkcmd = _setwalkcmd
    setwalkparams = _setwalkparams
    initialized = True

def run(time, head_control, dets):
    global gt_on_ball_search_last
    global gt_on_ball_search_last_head_pos
    global state
    global goal_align_time
    global start_align_turn
    global goal_align_post_start
    global goal_align_angle_start
    global goal_align_angle_time
    global yaw
    global time_play_start
    global ball_search_loss
    global set_yaw_compe_start_time
    global set_yaw_compe_start
    global odo_10min_dev
    global standing_up
    global robot_avoid_dir
    global data_testing

    deltaT = time - timed_start
    timedEnd = deltaT > timed_delay

    head_pitch = head_control[0]
    head_yaw = head_control[1]
    goal = gt.goal
    goal_theta = goal.theta.item(0)
    pitch = ypr[1]

    if show_head_angle:
        print("head py:"+str(head_pitch)+", "+str(head_yaw))

    if state == BALL_SEARCHING_INIT:
        setwalkparams(["z_move_amplitude", z_amp_normal])
        set_state(BALL_SEARCHING)

    elif state == BALL_SEARCHING:
        move = 0.6
        if bt.isEnabled and not gt.enabled and infer.ball_lock: 
            if head_pitch < -0.45: move = 0.3   
            if head_pitch < -0.56 and abs(head_yaw) < 0.1 :
                if abs(yaw/2) < 0.25:
                    set_state(GOAL_SCAN_INIT)
                elif time - time_play_start < max_time_from_play:
                    set_state(GOAL_ALIGN_BY_YAW_INIT)
                else:
                    set_state(GOAL_SCAN_INIT)
                return 
            ball_search_loss = 0
        else:
            ball_search_loss += 1
        yaw_control = clamp(head_yaw, -turn_yaw_max_rate, turn_yaw_max_rate)
        if abs(head_yaw) < 0.1:
            yaw_control = plus_or_min(head_yaw, 0.3)
        walk.setTarget(0.0, move, yaw_control)

        if ball_search_loss > max_ball_search_loss:
            walk.setTarget(0.0, 0.0, 1)
            set_state(BALL_SEARCHING_TURNING, 2)
            ball_search_loss = 0
        
    elif state == BALL_SEARCHING_TURNING:
        if bt.isEnabled and not gt.enabled and infer.ball_lock:
            set_state(BALL_SEARCHING_INIT)
        if timedEnd:
            walk.setTarget(0.0, 0.0, 0.0)
            set_state(BALL_SEARCHING_INIT)

    elif state == GOAL_SCAN_INIT: # init scan
        gt.enabled = True
        gt.set_pre_head_pos(head_control)
        walk.setTarget()
        set_state(GOAL_SCANING)
    
    elif state == GOAL_ALIGN_BY_YAW_INIT:
        turn_gain = plus_or_min(-yaw/2, yaw_x_turning_max)
        yaw_gain = clamp(turn_gain, -0.42, 0.42)
        walk.setTarget(-turn_gain , 0.045, yaw_gain)
        setwalkparams(["z_move_amplitude", z_amp_turning])
        set_state(GOAL_ALIGN_BY_YAW_TURNING, abs(yaw) * time_multi_goal_align_yaw)

    elif state == GOAL_ALIGN_BY_YAW_TURNING:
        if timedEnd:
            # yaw = yaw_init
            if not enable_goal_det:
                set_state(WALK_2_SHOOT_INIT_1)
            else: 
                set_state(GOAL_SCAN_INIT)

    elif state == GOAL_SCANING:
        if not gt.enabled:
            if goal.found:
                
                turn_gain = plus_or_min(goal_theta, yaw_x_turning_max)

                yaw_gain = clamp(turn_gain, -0.42, 0.42)
                walk.setTarget(-turn_gain , 0.045, yaw_gain)
                setwalkparams(["z_move_amplitude", z_amp_turning])
                if not enable_ball_align:
                    set_state(WALK_2_SHOOT_INIT_1, abs(goal_theta) * time_multi_goal_align)
                else: 
                    set_state(BALL_ALIGN_INIT, abs(goal_theta) * time_multi_goal_align)
            else:
                # if not enable_ball_align:
                set_state(WALK_2_SHOOT_INIT_1)
                # else: 
                # set_state(BALL_ALIGN_INIT)
                print("goal not found")


    elif state == BALL_ALIGN_INIT:
        if timedEnd:
            set_state(BALL_ALIGNING)

    elif state == BALL_ALIGNING:
        if bt.isEnabled and not gt.enabled and infer.ball_lock:
            pitch_deviation = head_pitch - pitch_ball_tar 
            yaw_deviation =  head_yaw - yaw_ball_tar
            if abs(pitch_deviation) < ball_tar_deviation:
                if abs(yaw_deviation) < ball_tar_deviation:
                    set_state(WALK_2_SHOOT_INIT_1)
            yaw_out = plus_or_min(yaw_deviation, 0.6)
            pitch_out = plus_or_min(pitch_deviation, 0.3)
            pitch_out = clamp(pitch_out, -0.15, 0.20)
            walk.setTarget(yaw_out, pitch_out, yaw_compe_ball_align)
    
    elif state == WALK_2_SHOOT_INIT_1:
        if timedEnd:
            walk.setTarget()
            setwalkcmd("stop")
            set_state(WALK_2_SHOOT_INIT_2, 1)
    
    elif state == WALK_2_SHOOT_INIT_2:
        if timedEnd:
            enableAction()
            set_state(WALK_2_SHOOT_INIT_3, 1)

    elif state == WALK_INIT:
        setwalkcmd("start")
        # yaw = yaw_init
        walk.setTarget()
        set_state(WALK_INIT_STARTING, play_delay)

    elif state == WALK_INIT_STARTING:
        if timedEnd:
            set_state(BALL_SEARCHING)

    elif state == WALK_STOP_INIT:
        walk.setTarget()
        set_state(WALK_STOPING, 1)
    elif state == WALK_STOPING:
        if timedEnd:
            setwalkcmd("stop")
            set_state(PAUSE)
        
    elif state == WALK_2_SHOOT_INIT_3:
        if timedEnd:
            set_state(SHOOTING_INIT)

    elif state == SHOOTING_2_WALK:
        if timedEnd:
            setwalkcmd("start")
            set_state(SHOOTING_2_WALK_POST, 0.5)
    
    elif state == SHOOTING_POST:
        if timedEnd:
            enableWalk()
            set_state(SHOOTING_2_WALK, 2)

    elif state == SHOOTING_INIT:
        playAction(12)
        set_state(SHOOTING, 6)
    
    elif state == SHOOTING:
        if timedEnd:
            stopAction()
            set_state(SHOOTING_POST, 1)

    elif state == SHOOTING_2_WALK_POST:
        walk.setTarget()
        set_state(WALK_INIT_STARTING, 3)
    
    elif state == AUTO_READY_INIT:
        setwalkcmd("start")
        # yaw = yaw_init
        walk.setTarget()
        set_state(AUTO_READY_INIT_STARTING, 3)

    elif state == AUTO_READY_INIT_STARTING:
        if timedEnd:
            walk.setTarget(y=0.7)
            set_state(AUTO_READY_POSITIONING, ready_time)
    
    elif state == AUTO_READY_POSITIONING:
        if timedEnd:
            walk.setTarget(yaw=0.5)
            set_state(AUTO_READY_TURNING, 3)

    elif state == AUTO_READY_TURNING:
        if timedEnd:
            walk.setTarget()
            set_state(AUTO_READY_POST, 1 )

    elif state == AUTO_READY_POST:
        if timedEnd:
            set_state(WALK_STOPING, 1)
    
    elif state == AUTO_PLAY:
        setwalkcmd("start")
        time_play_start = time
        set_state(WALK_INIT, play_delay)

    elif state == PAUSE:
        if timedEnd:
            return
    
    elif state == START_YAW_COMPE:
        if timedEnd:
            set_yaw_compe_start = yaw
            # set_yaw_compe_start_time = time
            set_state(END_YAW_COMPE, 10)
    
    elif state == END_YAW_COMPE:
        if timedEnd:
            # diff_time = time - set_yaw_compe_start_time
            diff_yaw = yaw - set_yaw_compe_start
            odo_10min_dev = -diff_yaw*60
            set_state(PAUSE)
    
    elif state == WALK_2_STANDUP_INIT:
        standing_up = True
        if timedEnd:
            setwalkcmd("stop")
            set_state(WALK_2_STANDUP_INIT_2, 1)

    elif state == WALK_2_STANDUP_INIT_2:
        if timedEnd:
            enableAction()
            set_state(ACTION_2_STANDUP_INIT, 1)

    elif state == ACTION_2_STANDUP_INIT:
        standing_up = True
        if timedEnd:
            if pitch < -500:
                playAction(21)
                set_state(STANDING_UP, 15)
            else:
                # playAction(22)
                set_state(PAUSE, 30)

    elif state == STANDING_UP:
        if timedEnd:
            stopAction() 
            set_state(STANDUP_POST, 1)

    elif state == STANDUP_POST:
        if timedEnd:
            standing_up = False
            enableWalk()
            set_state(WALK_INIT, 2)

    elif state == ROBOT_AVOID_INIT:
        if timedEnd:
            setwalkcmd("start")
            walk.setTarget()
            bt.isEnabled = False
            data_testing = True

            # acumulate avg
            for d in dets.robots:
                robot_dir_acu.append(d)
            set_state(ROBOT_AVOID_INIT_AVG, 3)

    elif state == ROBOT_AVOID_INIT_AVG:
        if timedEnd:
            # calculate avg
            avg = 0
            for d in robot_dir_acu:
                avg += d
            avg /= len(robot_dir_acu)
            avg /= 480
            avg -= 0.5
            # decide walk left or right
            yaw_control = plus_or_min(avg, 0.45)
            print("yaw_control_ra : "+str(yaw_control))
            # excecute
            walk.setTarget(0.0, 0.6, yaw_control)
            # store direction
            robot_avoid_dir = yaw_control
            set_state(ROBOT_AVOID_EVADE, 10)
    
    elif state == ROBOT_AVOID_EVADE:
        if timedEnd:
            # load stored dir
            walk.setTarget(0.0, 0.6, -robot_avoid_dir)
            # negative target
            set_state(ROBOT_AVOID_REALIGN, 10)
    
    elif state == ROBOT_AVOID_REALIGN:
        if timedEnd:
            # straghten
            walk.setTarget()
            # back to main state
            set_state(WALK_STOP_INIT, 3)
    ##
    elif state == TESTING_SPEED_PRE:
        if timedEnd:
            bt.isEnabled = False
            data_testing = True
            enableActionOnly()
            set_state(PAUSE, 4)
    
    elif state == TESTING_SPEED_INIT:
        if timedEnd:
            # load stored dir
            playAction(head_speed_idx)
            # negative target
            set_state(TESTING_SPEED_POST, 10)
    
    elif state == TESTING_SPEED_POST:
        if timedEnd:
            # straghten
            stopAction()
            # back to main state
            set_state(PAUSE)
    

    ####
    if((pitch > 400 or pitch < -700) and not standing_up):
        if actionEnabled:
            set_state(PAUSE)
        else:
            set_state(WALK_2_STANDUP_INIT)

def update_odo():
    global yaw
    yaw += 0.05 * walk.vectorCurrent.yaw

    if yaw < -odo_min_max: yaw = (odo_min_max*2) + yaw
    elif yaw > odo_min_max: yaw = (odo_min_max*-2) + yaw