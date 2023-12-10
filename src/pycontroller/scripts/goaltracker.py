import time
import numpy as np

SEARCHING = 0
FOUND = 1
LOST = 2
SCANING_PRE_HOLD = 7
SCANING_HOLD_PAN = 3
SCANING_POST_HOLD = 6
SCANING_PANING = 4
SCAN_DONE = 5
SCAN_DONE_POST = 8

state = SEARCHING

horizon_angle = 0.6

#    0
# 1.5 x -1.5
#  -2/2

fov = 0.85
offset_x = -0.40

direction = 0 
head_pan = 0
scan_tilt = 0.6
scan_subdivision = 6

min_scan = -1.5 # -90 deg
max_scan = 1.5
scan_pos = -1

scan_step = (max_scan - min_scan)/scan_subdivision

#searching param
interval_pan = 0.2 #sec
hold_pan = 0.1 #sec
pad_pan = 0.05 #sec
lastTic = 0

post_done_time = 1

current_pos = 0

frame_size = None

head_target_x = 0.0
head_target_y = 0.0


unclustered_goals = []

pre_head_pos = [0.0,0.0]

enabled = False

class Goal:
    def __init__(self):
        self.theta = np.array([0.0, 0.0])  #which angle goal detected
        self.grad = 0.0  #left-right gradient
        self.span = 1.0  #left-right span 
        self.found = False
        self.left = None
        self.right = None
    def setall(self, _theta, _grad, _span, _found):
        self.theta = _theta
        self.grad = _grad
        self.span = _span
        self.found = _found
    def setall2(self, _left, _right, _found):
        self.found = _found
        self.left = _left
        self.right = _right

goal = Goal()

def set_pre_head_pos(_head_pos):
    global pre_head_pos
    pre_head_pos[0] = _head_pos[0]
    pre_head_pos[1] = _head_pos[1]

def scan(dets):
    in_goals = dets.goals
    global lastTic 
    global scan_pos
    global current_pos
    global state
    global unclustered_goals
    toc = time.time()
    delta_t = toc - lastTic

    if(state == SCANING_HOLD_PAN):
        if(delta_t > hold_pan):
            lastTic = toc
            state = SCANING_POST_HOLD
            
            found = len(in_goals)
            print("goals : "+str(found)+" "+str(current_pos))
            if(found > 0 and scan_pos != 0):
                for goal in in_goals:
                    goalPosInFrame = (goal[0]/frame_size[0]*fov-(fov/2))*-1
                    goal = (current_pos+goalPosInFrame+offset_x, goal[1]/frame_size[1])
                    unclustered_goals.append(goal) 
            if scan_pos >= scan_subdivision:
                state = SCAN_DONE_POST
                lastTic = toc
            #insert
                if(len(unclustered_goals) > 0):
                    track(unclustered_goals)
            scan_pos+=1
        # else: dets.goals = []
    elif(state == SCANING_PANING):
        if(delta_t > interval_pan):
            lastTic = toc
            state = SCANING_PRE_HOLD
    elif(state == SCANING_PRE_HOLD):
        if(delta_t > pad_pan):
            lastTic = toc
            dets.goals = []
            state = SCANING_HOLD_PAN
            current_pos = (scan_step*scan_pos+min_scan)
    elif(state == SCANING_POST_HOLD):
        if(delta_t > pad_pan):
            lastTic = toc
            state = SCANING_PANING
    elif(state == SCAN_DONE_POST):
        if(delta_t > post_done_time):
            state = SCAN_DONE
            scan_pos = 0
    else:
        state = SCANING_PRE_HOLD
        unclustered_goals = []
        


def track(unclustered_goals):
    global goal 
    np_goals = np.array(unclustered_goals)
    max_goal_x, max_goal_y = np_goals.max(axis=0)
    min_goal_x, min_goal_y = np_goals.min(axis=0)
    delta_x = max_goal_x - min_goal_x
    mid = delta_x/2 + min_goal_x

    
    left = []
    right = []

    left_count = 0
    right_count = 0

    for goal_i in unclustered_goals:
        if goal_i[0] > mid:
            left.append(goal_i)
            left_count += 1
        else:
            right.append(goal_i)
            right_count += 1

    print(left)
    print(right)

    if len(right)>0 and len(left)>0:
        mean_left = np.mean(np.array(left), axis=0)
        mean_right = np.mean(np.array(right), axis=0)

        center = (mean_left-mean_right)/2+mean_right
        print("center:")
        print(center)

        # goal.setall2(mean_right.item(1), mean_left.item(1), True)
        goal.theta = center

        delta = mean_right-mean_left

        goal.found = True
        goal.grad = delta[1]
        goal.span = delta[0]
        print("span: "+str(goal.span))
        print("grad: "+str(goal.grad))

    else: goal.found = False 
