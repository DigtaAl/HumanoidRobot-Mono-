#!/usr/bin/env python3

# -1 to 1
# center is 0,0
# right, up = +, -

from yolo.iyolo import get_yolo, CLASSES

import math

import yolo.streamer as streamer

#DEF YOLO START

videocap = None

res = None
YOLO = None

lost_count = 0
ball_lock = False

max_lost = 30
goto_zero_x = 0.01
goto_zero_y = 0.01
stop_zone = 0.20

class Tracking:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.m = 100.0

    def set(self, tar):
        self.x = tar.x
        self.y = tar.y
        self.m = tar.m

class Detection:
    def __init__(self):
        self.balls = []
        self.goals = []
        self.robots = []

def detect(track_ball, d):

    global videocap

    img = videocap.read_in()

    dets, draw = YOLO.infer(img)

    is_found = False
    d_closest = 1000.0
    closest = Tracking()

    detected_goals = []
    d.robots = []

    if len(dets) > 0:
        for box in dets:
            cl = int(box[4])
            x = int((box[0] + box[2]) / 2)
            y = int((box[1] + box[3]) / 2)
            if cl == 1: #goal
                detected_goals.append([x, max(box[3], box[1])])
            elif cl == 2: # robot
                d.robots.append(x)
            else:
                
                x = (x / res[0] - 0.5) * 2
                y = (y / res[1] - 0.5) * 2
                
                d_x = x - track_ball.x
                d_y = y - track_ball.y

                dist = math.sqrt((d_x*d_x) + (d_y*d_y))

                if dist < d_closest:
                    d_closest = dist
                    closest.x = x
                    closest.y = y
                    is_found = True

    d.goals += detected_goals

    global ball_lock
    global lost_count

    if is_found:
        track_ball.x = closest.x
        track_ball.y = closest.y
        lost_count = 0
        ball_lock = True
    else:
        lost_count += 1

        if track_ball.x < -stop_zone:
            track_ball.x += goto_zero_x
        if track_ball.x > stop_zone:
            track_ball.x -= goto_zero_x
        if track_ball.y < -stop_zone:
            track_ball.y += goto_zero_y
        if track_ball.y > stop_zone:
            track_ball.y -= goto_zero_y    

        if lost_count > max_lost:
            lost_count = max_lost
            ball_lock = False
            track_ball.x = 0.0
            track_ball.y = 0.0

    videocap.store_out(draw)


def startInference():

    global videocap
    global YOLO
    global res

    YOLO = get_yolo("nas")

    videocap = streamer.VideoCapture()
    track = streamer.VideoOpencvTrack(videocap)
    streamer.video = track

    res = track.get_frame_size()


    print('inference is running...')
    return 0

def inferenceLoop(track_ball):
    while(True):
        detect(track_ball)


def shutdown():
    if videocap != None:
        videocap.release()