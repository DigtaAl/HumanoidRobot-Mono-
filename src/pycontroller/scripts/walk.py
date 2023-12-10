#!/usr/bin/env python3

import os
import signal
import sys

import rospy
import time
import json

import websockets
import asyncio

# websocket
import threading
import inference3 as inference
import ball_tracking
import goaltracker

from configloader import read_walk_balance_conf, read_ball_track_conf, read_walking_conf

from walking import Vector2, Vector2yaw, CONTROL_MODE_HEADLESS, CONTROL_MODE_YAWMODE, Walking
from walk_utils import joints, getWalkParamsDict, setWalkParamsConvert


from std_msgs.msg import String, Int32, Bool
from robotis_controller_msgs.msg import SyncWriteItem, StatusMsg, SensorYPR
from op3_walking_module_msgs.msg import WalkingParam, WalkingCorrection
from sensor_msgs.msg import Imu, JointState
from op3_walking_module_msgs.srv import GetWalkingParam
from op3_action_module_msgs.srv import IsRunning
from robotis_controller_msgs.srv import LoadOffset

import DATA as striker
    
###############################################################################

pubSWI = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=10)
pubBT = rospy.Publisher('/pycontroller/init', String, queue_size=10)
pubEnaMod = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
pubWalkCmd = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
pubSetParams = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=10)
pubWalkCorr = rospy.Publisher('walking_correction', WalkingCorrection, queue_size=10)
pubHeadControl = rospy.Publisher('/robotis/head_control/set_joint_states', JointState, queue_size=1)
pubMotionIndex = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
pubEnableOffset = rospy.Publisher('/robotis/enable_offset', Bool, queue_size=10)

OFFSET_PATH = "/home/name/agen3/src/ROBOTIS-OP3/op3_tuning_module/data/offset.yaml"

currentWalkParams = None # ? params
walkParams = None

server = None

robotIsOn = False
walking_module_enabled = False

walking = Walking()
track_ball = inference.Tracking()

dets = inference.Detection()

goaltracker.frame_size = (inference.streamer.frame_width, inference.streamer.frame_height)

clients = {}

SEND_PARAM_INTERVAL = 0.03
lastSendParamTic = time.time()

isManagerReady = False

connected_clients = {}

server_loop = None

head_control = [0,0] # pitch,yaw
movement = [0,0] # move, yaw

is_walking = False

gc_state = "initial"
enable_gc = 0

async def ws_handler(websocket, path):
    global enable_gc
    client_id = id(websocket)
    connected_clients[client_id] = websocket


    print(client_id, 'connected')
    send_message(client_id, "device_connected", client_id)
    send_message(client_id, "torque_control", robotIsOn)

    try:
        while True:
            message = await websocket.recv()
            data = json.loads(message)
            # print(data)
            cmd = data['cmd']

            if cmd == 'torque_on':
                startRobot()
            elif cmd == 'torque_off':
                setDxlTorque()
                send_message(-1, "torque_control", False)
            elif cmd == 'start_walk':
                walk_toggle(True)
            elif cmd == 'stop_walk':
                walk_toggle(False)
            elif cmd == 'save_walk_params':
                setWalkCmd("save")
                send_message(-1, "walk_params_saved", True)
            elif cmd == 'get_walk_params':
                send_message(-1, "update_walk_params", getWalkParams())
            elif cmd == 'set_walk_params':
                setWalkParams(data['params'])
                send_message(-1, "controller_msg", 'Walk params changed')
            elif cmd == "set_walking":
                if(client_id == walking.control):
                    vectorDict = data['params']
                    vector = Vector2yaw(vectorDict["x"], vectorDict["y"], vectorDict["yaw"])
                    walking.setTarget(vector)
            elif cmd == 'set_walking_offset':
                walking.setWalkingOffset()
                send_message(-1, "controller_msg", 'Walking offset changed')
            elif cmd == 'set_walking_conf':
                walking.setWalkingConf(data['params'])
                send_message(-1, "controller_msg", 'Walking configuration changed')
            elif cmd == "set_control_walking":
                if data['params'] == 1:
                    print("walking controll has been set!!")
                    walking.control = client_id
                    send_message(-1, "control_override", client_id)
                else:
                    walking.control = None
                    send_message(-1, "control_override", -1)
            elif cmd == 'get_walking_conf':
                send_message(-1, "update_walking_conf", walking.getWalkingConf())
            elif cmd == 'get_walking':
                send_message(client_id, "update_walking", walking.getWalkingConf())
            elif cmd == 'gyro_init':
                init_gyro()
                send_message(-1, "controller_msg", "Init gyro success")
            elif cmd == 'head_direct':
                headControlDirect(data['params'])
            elif cmd == 'track_head_control':
                headControlHandle(data['params'])
            elif cmd == 'edit_head_pid':
                headPIDHandle(data['params'])
            elif cmd == 'stream_offer':
                streamOfferHandle(data['params'], client_id)
            elif cmd == 'reload_ball_track_conf':
                reloadBallTrackerHandle()
            elif cmd == "load_offset":
                loadOffsetHandle(OFFSET_PATH)
            elif cmd == "load_walking_conf":
                handleLoadWalkingConf()
            elif cmd == "enable_ball_track": 
                walking.isEnabled = True
                send_message(-1, "controller_msg", "Ball tracking ENABLED")
            elif cmd == "disable_ball_track":
                walking.isEnabled = False
                send_message(-1, "controller_msg", "Ball tracking disabled")
            elif cmd == 'start_goal_track':
                handleGoalTrack()
            elif cmd == 'save_goal_track':
                saveGoalTrack(data['params'])
            elif cmd == 'shoot_trigger':

                striker.set_state(striker.AUTO_PLAY)
            elif cmd == 'set_state':
                striker.set_state(int(data['params']))
            elif cmd == 'set_ready_time':
                striker.ready_time = float(data['params'])
            elif cmd == 'set_play_delay':
                striker.play_delay = float(data['params'])
            elif cmd == 'set_yaw_ball_dev_multipler':
                striker.yaw_ball_dev_multipler = float(data['params'])
            elif cmd == 'set_pitch_ball_dev_multipler':
                striker.pitch_ball_dev_multipler = float(data['params'])
            elif cmd == 'enable_gc':
                enable_gc = int(data['params'])
            elif cmd == 'reset_yaw':
                striker.yaw = 0.0

            elif cmd == 'yaw_compe_ball_align':
                striker.pitch_ball_dev_multipler = float(data['params'])
            if cmd == 'time_multi_goal_align_yaw':
                striker.pitch_ball_dev_multipler = float(data['params'])
            elif cmd == 'time_multi_goal_align':
                striker.pitch_ball_dev_multipler = float(data['params'])
            elif cmd == 'yaw_x_turning_max':
                striker.pitch_ball_dev_multipler = float(data['params'])

            elif cmd == 'enable_goal_det':
                striker.enable_goal_det = bool(data['params'])
            if cmd == 'enable_ball_align':
                striker.enable_ball_align = int(data['params'])
            
            elif cmd == "odo_deviation":
                striker.odo_deviation = float(data['params'])
            elif cmd == "set_compe":
                striker.set_compe()

            elif cmd == "testing_speed_idx":
                striker.head_speed_idx = int(data['params'])
                striker.set_state(striker.TESTING_SPEED_INIT)
            elif cmd == "testing_speed":
                striker.set_state(striker.TESTING_SPEED_PRE)
            elif cmd == "robot_avoid":
                striker.set_state(striker.ROBOT_AVOID_INIT)


    finally:
        del connected_clients[client_id]
        print(client_id, 'closed')
        send_message(-1, "device_disconnected", client_id)

def between_callback():
    global server_loop

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    server_loop = loop
    ws_server = websockets.serve(ws_handler, '0.0.0.0', 8077)

    loop.run_until_complete(ws_server)
    loop.run_forever() # this is missing
    loop.close()

async def send_message2(websocket_con, message):
    await websocket_con.send(message)

def send_message(id, cmd, params):
    if server == None: return

    resp = {
        "cmd" : cmd,
        "params" : params
    }
    
    respJson = json.dumps(resp)
    if(id >= 0):
        asyncio.run_coroutine_threadsafe(send_message2(connected_clients[id], respJson), server_loop)
    else:
        for client in connected_clients.values():
            asyncio.run_coroutine_threadsafe(send_message2(client, respJson), server_loop)

def sendHeadControl():
    js = JointState()
    js.name.append("head_tilt")
    js.name.append("head_pan")
    js.position.append(head_control[0])
    js.position.append(head_control[1])
    pubHeadControl.publish(js)

head_control_once = False
def sendHeadControlOnce():
    global head_control_once
    if not head_control_once:
        head_control_once = True
        sendHeadControl()

def sendWithWalkParams():
    if not striker.enabled: return
    if is_walking == False: return
    global walkParams
    global pubSetParams
    walkParams.x_move_amplitude = walking.vectorCurrent.y
    walkParams.angle_move_amplitude = walking.vectorCurrent.yaw
    walkParams.y_move_amplitude = walking.vectorCurrent.x
    
    pubSetParams.publish(walkParams)
    # print("params set")
    # send_message(-1, "update_walking", walking.getWalkingCurrent())

def enableWalk():
    pubEnaMod.publish("walking_module")
    pubEnaMod.publish("head_control_module")

def setDxlTorque(): # list comprehension
    global robotIsOn

    isTorqueOn = False

    if robotIsOn == False:
        return
    robotIsOn = False

    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "torque_enable"
    for joint_name in joints:
        if((not isTorqueOn) and (joint_name == "head_pan" or joint_name == "head_tilt")):
            continue
        syncwrite_msg.joint_name.append(joint_name)
        syncwrite_msg.value.append(isTorqueOn) 

    pubSWI.publish(syncwrite_msg)

def initGyro():
    syncwrite_msg = SyncWriteItem()
    syncwrite_msg.item_name = "imu_control"
    syncwrite_msg.joint_name.append("open-cr")
    syncwrite_msg.value.append(8)

    pubSWI.publish(syncwrite_msg)

def startRobot():
    global robotIsOn
    if robotIsOn:
        return
    print("pub long")
    robotIsOn = True
    pubBT.publish("user_long")

def headControlDirect(data):
    global head_control
    head_control[0] = data["pitch"]
    head_control[1] = data["yaw"]

def headControlHandle(data):
    if(data["enabled"] == True):
        ball_tracking.isEnabled = True
    elif(data["enabled"] == False):
        ball_tracking.isEnabled = False

def handleGoalTrack():
    goaltracker.enabled = True

def saveGoalTrack(param):
    goal = goaltracker.goal
    theta = json.dumps(goal.theta.tolist())
    grad = str(goal.grad)
    span = str(goal.span)


def reloadBallTrackerHandle():
    ball_tracking.x_p = read_ball_track_conf("PID", "x_p")
    ball_tracking.y_p = read_ball_track_conf("PID", "y_p")
    ball_tracking.x_i = read_ball_track_conf("PID", "x_i")
    ball_tracking.y_i = read_ball_track_conf("PID", "y_i")
    ball_tracking.x_d = read_ball_track_conf("PID", "x_d")
    ball_tracking.y_d = read_ball_track_conf("PID", "y_d")
    ball_tracking.flip_x = read_ball_track_conf("PID", "flip_x")
    ball_tracking.flip_y = read_ball_track_conf("PID", "flip_y")
    ball_tracking.out_scale_x = read_ball_track_conf("PID", "out_scale_x")
    ball_tracking.out_scale_y = read_ball_track_conf("PID", "out_scale_y")
    ball_tracking.max_pitch = read_ball_track_conf("PID", "max_pitch")
    ball_tracking.min_pitch = read_ball_track_conf("PID", "min_pitch")
    ball_tracking.max_yaw = read_ball_track_conf("PID", "max_yaw")
    ball_tracking.min_yaw = read_ball_track_conf("PID", "min_yaw")
    ball_tracking.zero_offset_x = read_ball_track_conf("PID", "zero_offset_x")
    ball_tracking.zero_offset_y = read_ball_track_conf("PID", "zero_offset_y")

    ball_tracking.reload()

    send_message(-1, "controller_msg", "ball_tracking params updated!")

async def offering(request, client):
    print("processing offer...")
    
    answer = await inference.streamer.offer(request)
    if server == None: return
    print("ANSWER")
    print(answer)
    print(type(answer))
    
    await send_message2(connected_clients[client], json.dumps(answer))
    print("answer sent")

def streamOfferHandle(data, client):
    global server_loop
    asyncio.run_coroutine_threadsafe(offering(data, client), server_loop)

def headPIDHandle(data):
    ball_tracking.pid_x.tunings(data["p"], data["i"], data["d"])

def updateAngle():
    yaw = striker.yaw
    if yaw != striker.last_yaw:
        striker.last_yaw = yaw
    # send_message(-1, 'angle_update', yaw)

def setWalkCmd(walkCmd):
    global is_walking
    # sendWalkCorrectionConf()
    if walkCmd == "start": 
        is_walking = True
        send_message(-1, "walk_control", True)
    elif walkCmd ==  "stop": 
        is_walking = False
        send_message(-1, "walk_control", False)
    if walkCmd == "start" or walkCmd == "stop" or walkCmd == "balance on" or walkCmd == "balance off" or walkCmd == "save":
        print("walk : ", walkCmd)
        pubWalkCmd.publish(walkCmd)

def setWalkParams(param):
    global walkParams

    setWalkParamsConvert(walkParams, param)
    
    pubSetParams.publish(walkParams)

def getWalkParams():
    global currentWalkParams
    global walkParams
    rospy.wait_for_service('/robotis/walking/get_params')
    try:
        getParams = rospy.ServiceProxy('/robotis/walking/get_params', GetWalkingParam)
        resp = getParams()
        params = resp.parameters
        walkParams = params
        paramsDict = getWalkParamsDict(params)
        currentWalkParams = paramsDict
        return paramsDict
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def loadOffsetHandle(path):
    rospy.wait_for_service('/robotis/load_offset')
    try:
        getParams = rospy.ServiceProxy('/robotis/load_offset', LoadOffset)
        resp = getParams(path)
        send_message(-1, "offset_updated", resp.result)
        if resp.result:
            print("offset_updated")
        else:
            print("offset_fail")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def handleLoadWalkingConf():
    walking.max_speed = read_walking_conf("common", "max_speed")
    walking.feed_rate = read_walking_conf("common", "feed_rate")
    walking.stationary_offset.x = read_walking_conf("stationary_offset", "x")
    walking.stationary_offset.y = read_walking_conf("stationary_offset", "y")
    walking.stationary_offset.yaw = read_walking_conf("stationary_offset", "yaw")
    walking.step.x = read_walking_conf("step", "x")
    walking.step.y = read_walking_conf("step", "y")
    walking.step.yaw = read_walking_conf("step", "yaw")
    walking.vectorMultiplier.x = read_walking_conf("vector_multipler", "x")
    walking.vectorMultiplier.y = read_walking_conf("vector_multipler", "y")
    walking.vectorMultiplier.yaw = read_walking_conf("vector_multipler", "yaw")

    send_message(-1, "controller_msg", "walking params updated!")
        

def init_gyro():
    init_gyro_msg = SyncWriteItem()
    init_gyro_msg.item_name = "imu_control"
    init_gyro_msg.joint_name.append("open-cr")
    init_gyro_msg.value.append(8)
    pubSWI.publish(init_gyro_msg)

imu = Imu()
sensor_ypr = SensorYPR()

def handleImu(imu_msg_):
    global imu
    imu = imu_msg_

def handleBalanceMonitor(msg):
    send_message(-1, "balance_monitor", msg.data)


def sendWalkCorrectionConf():
    wc = WalkingCorrection()

    wc.wb_p_gain = read_walk_balance_conf("Balance", "wb_p_gain")
    wc.wb_i_gain = read_walk_balance_conf("Balance", "wb_i_gain")
    wc.wb_d_gain = read_walk_balance_conf("Balance", "wb_d_gain")
    wc.zero_pitch_offset = read_walk_balance_conf("Balance", "zero_pitch_offset")
    wc.pitch_offset_multiplier = read_walk_balance_conf("Balance", "pitch_offset_multiplier")
    wc.x_offset_multiplier = read_walk_balance_conf("Balance", "x_offset_multiplier")

    pubWalkCorr.publish(wc)

def handleStatusMsg(statusMsg):
    print(statusMsg.status_msg)
    print("status ^^^^^")
    if(statusMsg.status_msg == "Walking Enabled"):
        init_gyro()
        print("init gyro...")

    if(statusMsg.status_msg == "Finish Init Pose"):
        enableWalk()
        print("ENABLE WALK")
        send_message(-1, "torque_control", True)
        global isManagerReady
        isManagerReady = True

    statusDict = {
        'type':statusMsg.type,
        'module_name':statusMsg.module_name,
        'status_msg':statusMsg.status_msg
    }

    send_message(-1, 'update_status', statusDict)

def walk_toggle(_on):
    if not striker.initialized: return
    if _on:
        striker.set_state(striker.WALK_INIT_STARTING,  2.0)
    else: striker.set_state(striker.WALK_STOPING,  1.0)

def isActionRunning():
    rospy.wait_for_service('/robotis/action/is_running')
    try:
        getParams = rospy.ServiceProxy('/robotis/action/is_running', IsRunning)
        resp = getParams()
        global action_status
        action_status = resp.is_running
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def gamestate_callback(data):
    global gc_state
    new_gc_state = data.data
    print("GC COMMAND in :"+new_gc_state)
    if(new_gc_state == "play" and new_gc_state != gc_state):
        striker.set_state(striker.AUTO_PLAY)
        gc_state = new_gc_state
    elif(new_gc_state == "ready" and new_gc_state != gc_state):
        striker.set_state(striker.AUTO_READY_INIT_STARTING, 3)
        gc_state = new_gc_state

def button_callback(data):
    if data.data == "start":
        print("start pressed")
        if striker.state == striker.PAUSE:
            striker.set_state(striker.AUTO_PLAY)
        else: walk_toggle(False)
    elif data.data == "start_long":
        print("start_long pressed")
        if robotIsOn: setDxlTorque()
        else: startRobot()
    elif data.data == "mode":
        print("mode pressed")
        striker.zero_ypr()
    elif data.data == "mode_long":
        # striker.zero_ypr()
        print("mode_long pressed")
    else:
        print("unhandled button pressed")

def main():
    global server
    global head_control

    rospy.init_node('main', anonymous=True)

    rospy.Subscriber("/robotis/status", StatusMsg, handleStatusMsg)
    rospy.Subscriber("sensor_ypr", SensorYPR, striker.set_ypr)
    # rospy.Subscriber("gamestate", String, gamestate_callback)
    rospy.Subscriber("/robotis/open_cr/button2", String, button_callback)

    rospy.loginfo("Waiting manager...")
    count = 0
    while(isManagerReady==False and count < 100): 
        time.sleep(1)
        print(count)
        count +=1

    server = threading.Thread(target=between_callback, daemon=True)
    server.start()
    rospy.loginfo("ws server starting...")

    time.sleep(10)

    print("controller runnning")
    rospy.loginfo("Wait for manager complete")
    getWalkParams()
    startRobot()
    print(" ")
    time.sleep(10)

    reloadBallTrackerHandle()
    handleLoadWalkingConf()

    print("finish")

    striker.init(goaltracker, inference, ball_tracking, walking, setWalkParams, setWalkCmd)
    striker.init_action(pubMotionIndex, isActionRunning, pubEnaMod, pubEnableOffset)

    if(inference.startInference() < 0):
        rospy.loginfo("Inference Start ERROR")
        raise SystemExit('ERROR: failed to start inference!')
    else:
        rospy.loginfo("Inference Started")

        setWalkParams(["balance_enable", 1])

    global lastSendParamTic
    global track_ball
    global dets

    last_goal_scan = 0.0
    goal_scan_interval = 5.0 

    while not rospy.is_shutdown():
        # try:
            toc = time.time()
            delta_t = toc - lastSendParamTic

            inference.detect(track_ball, dets)

            if(toc - last_goal_scan >= goal_scan_interval):
                # goaltracker.enabled = not goaltracker.enabled
                last_goal_scan = toc

            striker.run(toc, head_control, dets)

            if(goaltracker.enabled):
                goaltracker.scan(dets)

                if(goaltracker.state == goaltracker.SCAN_DONE):
                    goaltracker.enabled = False
                    statusDict = {
                        'dets': goaltracker.unclustered_goals,
                        'center': goaltracker.goal.theta.tolist(),
                        'found': goaltracker.goal.found
                    }
                    send_message(-1, 'goal_scan_update', statusDict)
                elif goaltracker.state == goaltracker.SCAN_DONE_POST:
                    head_control[0] = goaltracker.pre_head_pos[0]
                    head_control[1] = goaltracker.pre_head_pos[1]
                else:
                    head_control[0] = goaltracker.scan_tilt 
                    head_control[1] = goaltracker.current_pos

            elif ball_tracking.isEnabled:
                if inference.ball_lock:
                    if(ball_tracking.searching):
                        ball_tracking.searching = False
                        ball_tracking.set_py_from_buff()
                        

                    ball_tracking.track(track_ball)
                else:
                    ball_tracking.searching = True
                    ball_tracking.search(toc)

                head_control[0] = ball_tracking.pitch
                head_control[1] = ball_tracking.yaw

            elif striker.data_testing:
                head_control[0] = 0.4
                head_control[1] = 0.0
                # sendHeadControlOnce()

            if not striker.data_testing:
                sendHeadControl()

            if(delta_t > SEND_PARAM_INTERVAL):
                lastSendParamTic = toc
                walking.stepToTargetVel()

                updateAngle()
                sendWithWalkParams()
            
    
            


def shutdown():
    global server
    inference.shutdown()
    server.join()
    sys.exit()

def close_sig_handler(signal, frame):
    shutdown()

signal.signal(signal.SIGINT, close_sig_handler)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown()
