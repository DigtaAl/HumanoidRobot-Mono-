joints = [
    "head_pan",
    "head_tilt",
    "l_ank_pitch",
    "l_ank_roll",
    "l_el",
    "l_hip_pitch",
    "l_hip_roll",
    "l_hip_yaw",
    "l_knee",
    "l_sho_pitch",
    "l_sho_roll",
    "r_ank_pitch",
    "r_ank_roll",
    "r_el",
    "r_hip_pitch",
    "r_hip_roll",
    "r_hip_yaw",
    "r_knee",
    "r_sho_pitch",
    "r_sho_roll"
]

def setWalkParamsConvert(walkParams, param):

    paramName = param[0]
    paramValue = param[1]
    print("params changing:")
    print(param[0])
    print(param[1])

    if paramName == "init_x_offset" : walkParams.init_x_offset = paramValue
    elif paramName == "init_y_offset" : walkParams.init_y_offset = paramValue
    elif paramName == "init_z_offset" : walkParams.init_z_offset = paramValue
    elif paramName == "init_roll_offset" : walkParams.init_roll_offset = paramValue
    elif paramName == "init_pitch_offset" : walkParams.init_pitch_offset = paramValue
    elif paramName == "init_yaw_offset" : walkParams.init_yaw_offset = paramValue
    elif paramName == "period_time" : walkParams.period_time = paramValue
    elif paramName == "dsp_ratio" : walkParams.dsp_ratio = paramValue
    elif paramName == "step_fb_ratio" : walkParams.step_fb_ratio = paramValue
    elif paramName == "x_move_amplitude" : walkParams.x_move_amplitude = paramValue
    elif paramName == "y_move_amplitude" : walkParams.y_move_amplitude = paramValue
    elif paramName == "z_move_amplitude" : walkParams.z_move_amplitude = paramValue
    elif paramName == "angle_move_amplitude" : walkParams.angle_move_amplitude = paramValue
    elif paramName == "move_aim_on" : walkParams.move_aim_on = paramValue
    elif paramName == "balance_enable" : walkParams.balance_enable = paramValue
    elif paramName == "balance_hip_roll_gain" : walkParams.balance_hip_roll_gain = paramValue
    elif paramName == "balance_knee_gain" : walkParams.balance_knee_gain = paramValue
    elif paramName == "balance_ankle_roll_gain" : walkParams.balance_ankle_roll_gain = paramValue
    elif paramName == "balance_ankle_pitch_gain" : walkParams.balance_ankle_pitch_gain = paramValue
    elif paramName == "y_swap_amplitude" : walkParams.y_swap_amplitude = paramValue
    elif paramName == "z_swap_amplitude" : walkParams.z_swap_amplitude = paramValue
    elif paramName == "arm_swing_gain" : walkParams.arm_swing_gain = paramValue
    elif paramName == "pelvis_offset" : walkParams.pelvis_offset = paramValue
    elif paramName == "hip_pitch_offset" : walkParams.hip_pitch_offset = paramValue
    elif paramName == "p_gain" : walkParams.p_gain = paramValue
    elif paramName == "i_gain" : walkParams.i_gain = paramValue
    elif paramName == "d_gain" : walkParams.d_gain = paramValue

def getWalkParamsDict(params):
    paramsDict = {
            "init_x_offset" : params.init_x_offset,             
            "init_y_offset" : params.init_y_offset,
            "init_z_offset" : params.init_z_offset,
            "init_roll_offset" : params.init_roll_offset,
            "init_pitch_offset" : params.init_pitch_offset,
            "init_yaw_offset" : params.init_yaw_offset,
            "period_time" : params.period_time,
            "dsp_ratio" : params.dsp_ratio,
            "step_fb_ratio" : params.step_fb_ratio,
            "x_move_amplitude" : params.x_move_amplitude,
            "y_move_amplitude" : params.y_move_amplitude,
            "z_move_amplitude" : params.z_move_amplitude,
            "angle_move_amplitude" : params.angle_move_amplitude,
            "move_aim_on" : params.move_aim_on,
            "balance_enable" : params.balance_enable,
            "balance_hip_roll_gain" : params.balance_hip_roll_gain,
            "balance_knee_gain" : params.balance_knee_gain,
            "balance_ankle_roll_gain" : params.balance_ankle_roll_gain,
            "balance_ankle_pitch_gain" : params.balance_ankle_pitch_gain,
            "y_swap_amplitude" : params.y_swap_amplitude,
            "z_swap_amplitude" : params.z_swap_amplitude,
            "arm_swing_gain" : params.arm_swing_gain,
            "pelvis_offset" : params.pelvis_offset,
            "hip_pitch_offset" : params.hip_pitch_offset,
            "p_gain" : params.p_gain,
            "i_gain" : params.i_gain,
            "d_gain" : params.d_gain        
    }
    return paramsDict