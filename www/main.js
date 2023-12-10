ws = new WebSocket("ws://"+ window.location.href.split('/')[2] +":8077");

var con_h1 = document.getElementById("con_h1");
var con_log = document.getElementById("log_container");
var voltage_badge = document.getElementById("voltage_badge");
var bt_torque_toggle = document.querySelector("#bt_torque_toggle");
var bt_walk_toggle = document.querySelector("#bt_walk_toggle");
var bt_get_walk_params = document.querySelector("#bt_get_walk_params");
var bt_show_settings = document.querySelector("#bt_show_settings");
var bt_show_manuctrl = document.querySelector("#bt_show_manuctrl");
var bt_show_headctrl = document.querySelector("#bt_show_headctrl");
var bt_show_pidmon = document.querySelector("#bt_show_pidmon");
var bt_vision_stream = document.querySelector("#bt_vision_stream");
var bt_send_stationary = document.querySelector("#bt_send_stationary");
var bt_submit_settings = document.querySelector("#bt_submit_settings");
var bt_load_offset = document.querySelector("#bt_load_offset");
$("#"+bt_send_stationary.id).collapse("hide");

var input_walking_turnspeed = document.querySelector("#set_walking_turnspeed");
var input_walking_Accel = document.querySelector("#set_walking_Accel");
var input_walking_multipler = document.querySelector("#set_walking_multipler");
var input_walking_turnrate = document.querySelector("#set_walking_turnrate");


var isWalkParamsLoaded = false;

var isTorqueOn = false;
var isWalking = false;
var selfID = -1;
var hasControlID = -1;

var pidMonGraph =new GraphMonitor(document.getElementById("pid_mon_graph"));
pidMonGraph.build();
var goal_detection =new GoalDetection(document.getElementById("goal_detection"));
goal_detection.build();
var yaw_dir_vis =new YawDirVis(document.getElementById("yaw_direction"));
yaw_dir_vis.build();

yaw_dir_vis.set_angle(0);

var field_element = document.getElementById("field");
var field = new Field(field_element);
field.build();

// goal_detection.addDet(1);
// goal_detection.addDet(-1);
// goal_detection.addDet(0);

document.addEventListener("keydown", (event) => {
    if (event.isComposing || event.keyCode === 79) {
        console.log("masuk2")
        onStartGoalTrack();
    }
    else if (event.isComposing || event.keyCode === 81) {
        console.log("masuk18")
        onSaveGoalTrack();
    }
});


var pc = null;

var cmdTimeout = 3;

const CMD_TORQUE_TOGGLE = 1;
const CMD_WALK_TOGGLE = 2;
const CMD_CONTROL_OVERRIDE = 3;
const CMD_UPDATE_WAlK_PARAMS = 4;

const TURN_MODE_YAW = "dropdown_turn_mode_yaw";
const TURN_MODE_HEADLESS = "dropdown_turn_mode_headless";
const CONTROL_MODE_OFFSET = "dropdown_control_mode_offset";
const CONTROL_MODE_CONTROL = "dropdown_control_mode_control";
const CONTROL_MODE_OFF = "dropdown_control_mode_off";


var controlTurnMode = TURN_MODE_YAW;
var controlMode = CONTROL_MODE_OFF;

var modal = {
    container : document.querySelector("#modalMaster"),
    title : document.querySelector("#modalLabel"),
    body : document.querySelector("#modalBody"),
    inputNo : document.querySelector("#inputModal1"),
    inputYes : document.querySelector("#inputModal2"),
    showConfirm : function(title, body, callback){
        this.title.innerHTML = title;
        this.body.innerHTML = body;
        this.inputNo.innerHTML = "Cancel";
        this.inputYes.innerHTML = "Confirm";
        this.inputYes.setAttribute("onclick", callback);
        $('#'+this.container.id).modal("show");
    },
    close : function(){ 
    }
}

// modal.showConfirm("anu","woa","handleTorqueToggleUpdate(1)");

var alert = {
    TYPE_WARN : 1,
    TYPE_DANGER : 2,
    TYPE_SUCCESS : 4,
    TYPE_INFO : 3,

    warning : document.querySelector("#alert_success"),
    danger : document.querySelector("#alert_danger"),
    success : document.querySelector("#alert_warning"),
    info : document.querySelector("#alert_info"),

    timeOut: 5000,
    
    show : function(type, msg){
        let cont = this.info;
        switch(type){
            case this.TYPE_WARN:
                cont = this.warning;
                break;
            case this.TYPE_SUCCESS:
                cont = this.success;
                break;
            case this.TYPE_DANGER:
                cont = this.danger;
                break;
            default:
                cont = this.info;
        }

        cont.innerHTML = "<strong>"+msg+"</strong>";
        $('#'+cont.id).show();
        setTimeout(function() {
            $('#'+cont.id).fadeOut();
        }, this.timeOut);
    },
    close : function(){
        
    },
    init : function(){
        $("#"+this.warning.id).hide();
        $("#"+this.success.id).hide();
        $("#"+this.danger.id).hide();
        $("#"+this.info.id).hide();
    }
}

var Dropdown = function(keys, names, parent, callback){
    this.items = keys.map((id) => {
        let element = document.querySelector('#'+id);
        element.setAttribute("onclick", callback+"('"+id+"')");
        return element;
    });
    this.parent = document.querySelector('#'+parent);
    this.keys = keys;
    this.names = names;
    this.setActive = function(key){
        let index = this.keys.indexOf(key);
        let item = this.items[index];
        this.parent.innerHTML = names[index];
        for(let i = 0; i < this.items.length; i++){
            if(i === index) continue;
            $("#"+this.items[i].id).show();
        }
        $("#"+item.id).hide();
    }
};

alert.init();

var responsiveCmd = []

var paramElements = [
    $("#set_init_x_offset"),
    $("#set_init_y_offset"),
    $("#set_init_z_offset"),
    $("#set_init_roll_offset"),
    $("#set_init_pitch_offset"),
    $("#set_init_yaw_offset"),
    $("#set_period_time"),
    $("#set_dsp_ratio"),
    $("#set_step_fb_ratio"),
    $("#set_x_move_amplitude"),
    $("#set_y_move_amplitude"),
    $("#set_z_move_amplitude"),
    $("#set_angle_move_amplitude"),
    $("#set_move_aim_on"),
    $("#set_balance_enable"),
    $("#set_balance_hip_roll_gain"),
    $("#set_balance_knee_gain"),
    $("#set_balance_ankle_roll_gain"),
    $("#set_balance_ankle_pitch_gain"),
    $("#set_y_swap_amplitude"),
    $("#set_z_swap_amplitude"),
    $("#set_arm_swing_gain"),
    $("#set_pelvis_offset"),
    $("#set_hip_pitch_offset"),
    $("#set_p_gain"),
    $("#set_i_gain"),
    $("#set_d_gain")
];

var btSubmitSettings = document.getElementsByClassName("submit_settings");

function setEnabledSettings(enabled){
    isWalkParamsLoaded = enabled;
    if(enabled){
        paramElements.forEach(element => {
            element.removeAttr('disabled');
        });
        for(let i = 0; i < btSubmitSettings.length; i++){
            btSubmitSettings[i].disabled = false;
        }
    }
    else{
        paramElements.forEach(element => {
            element.attr('disabled', 'disabled');
        });
        for(let i = 0; i < btSubmitSettings.length; i++){
            btSubmitSettings[i].disabled = true;
        }
    }
}

function setEnabledWalkingConf(enabled){
    var walkconf = document.getElementsByClassName("walking-conf");
    isWalkParamsLoaded = enabled;
    
    for (let element of walkconf) {
        element.setAttribute('disabled',enabled? false : true);
        if(enabled) element.classList.remove("disabled");
        else element.classList.add("disabled");
    }
}

setEnabledWalkingConf(false);
setEnabledSettings(false);

function updateWalkParams(params){
    console.log(params.init_x_offset);
    document.getElementById("set_init_x_offset").value = params.init_x_offset;
    document.getElementById("set_init_y_offset").value = params.init_y_offset;
    document.getElementById("set_init_z_offset").value = params.init_z_offset;
    document.getElementById("set_init_roll_offset").value = params.init_roll_offset;
    document.getElementById("set_init_pitch_offset").value = params.init_pitch_offset;
    document.getElementById("set_init_yaw_offset").value = params.init_yaw_offset;
    document.getElementById("set_period_time").value = params.period_time;
    document.getElementById("set_dsp_ratio").value = params.dsp_ratio;
    document.getElementById("set_step_fb_ratio").value = params.step_fb_ratio;
    document.getElementById("set_x_move_amplitude").value = params.x_move_amplitude;
    document.getElementById("set_y_move_amplitude").value = params.y_move_amplitude;
    document.getElementById("set_z_move_amplitude").value = params.z_move_amplitude;
    document.getElementById("set_angle_move_amplitude").value = params.angle_move_amplitude;
    document.getElementById("set_move_aim_on").checked = params.move_aim_on;
    document.getElementById("set_balance_enable").checked = params.balance_enable;
    document.getElementById("set_balance_hip_roll_gain").value = params.balance_hip_roll_gain;
    document.getElementById("set_balance_knee_gain").value = params.balance_knee_gain;
    document.getElementById("set_balance_ankle_roll_gain").value = params.balance_ankle_roll_gain;
    document.getElementById("set_balance_ankle_pitch_gain").value = params.balance_ankle_pitch_gain;
    document.getElementById("set_y_swap_amplitude").value = params.y_swap_amplitude;
    document.getElementById("set_z_swap_amplitude").value = params.z_swap_amplitude;
    document.getElementById("set_arm_swing_gain").value = params.arm_swing_gain;
    document.getElementById("set_pelvis_offset").value = params.pelvis_offset;
    document.getElementById("set_hip_pitch_offset").value = params.hip_pitch_offset;
    document.getElementById("set_p_gain").value = params.p_gain;
    document.getElementById("set_i_gain").value = params.i_gain;
    document.getElementById("set_d_gain").value = params.d_gain;

    bt_get_walk_params.classList.remove("disabled");
    setEnabledSettings(true);
}

function updateWalkingConf(params) {
    let obj = params

    hasControlID = obj.control; 
    dropdownTurnMode.setActive(obj.turn_mode === 1? TURN_MODE_YAW : TURN_MODE_HEADLESS);
    //obj.stationary_offset;

    input_walking_turnspeed.value = obj.step[2];
    input_walking_Accel.value = obj.step[0];
    input_walking_multipler.value = obj.multiplier[0];
    input_walking_turnrate.value = obj.multiplier[2];
    setEnabledWalkingConf(true);
}

function updateWalking(params){
    var x = params[0] / parseFloat(input_walking_multipler.value)/2;
    if(controlTurnMode === TURN_MODE_YAW){
        x = params[2] / parseFloat(input_walking_turnrate.value)/2;
    }
    setAnalogFeedback(x, params[1] / parseFloat(input_walking_multipler.value)/2);
}

function handleTorqueToggleUpdate(params){
    bt_torque_toggle.classList.remove("disabled");
    if(!params){
        isTorqueOn = false;
        bt_walk_toggle.classList.add("disabled");
        bt_torque_toggle.innerHTML = 'Torque: <span class="badge badge-dark">Off</span>';
    }else{
        isTorqueOn = true;
        bt_walk_toggle.classList.remove("disabled");
        bt_torque_toggle.innerHTML = 'Torque: <span class="badge badge-success">On</span>';
    }
}

function handleWalkToggleUpdate(params){
    bt_walk_toggle.classList.remove("disabled");
    if(params){
        isWalking = true;
        bt_torque_toggle.classList.add("disabled");
        bt_walk_toggle.innerHTML =  'Walk: <span class="badge badge-success">On</span>';
    }else{
        isWalking = false;
        bt_torque_toggle.classList.remove("disabled");
        bt_walk_toggle.innerHTML =  'Walk: <span class="badge badge-dark">Off</span>';
    }
}

function handleControlOverrideUpdate(params){
    if(params[1] != selfID[1]){
        controlMode = CONTROL_MODE_OFF;
    }
    hasControlID = params[1];
}

function handleDeviceConnected(params){
    selfID = params;
    alert.show(alert.TYPE_SUCCESS, "Connected to robot");
}

function handleStatusMsg(params){
    let color = "text-warning";

    let type = params.type;
    let msg = params.status_msg;

    if(type === alert.TYPE_DANGER){
        color = "text-danger";
    }else if(type === alert.TYPE_INFO){
        color = "text-info";
    }else if(type === alert.TYPE_SUCCESS){
        color = "text-success";
    }

    let searchVolt = msg.search("Volt : ");
    if(searchVolt >= 0){
        voltage_badge.innerHTML = msg.substring(searchVolt+1, msg.length);
    }

    con_log.innerHTML += '<p class="log '+color+'">['+type+']['+params.module_name+'] '+msg+'<p>';
    con_log.scrollTop = con_log.scrollHeight;
}



ws.onopen = function (e){
    con_h1.innerHTML = "CONNECTED";
    getAll();
};

ws.onmessage = function (event){
    console.log(event.data)
    var obj = JSON.parse(event.data);
    
    if(obj.cmd == null) return;
    let cmd = obj.cmd;

    if(cmd == "update_walk_params"){
        updateWalkParams(obj.params);
    }
    if(cmd == "update_walking"){
        updateWalking(obj.params);
    }
    if(cmd == "goal_scan_update"){
        goal_detection.updateGoalScan(obj.params);
        
    }
    if(cmd == "update_walking_conf"){
        updateWalkingConf(obj.params);
    }
    if(cmd == "update_status"){
        handleStatusMsg(obj.params);
    }

    if(cmd == "angle_update"){
        handle_angle_update(obj.params);
    }

    if(cmd == "controller_msg"){
        alert.show(alert.TYPE_INFO, obj.params);
    }
    if(cmd == "walk_params_saved"){
        alert.show(alert.TYPE_INFO, obj.params);
    }

    if(cmd == "torque_control"){
        handleTorqueToggleUpdate(obj.params);
    }if(cmd == "walk_control"){
        handleWalkToggleUpdate(obj.params);
    }if(cmd == "control_override"){
        handleControlOverrideUpdate(obj.params);
    }if(cmd == "device_connected"){
        handleDeviceConnected(obj.params);
    }if(cmd == "balance_monitor"){
        pidMonGraph.addItems(obj.params);
        pidMonGraph.updateAllLine();
    }if(cmd == "stream_answer"){
        console.log("its an stream answer!")
        console.log(obj)
        pc.setRemoteDescription(obj);
    }if(cmd == "offset_updated"){
        console.log("offset_updated!")
    }
}

ws.onerror = function(err) {
    console.error('Socket encountered error: ', err.message, 'Closing socket');
    ws.close();
    alert.show(alert.TYPE_DANGER, "You are not connected with the robot!");
    con_h1.innerHTML = "ERR! DISCONNECTED";
    con_h1.style.backgroundColor = "red";
};

function onSubmit(id){
    var el = document.getElementById(id);
    if(el == null) return false;
    sendParameterized("set_walk_params", '["'+ id.substring(4, id.length) +'",'+ el.value +']');
    return false;
}

function reloadBallTrackerParams(){
    sendCmd("reload_ball_track_conf");
}

function onSubmitHead(id){
    var el = document.getElementById(id);
    param = {
        "yaw" : parseFloat(document.getElementById("head_yaw").value),
        "pitch" : parseFloat(document.getElementById("head_pitch").value)
    }

    sendParameterized("head_direct", JSON.stringify(param));
    // sendParameterized("head_direct", JSON.stringify({"yaw" : 0.0,"pitch" : 0.0}));
    console.log(id + el.value);
    return false;
}

function onStartGoalTrack(){
    sendCmd("start_goal_track");
    return false;
}

function onSaveGoalTrack(){
    sendParameterized("save_goal_track","["+field.start.x.toString()+","+field.start.y.toString()+","+field.end.y.toString()+","+field.end.y.toString()+"]");
    return false;
}

function onSubmitWalking(id){
    var el = document.getElementById(id);
    if(el == null) return false;

    let axis = null;
    let param = null;
    if(id === "set_walking_multipler"){
        axis = "xy";
        param = "multipler";
    }
    else if(id === "set_walking_turnrate"){
        axis = "yaw";
        param = "multipler";
    }
    else if(id ==="set_walking_Accel"){
        axis = "xy";
        param = "step";
    }
    else if(id === "set_walking_turnspeed"){
        axis = "yaw";
        param = "step";
    }


    else if(id === "set_walking_offset_x"){
        axis = "x";
        param = "offset";
    }
    else if(id ==="set_walking_offset_y"){
        axis = "y";
        param = "offset";
    }
    else if(id === "set_walking_offset_yaw"){
        axis = "yaw";
        param = "offset";
    }
    sendParameterized("set_walking_conf", JSON.stringify([param, [axis, el.value]]));
    console.log(id + el.value);
    return false;
}

function handle_angle_update(angle){
    yaw_dir_vis.set_angle(angle)
}

function onSubmitCB(id){
    var el = document.getElementById(id);
    if(el == null) return false;
    sendParameterized("set_walk_params", '["'+ id.substring(4, id.length) +'",'+ (el.checked?'true':'false') +']');
    return false;
}

function sendWalking(){
    if(isKeyboardHasControl){
        let v = getKeyboardControlVector();
        setAnalogPointer(v.x, v.y);
    }

    let obj = {
        x: analogValue.x,
        y: analogValue.y,
        yaw: analogValue.x
    };

    sendParameterized('set_walking', JSON.stringify(obj));
}

var dropdownControlMode = new Dropdown([
    CONTROL_MODE_OFF,    
    CONTROL_MODE_OFFSET,    
    CONTROL_MODE_CONTROL
],[
    "Off",    
    "Set Stationary",    
    "Auto Centering"
],"dropdown_control_mode_parent",
"onWalkingControlModeChanged");

var dropdownTurnMode = new Dropdown([
    TURN_MODE_YAW,
    TURN_MODE_HEADLESS
],[
    "Yaw Mode",
    "Headless Mode"
],"dropdown_turn_mode_parent",
"onWalkingTurnModeChanged");

function onWalkingControlModeChanged(controlMode){
    setAnalogPointer(0,0);
    sendWalking();
    if(controlMode === CONTROL_MODE_OFF){
        sendParameterized("set_control_walking", 0);
        $("#"+bt_send_stationary.id).collapse("hide");
        dropdownControlMode.setActive(CONTROL_MODE_OFF);
        controlMode = CONTROL_MODE_OFF;
    }else if(controlMode === CONTROL_MODE_OFFSET){
        $("#"+bt_send_stationary.id).collapse("show");
        dropdownControlMode.setActive(CONTROL_MODE_OFFSET);
        controlMode = CONTROL_MODE_OFFSET;
        isHoldMode = true;
        setWalkingControlOverride();
    }else if(controlMode === CONTROL_MODE_CONTROL){
        $("#"+bt_send_stationary.id).collapse("hide");
        dropdownControlMode.setActive(CONTROL_MODE_CONTROL);
        controlMode = CONTROL_MODE_CONTROL
        isHoldMode = false;
        setWalkingControlOverride();
    }
}

function onWalkingTurnModeChanged(controlMode){
    if(controlMode === TURN_MODE_YAW){
        sendParameterized("set_walking_conf", JSON.stringify(["turn_mode", 1]));
        dropdownTurnMode.setActive(TURN_MODE_YAW)
        controlTurnMode = TURN_MODE_YAW;
    }else if(controlMode === TURN_MODE_HEADLESS){
        sendParameterized("set_walking_conf", JSON.stringify(["turn_mode", 0]));
        dropdownTurnMode.setActive(TURN_MODE_HEADLESS)
        controlTurnMode = TURN_MODE_HEADLESS;
    }
}


function setWalkingControlOverride(){
    if(hasControlID >= 0){
        modal.showConfirm(
            "Override control?",
            hasControlID+" is controlling, confirm to override...",
            'sendParameterized("set_control_walking", 1)');
    }else{
        sendParameterized("set_control_walking", 1);
    }
}

function getAll(){
    sendCmd("get_walking_conf");
    sendCmd("get_walk_params");
}




function sendCmd(cmd){
    var data = '{"cmd":"'+ cmd +'"}';
    ws.send(data);
}

function sendParameterized(cmd, param){
    var data = '{"cmd":"'+ cmd +'","params":'+param+'}';
    console.log(data);
    ws.send(data);
}

// function sendCmd(_cmd){
//     let dataObj = {
//         cmd: _cmd
//     }
//     var data = JSON.stringify(dataObj);
//     ws.send(data);
// }

// function sendParameterized(_cmd, param){
//     let dataObj = {
//         cmd: _cmd,
//         params: param
//     }

//     var data = JSON.stringify(dataObj);
//     ws.send(data);
// }

bt_torque_toggle.addEventListener("click", function(event) {
    event.preventDefault();
    if(!isTorqueOn){
        sendCmd("torque_on");
    }else{
        sendCmd("torque_off");
    }
    responsiveCmd.push([CMD_TORQUE_TOGGLE, 0]);
    bt_torque_toggle.classList.add("disabled");
}, false);

bt_walk_toggle.addEventListener("click", function(event) {
    event.preventDefault();
    if(isWalking){
        sendCmd("stop_walk");
    }else{
        sendCmd("start_walk");
    }
    responsiveCmd.push([CMD_WALK_TOGGLE, 0]);
    bt_walk_toggle.classList.add("disabled");
}, false);

bt_get_walk_params.addEventListener("click", function(event) {
    event.preventDefault();
    setEnabledSettings(false);
    sendCmd("get_walk_params");
    bt_get_walk_params.classList.add("disabled");
}, false);

/// STREAMER

function negotiate() {
    pc.addTransceiver('video', {direction: 'recvonly'});
    return pc.createOffer().then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        // wait for ICE gathering to complete
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });

    }).then(function() {
        var offer = pc.localDescription;
        var request = JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            });
        sendParameterized("stream_offer", request);
    });
}

function load_walking_conf(){
    sendParameterized("load_walking_conf", request);
}

function startVision(){
    var config = {
        sdpSemantics: 'unified-plan'
    };

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        }
    });

    negotiate();
}

//// TABS

var collapsibles = [
    $("#settings"),
    $("#manual_ctrl"),
    $("#head_ctrl"),
    $("#pid_mon"),
    $("#vision_stream")
];

var onShow = null; 

bt_show_settings.addEventListener("click", function(event) {
    event.preventDefault();

    if(onShow == null){
        collapsibles[0].collapse("show");
        onShow = collapsibles[0];
    }else if(onShow == collapsibles[0]){
        collapsibles[0].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide");
        collapsibles[0].collapse("show");
        onShow = collapsibles[0];
    }

}, false);

bt_show_manuctrl.addEventListener("click", function(event) {
    event.preventDefault();
    if(onShow == null){
        collapsibles[1].collapse("show");
        onShow = collapsibles[1];
    }else if(onShow == collapsibles[1]){
        collapsibles[1].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide");
        collapsibles[1].collapse("show");
        onShow = collapsibles[1];
    }
}, false);

bt_show_headctrl.addEventListener("click", function(event) {
    event.preventDefault();
    if(onShow == null){
        collapsibles[2].collapse("show");
        onShow = collapsibles[2];
    }else if(onShow == collapsibles[2]){
        collapsibles[2].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide");
        collapsibles[2].collapse("show");
        onShow = collapsibles[2];
    }
}, false);

bt_show_pidmon.addEventListener("click", function(event) {
    event.preventDefault();
    if(onShow == null){
        collapsibles[3].collapse("show");
        onShow = collapsibles[3];
    }else if(onShow == collapsibles[3]){
        collapsibles[3].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide");
        collapsibles[3].collapse("show");
        onShow = collapsibles[3];
    }
}, false);

bt_vision_stream.addEventListener("click", function(event) {
    event.preventDefault();
    if(onShow == null){
        collapsibles[4].collapse("show");
        onShow = collapsibles[4];
    }else if(onShow == collapsibles[4]){
        collapsibles[4].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide");
        collapsibles[4].collapse("show");
        onShow = collapsibles[4];
    }
}, false);

bt_send_stationary.addEventListener("click", function(event) {
    sendCmd("set_walking_offset");
}, false);

bt_submit_settings.addEventListener("click", function(event) {
    sendCmd("save_walk_params");
}, false);

bt_load_offset.addEventListener("click", function(e){
    sendCmd("load_offset")
}, false);


timerResponsiveCmd = setInterval(function (){
    for(var i=0; i < responsiveCmd.length; i++){
        rCmd = responsiveCmd[i];
        rCmd[1]++;
        if(rCmd[1] > cmdTimeout){
            switch(rCmd[0]){
            case CMD_WALK_TOGGLE:
                bt_walk_toggle.classList.remove("disabled");
                break;
            case CMD_TORQUE_TOGGLE:
                bt_torque_toggle.classList.remove("disabled");
                break;
            case CMD_CONTROL_OVERRIDE:
                
                break;
            case CMD_UPDATE_WAlK_PARAMS:
                bt_get_walk_params.classList.remove("disabled");
                break;
            }
            responsiveCmd.splice(i, 1);
        }
    }
}, 1000);


var realTimeControl = setInterval(function (){
    if(hasControlID == selfID[1] && selfID[1] != -1){
        sendWalking();
    }
}, 100);

