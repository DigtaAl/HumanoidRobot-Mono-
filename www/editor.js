class Vector2 {
    constructor(x, y) {
      this.x = x;
      this.y = y;
    }
}

var onEdit = -1;
var onParamEditor = -1;
var paramElementOnEdit = null;
var currentEditVal = 1;

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

var paramScales = [
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4],
    [0,4]
];

var scales = [0.001, 0.01, 0.1, 1, 10, 100];

var sizeZero = 0;

var heightIndicator = 120;
var sizePointer = new Vector2(40,40);

var start = new Vector2(0.0,0.0);

var indicator = document.getElementById("indicator_editor");
var indicatorZero = document.getElementById("indicator_zero_editor");
var pointer = document.getElementById("pointer_editor");

indicator.style.height = heightIndicator + "px";
pointer.style.width = sizePointer.x + "px";
pointer.style.height = sizePointer.y + "px";

function setHidden(isHidden){
    pointer.hidden = isHidden;
    indicator.hidden = isHidden;
    indicatorZero.hidden = isHidden;
}
setHidden(true);

function clamp(val, max, min){
    return Math.max(Math.min(val, max), min);
}

function editorTimer(){
    var val2  = parseFloat(paramElementOnEdit.val()) + currentEditVal;
    var val2 =  Math.abs(currentEditVal) == 0.1? Math.round((val2 + Number.EPSILON) * 10) / 10:
                Math.abs(currentEditVal) == 0.01? Math.round((val2 + Number.EPSILON) * 100) / 100:
                Math.abs(currentEditVal) == 0.001? Math.round((val2 + Number.EPSILON) * 1000) / 1000:
                Math.round((val2 + Number.EPSILON) * 1) / 1;
    paramElementOnEdit.val(val2);
}

var timerEditor = null;

function startTimerEditor(){
    if(timerEditor == null){
        timerEditor = setInterval(editorTimer, 200);
    }
}

function stopTimerEditor(){
    clearInterval(timerEditor);
    timerEditor = null;
}


let indexParamScale = 0;
let paramScale = 0;
let lenScale = 0;
let sizeBlock = 0;

document.querySelector("*").addEventListener("mousemove", function(e) {
    e.preventDefault();
    if(onEdit === -1) return;
    if (e.which == 1) {

        let y = clamp(e.pageY, start.y + heightIndicator/2, start.y - heightIndicator/2);

        pointer.style.top = (y - sizePointer.y/2) + "px";

        //
        let pos = y - start.y;
        let point = (pos < 0)? Math.ceil(pos/sizeBlock) : Math.floor(pos/sizeBlock); 
        let indexScale = (Math.abs(point)-sizeZero-1);
        
        if(indexScale < 0) {
            currentEditVal = 0;
            stopTimerEditor();
        }
        else{ 
            currentEditVal = scales[indexScale+paramScale[0]] * (pos < 0? 1 : -1);
            startTimerEditor();
        }
    }
}, false);


function mouseEnterEditor(id){
    //find index
    if(onEdit < 0){
        for(let i = 0; i < paramElements.length; i++){
            if(paramElements[i].attr("id") == id){
                onParamEditor = i;    
                paramElementOnEdit = paramElements[i];
            }
        }
    }
}

function mouseLeaveEditor(){
    onParamEditor = -1;
}

document.querySelector("*").addEventListener("mousedown", function(e) {
    //e.preventDefault();
    
    if(onParamEditor === -1) return;
    
    if (e.which == 1) {
        setHidden(false);
        
        pointer.style.top = (e.pageY - sizePointer.y/2) + "px";
        pointer.style.left = (e.pageX - 30) + "px";
        
        indicator.style.top = (e.pageY - heightIndicator/2) + "px";
        indicator.style.left = (e.pageX - 52) + "px";
        
        start = new Vector2(e.pageX, e.pageY);
        
        indexParamScale = onParamEditor;
        paramScale = paramScales[indexParamScale];
        lenScale = paramScale[1]-paramScale[0];
        sizeBlock = (heightIndicator/2)/(lenScale+sizeZero);

        let hZero = (sizeBlock + sizeZero)*2;
        indicatorZero.style.height = hZero + "px";

        indicatorZero.style.top = (e.pageY - hZero/2) + "px";
        indicatorZero.style.left = (e.pageX - 52) + "px";

        onEdit = onParamEditor;
    }
}, false);

document.querySelector("*").addEventListener("mouseup", function(e) {
    e.preventDefault();
    if (e.which == 1) {
        setHidden(true);
        onEdit = -1;
        stopTimerEditor();
    }
}, false);




