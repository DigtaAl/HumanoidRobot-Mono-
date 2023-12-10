class Vector2 {
    constructor(x, y) {
      this.x = x;
      this.y = y;
    }
}

function clamp(val, max, min){
    return Math.max(Math.min(val, max), min);
}

var analog_container = document.getElementById('analog_container');
var h_bar = document.getElementById('h_bar');
var v_bar = document.getElementById('v_bar');
var analog_area = document.getElementById('analog_area');
var analog_pointer = document.getElementById('analog_pointer');
var analog_line_v = document.getElementById('analog_line_v');
var analog_line_h = document.getElementById('analog_line_h');

var pointer_v_bar = document.getElementById('pointer_v_bar');
var pointer_h_bar = document.getElementById('pointer_h_bar');

var pointer_tar_v_bar = document.getElementById('pointer_tar_v_bar');
var pointer_tar_h_bar = document.getElementById('pointer_tar_h_bar');

var h_bar_strip = document.getElementsByClassName('strip-horizontal');
var v_bar_strip = document.getElementsByClassName('strip-vertical');
var h_bar_strip_center = document.getElementsByClassName('strip-center-h')[0];
var v_bar_strip_center = document.getElementsByClassName('strip-center-v')[0];

var bar_length = 150;
var bar_width = 20;
var indicator_analog_size = 18;
var strip_count = h_bar_strip.length;

var strip_width = 2;
var area_line_width = 2;
var bar_distance = 50;
var area_dimens = new Vector2(150,150);
var analog_container_size = new Vector2(250,250);
var analog_pointer_size = 40;

var on_analog_edit = false;
var mouse_on_analog = false;

var analogValue = new Vector2(0.0,0.0);

var isHoldMode = false;

function initAnalog(){

    //analog_container
    analog_container.style.width = analog_container_size.x + "px";
    analog_container.style.height = analog_container_size.y + "px";


    // analog_area
    analog_area.style.width = area_dimens.x + "px";
    analog_area.style.height = area_dimens.y + "px";

    analog_area.style.top = Math.floor(analog_container_size.y/2 - area_dimens.y/2) + "px"; 
    analog_area.style.left = Math.floor(analog_container_size.x/2 - area_dimens.x/2) + "px"; 

    // line
    analog_line_h.style.width = area_dimens.x + "px";
    analog_line_v.style.width = area_line_width + "px";
    analog_line_h.style.height = area_line_width + "px";
    analog_line_v.style.height = area_dimens.y + "px";

    analog_line_h.style.top = Math.floor(area_dimens.y/2 - area_line_width/2) + "px";
    analog_line_v.style.left = Math.floor(area_dimens.x/2 - area_line_width/2) + "px";

    // bar
    v_bar.style.top = -bar_distance + "px";
    h_bar.style.left = -bar_distance + "px";

    // Strip
    strip_distance = bar_length/(strip_count+2);

    for(var i = 1; i < strip_count/2 + 1; i++){
        v_bar_strip[i-1].style.top = ((strip_distance * i) - strip_width/2) + "px";
    }
    for(var i = strip_count/2+2; i < strip_count + 2; i++){
        v_bar_strip[i-2].style.top = ((strip_distance * i) - strip_width/2) + "px";
    }

    v_bar_strip_center.style.top = (bar_length/2 - strip_width/2) + "px";

    for(var i = 1; i < strip_count/2 + 1; i++){
        h_bar_strip[i-1].style.left = ((strip_distance * i) - strip_width/2) + "px";
    }
    for(var i = strip_count/2+2; i < strip_count + 2; i++){
        h_bar_strip[i-2].style.left = ((strip_distance * i) - strip_width/2) + "px";
    }

    h_bar_strip_center.style.left = (bar_length/2 - strip_width/2) + "px";


    // analog_indicator
    pointer_v_bar.style.left = -bar_distance + bar_width + "px";
    pointer_h_bar.style.top = -bar_distance + bar_width - 4 + "px";

    setAnalogIndicator(true, pointer_v_bar, 0);
    setAnalogIndicator(false, pointer_h_bar, 0);

    pointer_tar_v_bar.style.left = -bar_distance + bar_width + "px";
    pointer_tar_h_bar.style.top = -bar_distance + bar_width - 4 + "px";

    setAnalogIndicator(true, pointer_tar_v_bar, 0);
    setAnalogIndicator(false, pointer_tar_h_bar, 0);

    // analog_pointer
    analog_pointer.style.width = analog_pointer_size.x + "px";
    analog_pointer.style.height = analog_pointer_size.y + "px";

    setAnalogPointer(0,0);

}

function setAnalogIndicator(isVertical, element, value) { // value normalized to .5
    if(isVertical){
        element.style.top = bar_length - (value + 0.5) * (bar_length) - (indicator_analog_size/2) -4 + "px";
    }else{
        element.style.left = (value + 0.5) * (bar_length) - (indicator_analog_size/2) + "px";
    }
}

function setAnalogPointer(x,y) {

    analogValue.y = y*2; // normalized to 1
    analogValue.x = x*2; // normalized to 1

    setAnalogIndicator(true, pointer_tar_v_bar, y);
    setAnalogIndicator(false, pointer_tar_h_bar, x);

    analog_pointer.style.bottom = (y + 0.5) * (area_dimens.y) - (analog_pointer_size/2) + "px";
    analog_pointer.style.left = (x + 0.5) * (area_dimens.x) - (analog_pointer_size/2) + "px";
}

function setAnalogFeedback(x,y) {
    console.log(x+" "+y);
    setAnalogIndicator(true, pointer_v_bar, y);
    setAnalogIndicator(false, pointer_h_bar, x);
}

initAnalog();

document.querySelector("*").addEventListener("mousemove", function(e) {
    if(!on_analog_edit) return;
    e.preventDefault();
    if (e.which == 1) {
        var rect = analog_area.getBoundingClientRect();
        var x = e.clientX - rect.left; 
        var y = e.clientY - rect.top;  

        setAnalogPointer(clamp(x/area_dimens.x - 0.5, 0.5, -0.5), clamp(-y/area_dimens.y + 0.5, 0.5, -0.5));
    }
}, false);

document.querySelector("*").addEventListener("mousedown", function(e) {
    //e.preventDefault();
    if (e.which == 1) {
        if(mouse_on_analog) on_analog_edit = true;
    }
}, false);

function mouseEnterAnalog(){
    //find index
    mouse_on_analog = true;
}

function mouseLeaveAnalog(){
    mouse_on_analog = false;
}

document.querySelector("*").addEventListener("mouseup", function(e) {
    e.preventDefault();
    if (e.which == 1) {
        
        if(!isHoldMode)
            setAnalogPointer(0,0);
        on_analog_edit = false;

    }
}, false);

// touch

document.querySelector("*").addEventListener('touchstart', function(e) {
    if (e.which == 1) {
        if(mouse_on_analog) on_analog_edit = true;
    }
    console.log("touchstart");
}, false);

document.querySelector("*").addEventListener('touchmove', function(e) {
    if(!on_analog_edit) return;
    e.preventDefault();
    if (e.which == 1) {
        var rect = analog_area.getBoundingClientRect();
        var x = e.clientX - rect.left; 
        var y = e.clientY - rect.top;  

        setAnalogPointer(clamp(x/area_dimens.x - 0.5, 0.5, -0.5), clamp(-y/area_dimens.y + 0.5, 0.5, -0.5));
    }
    console.log("touchmove");
}, false);
document.querySelector("*").addEventListener('touchcancel', function(e) {
    e.preventDefault();
    if (e.which == 1) {
        
        if(!isHoldMode)
            setAnalogPointer(0,0);
        on_analog_edit = false;

    }

    console.log("touchcancel");
}, false);
document.querySelector("*").addEventListener('touchend', function(e) {
    e.preventDefault();
    if (e.which == 1) {
        
        if(!isHoldMode)
            setAnalogPointer(0,0);
        on_analog_edit = false;

    }
    console.log("touchend");
}, false);