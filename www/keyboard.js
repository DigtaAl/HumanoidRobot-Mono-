var isKeyboardHasControl = false;

var cf = false;
var cb = false;
var cl = false;
var cr = false;

document.addEventListener("keydown", event => {
    if (event.isComposing || event.keyCode === 87) {
        event.preventDefault();
        if(cf)  return;
        cf = true;
    }
    if (event.isComposing || event.keyCode === 83) {
        event.preventDefault();
        if(cb)  return;
        cb = true;
    }
    if (event.isComposing || event.keyCode === 65) {
        event.preventDefault();
        if(cl)  return;
        cl = true;
    }
    if (event.isComposing || event.keyCode === 68) {
        event.preventDefault();
        if(cr)  return;
        cr = true;
    }
});

document.addEventListener("keyup", event => {
    if (event.isComposing || event.keyCode === 87) {
        event.preventDefault();
        cf = false;
    }
    if (event.isComposing || event.keyCode === 83) {
        event.preventDefault();
        cb = false;
    } 
    if (event.isComposing || event.keyCode === 65) {
        cl = false;
    }
    if (event.isComposing || event.keyCode === 68) {
        cr = false;
    }
});

function getKeyboardControlVector(){
    var vec = new Vector2(0.0, 0.0); 
    if(cf)
        vec.y += 0.5;
    if(cb)
        vec.y -= 0.5;
    if(cl)
        vec.x -= 0.5;
    if(cr)
        vec.x += 0.5;

    return vec;
}