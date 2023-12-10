

var GoalDetection = function(el){
    this.canvas = el,
    this.height = 300,
    this.width = 300,
    this.length = 40,
    this.radius = 100,
    this.offset_x = 0,
    this.offset_y = 0,

    this.found = false,
    this.dets = undefined;
    
    this.build = function (){

        this.offset_x = this.width/2;
        this.offset_y = this.height/2;

        var ctx = this.canvas.getContext("2d");
        this.canvas.setAttribute("height",this.height);
        this.canvas.setAttribute("width", this.width);
        
        let line_subdivision = 20
        this.radius = 120
        
        for(var i = 0; i <= line_subdivision; i++){
            let circle_point = Math.PI/line_subdivision*i;
            let pos_x = Math.cos(circle_point)*this.radius + this.offset_x;
            let pos_y = -Math.sin(circle_point)*this.radius+ this.offset_y;
            ctx.strokeStyle = "rgba(150, 150, 150, 1)";
            ctx.lineWidth = 3;
            ctx.lineTo(pos_x, pos_y);
            ctx.stroke();
            if(i == 0) ctx.moveTo(pos_x, pos_y);
        }
        
    },
    this.addDet = function(pos, width, color){
        var ctx = this.canvas.getContext("2d");
        ctx.beginPath();

        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        
        let circlepoint = Math.PI/2*(pos*(2/3))+Math.PI/2;
        let pos_x = Math.cos(circlepoint);
        let pos_y = -Math.sin(circlepoint);
        
        ctx.moveTo(pos_x *(this.radius-10) + this.offset_x, pos_y *(this.radius-10) + this.offset_y);
        ctx.lineTo(pos_x *(this.radius+10) + this.offset_x, pos_y *(this.radius+10) + this.offset_y);
        ctx.stroke();
    },
    this.updateGoalScan = function(obj){
        let dets = obj.dets
        let center = obj.center
        let det_len = dets.length;
        this.build();
        for(var i = 0; i< det_len; i++){
            this.addDet(dets[i][0], 1, "rgba(100, 100, 255, 1)");
        } 

        console.log(obj);
        console.log(center);
        if(obj.found === true){
            console.log("found!");
            this.addDet(center[0], 3, "rgba(200, 255, 200, 1)");
            this.dets = obj;
        }
    },

    this.set_angle = function(angle){
        var ctx = this.canvas.getContext("2d");
        ctx.beginPath();

        ctx.strokeStyle = "rgba(200, 255, 200, 2)";
        ctx.lineWidth = 3;
        
        let circlepoint = angle
        let pos_x = Math.cos(circlepoint) * this.radius/2;
        let pos_y = -Math.sin(circlepoint) * this.radius/2;
        
        let offsetx = this.width/2;
        let offsety = this.height * 3/4;
        
        ctx.moveTo(offsetx, offsety);
        ctx.lineTo(pos_x + offsetx, pos_y + offsety);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.arc(offsetx, offsety, 8, 0, 2 * Math.PI);  
        ctx.fill();
    }
};



var YawDirVis = function(el){
    this.canvas = el,
    this.height = 300,
    this.width = 300,
    this.length = 40,
    this.radius = 100,
    this.offset_x = 0,
    this.offset_y = 0,
    
    this.build = function (){

        this.offset_x = this.width/2;
        this.offset_y = this.height/2;

        var ctx = this.canvas.getContext("2d");
        this.canvas.setAttribute("height",this.height);
        this.canvas.setAttribute("width", this.width);
        
        ctx.clearRect(0, 0, this.width, this.height);
        ctx.fillStyle = "black";
        ctx.fillRect(0, 0, this.width, this.height);
        ctx.fillStyle = "orange";
        ctx.fillRect(20, 0, this.width - 40, 20);

        for(let i = 0; i < Math.PI*2; i+=0.2){
            this.addBar(i, 1, "white");
        }
    },

    this.addBar = function(pos, width, color){
        var ctx = this.canvas.getContext("2d");
        ctx.beginPath();

        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        
        let circlepoint = Math.PI/2*(pos*(2/3))+Math.PI/2;
        let pos_x = Math.cos(circlepoint);
        let pos_y = -Math.sin(circlepoint);
        
        ctx.moveTo(pos_x *(this.radius-10) + this.offset_x, pos_y *(this.radius-10) + this.offset_y);
        ctx.lineTo(pos_x *(this.radius+10) + this.offset_x, pos_y *(this.radius+10) + this.offset_y);
        ctx.stroke();
    },

    this.set_angle = function(angle){

        this.build()

        var ctx = this.canvas.getContext("2d");
        
        ctx.beginPath();

        ctx.strokeStyle = "rgba(200, 255, 200, 2)";
        ctx.lineWidth = 3;
        
        let circlepoint = angle + Math.PI/2;
        let pos_x = Math.cos(circlepoint) * this.radius;
        let pos_y = -Math.sin(circlepoint) * this.radius;
        
        let offsetx = this.width/2;
        let offsety = this.height/2;
        
        ctx.moveTo(offsetx, offsety);
        ctx.lineTo(pos_x + offsetx, pos_y + offsety);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.arc(offsetx, offsety, 8, 0, 2 * Math.PI);  
        ctx.fill();
    }
};

