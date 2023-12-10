var Vec = function () {
    this.x = 0.0;
    this.y = 0.0;
    this.set = function (_x, _y) {
        this.x = _x;
        this.y = _y;
    }
};

function drawCircle(ctx, x, y, radius, fill, stroke, strokeWidth) {
    ctx.beginPath()
    ctx.arc(x, y, radius, 0, 2 * Math.PI, false)
    if (fill) {
        ctx.fillStyle = fill
        ctx.fill()
    }
    if (stroke) {
        ctx.lineWidth = strokeWidth
        ctx.strokeStyle = stroke
        ctx.stroke()
    }
}

var Field = function (el) {
    this.canvas = el,
        this.height = 400,
        this.width = 600,
        this.length = 40,
        this.start = new Vec(),
        this.end = new Vec(),

        this.bg = function () {
            var ctx = this.canvas.getContext("2d");

            let img = document.getElementById("field_img");
            ctx.drawImage(img, 10, 10, this.width, this.height);
            img.hidden = true;
        },

        this.build = function () {

            var ctx = this.canvas.getContext("2d");
            this.canvas.setAttribute("height", this.height + 20);
            this.canvas.setAttribute("width", this.width + 20);

            this.bg();

            const c = this.canvas;
            const startPos = this.start;
            const endPos = this.end;
            const ini = this;

            this.canvas.addEventListener('mousedown', function (e) {
                const rect = c.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const y = e.clientY - rect.top;
                startPos.set(x, y);
                ini.setPos(startPos, x, y);
            });

            this.canvas.addEventListener('mouseup', function (e) {
                const rect = c.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const y = e.clientY - rect.top;

                endPos.set(x, y);
                ini.setPos(startPos, x, y);

                console.log("x: " + x + " y: " + y);
            });

            

            this.canvas.addEventListener('mousemove', function (e) {
                const rect = c.getBoundingClientRect();
                const x = e.clientX - rect.left;
                const y = e.clientY - rect.top;

                endPos.set(x, y);
                ini.setPos(startPos, x, y);

                console.log("x: " + x + " y: " + y);
            });
        },

        this.setPos = function (pos, tar_x, tar_y) {
            var ctx = this.canvas.getContext("2d");
            ctx.clearRect(0, 0, this.width + 20, this.height + 20);
            this.bg();

            let delta = new Vec(tar_x - pos.x, tar_y - pos.y);


            drawCircle(ctx, pos.x, pos.y, 6, 'black', 'red', 1)
            drawCircle(ctx, tar_x, tar_y, 4, 'blue', 'yellow', 1)
            ctx.lineWidth = 3;
            ctx.lineColor = "rgba(100, 100, 255, 1)";
            ctx.beginPath();
            ctx.moveTo(pos.x, pos.y);
            ctx.lineTo(tar_x, tar_y);
            ctx.stroke();
        }
};