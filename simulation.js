class DoublePendulum {
    constructor(offsetX, offsetY) {
        // Initial configuration
        this.theta1 = Math.PI / 2;
        this.theta2 = Math.PI / 2;
        this.p1 = 0;
        this.p2 = 0;
        this.m1 = 1;
        this.m2 = 1;
        this.l1 = 1;
        this.l2 = 1;
        this.g = 9.81;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
    }

    rk4_step() {
        const dt = 0.01;

        const dtheta1 = (p1, p2, theta1, theta2) => {
            return (6 / (this.m1 * this.l1 * this.l1)) * (2 * p1 - 3 * Math.cos(theta1 - theta2) * p2) / (16 - 9 * Math.cos(theta1 - theta2) * Math.cos(theta1 - theta2));
        };

        const dtheta2 = (p1, p2, theta1, theta2) => {
            return (6 / (this.m2 * this.l2 * this.l2)) * (8 * p2 - 3 * Math.cos(theta1 - theta2) * p1) / (16 - 9 * Math.cos(theta1 - theta2) * Math.cos(theta1 - theta2));
        };

        const dp1 = (p1, p2, theta1, theta2) => {
            return -(this.m1 + this.m2) * this.g * this.l1 * Math.sin(theta1) - dtheta1(p1, p2, theta1, theta2) * dtheta2(p1, p2, theta1, theta2) * Math.sin(theta1 - theta2);
        };

        const dp2 = (p1, p2, theta1, theta2) => {
            return -this.m2 * this.g * this.l2 * Math.sin(theta2) + dtheta1(p1, p2, theta1, theta2) * dtheta2(p1, p2, theta1, theta2) * Math.sin(theta1 - theta2);
        };

        const k1_theta1 = dt * dtheta1(this.p1, this.p2, this.theta1, this.theta2);
        const k1_theta2 = dt * dtheta2(this.p1, this.p2, this.theta1, this.theta2);
        const k1_p1 = dt * dp1(this.p1, this.p2, this.theta1, this.theta2);
        const k1_p2 = dt * dp2(this.p1, this.p2, this.theta1, this.theta2);

        const k2_theta1 = dt * dtheta1(this.p1 + 0.5 * k1_p1, this.p2 + 0.5 * k1_p2, this.theta1 + 0.5 * k1_theta1, this.theta2 + 0.5 * k1_theta2);
        const k2_theta2 = dt * dtheta2(this.p1 + 0.5 * k1_p1, this.p2 + 0.5 * k1_p2, this.theta1 + 0.5 * k1_theta1, this.theta2 + 0.5 * k1_theta2);
        const k2_p1 = dt * dp1(this.p1 + 0.5 * k1_p1, this.p2 + 0.5 * k1_p2, this.theta1 + 0.5 * k1_theta1, this.theta2 + 0.5 * k1_theta2);
        const k2_p2 = dt * dp2(this.p1 + 0.5 * k1_p1, this.p2 + 0.5 * k1_p2, this.theta1 + 0.5 * k1_theta1, this.theta2 + 0.5 * k1_theta2);

        const k3_theta1 = dt * dtheta1(this.p1 + 0.5 * k2_p1, this.p2 + 0.5 * k2_p2, this.theta1 + 0.5 * k2_theta1, this.theta2 + 0.5 * k2_theta2);
        const k3_theta2 = dt * dtheta2(this.p1 + 0.5 * k2_p1, this.p2 + 0.5 * k2_p2, this.theta1 + 0.5 * k2_theta1, this.theta2 + 0.5 * k2_theta2);
        const k3_p1 = dt * dp1(this.p1 + 0.5 * k2_p1, this.p2 + 0.5 * k2_p2, this.theta1 + 0.5 * k2_theta1, this.theta2 + 0.5 * k2_theta2);
        const k3_p2 = dt * dp2(this.p1 + 0.5 * k2_p1, this.p2 + 0.5 * k2_p2, this.theta1 + 0.5 * k2_theta1, this.theta2 + 0.5 * k2_theta2);

        const k4_theta1 = dt * dtheta1(this.p1 + k3_p1, this.p2 + k3_p2, this.theta1 + k3_theta1, this.theta2 + k3_theta2);
        const k4_theta2 = dt * dtheta2(this.p1 + k3_p1, this.p2 + k3_p2, this.theta1 + k3_theta1, this.theta2 + k3_theta2);
        const k4_p1 = dt * dp1(this.p1 + k3_p1, this.p2 + k3_p2, this.theta1 + k3_theta1, this.theta2 + k3_theta2);
        const k4_p2 = dt * dp2(this.p1 + k3_p1, this.p2 + k3_p2, this.theta1 + k3_theta1, this.theta2 + k3_theta2);

        this.theta1 += (k1_theta1 + 2 * k2_theta1 + 2 * k3_theta1 + k4_theta1) / 6;
        this.theta2 += (k1_theta2 + 2 * k2_theta2 + 2 * k3_theta2 + k4_theta2) / 6;
        this.p1 += (k1_p1 + 2 * k2_p1 + 2 * k3_p1 + k4_p1) / 6;
        this.p2 += (k1_p2 + 2 * k2_p2 + 2 * k3_p2 + k4_p2) / 6;
    }

    update() {
        this.rk4_step();
    }

    draw(ctx) {
        // Draw the double pendulum on the canvas
        const x1 = this.l1 * Math.sin(this.theta1);
        const y1 = this.l1 * Math.cos(this.theta1);
        const x2 = x1 + this.l2 * Math.sin(this.theta2);
        const y2 = y1 + this.l2 * Math.cos(this.theta2);

        ctx.beginPath();
        ctx.moveTo(200 + this.offsetX, 200 + this.offsetY);
        ctx.lineTo(200 + x1 * 100 + this.offsetX, 200 + y1 * 100 + this.offsetY);
        ctx.lineTo(200 + x2 * 100 + this.offsetX, 200 + y2 * 100 + this.offsetY);
        ctx.stroke();
    }
}

const canvasJs = document.getElementById('canvas-js');
const ctxJs = canvasJs.getContext('2d');
let pendulumsJs = [];
let pendulumCountJs = 10;
let lastTimeJs = performance.now();
let frameCountJs = 0;
let totalTimeJs = 0;

function initializePendulumsJs(count) {
    pendulumsJs = [];
    for (let i = 0; i < count; i++) {
        const offsetX = (i % 10) * 5;
        const offsetY = Math.floor(i / 10) * 5;
        pendulumsJs.push(new DoublePendulum(offsetX, offsetY));
    }
}

initializePendulumsJs(pendulumCountJs);

function updateJs() {
    const now = performance.now();
    const deltaTime = now - lastTimeJs;
    lastTimeJs = now;
    totalTimeJs += deltaTime;
    frameCountJs++;

    ctxJs.clearRect(0, 0, canvasJs.width, canvasJs.height);
    for (const pendulum of pendulumsJs) {
        pendulum.update();
        pendulum.draw(ctxJs);
    }

    if (frameCountJs % 60 === 0) {
        const fps = (frameCountJs / totalTimeJs) * 1000;
        const avgTime = totalTimeJs / frameCountJs;
        document.getElementById('fps-js').textContent = fps.toFixed(2);
        document.getElementById('avg-time-js').textContent = avgTime.toFixed(2);
    }

    requestAnimationFrame(updateJs);
}

updateJs();

document.getElementById('pendulum-count-js').addEventListener('input', (event) => {
    pendulumCountJs = event.target.value;
    document.getElementById('pendulum-count-display-js').textContent = pendulumCountJs;
    initializePendulumsJs(pendulumCountJs);
});
