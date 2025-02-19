class DoublePendulum {
    constructor(offset_x, offset_y) {
        this.theta1 = Math.PI / 2;
        this.theta2 = Math.PI / 2;
        this.p1 = 0;
        this.p2 = 0;
        this.m1 = 1;
        this.m2 = 1;
        this.l1 = 1;
        this.l2 = 1;
        this.g = 9.81;
        this.offset_x = offset_x;
        this.offset_y = offset_y;
    }

    rk4_step() {
        const dt = 0.01;

        const dtheta1 = (p1, p2, theta1, theta2) => {
            return (6 / (this.m1 * this.l1 * this.l1)) * (2 * p1 - 3 * Math.cos(theta1 - theta2) * p2) / (16 - 9 * Math.cos(theta1 - theta2) ** 2);
        };

        const dtheta2 = (p1, p2, theta1, theta2) => {
            return (6 / (this.m2 * this.l2 * this.l2)) * (8 * p2 - 3 * Math.cos(theta1 - theta2) * p1) / (16 - 9 * Math.cos(theta1 - theta2) ** 2);
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
        const x1 = this.l1 * Math.sin(this.theta1);
        const y1 = this.l1 * Math.cos(this.theta1);
        const x2 = x1 + this.l2 * Math.sin(this.theta2);
        const y2 = y1 + this.l2 * Math.cos(this.theta2);

        ctx.beginPath();
        ctx.moveTo(200 + this.offset_x, 200 + this.offset_y);
        ctx.lineTo(200 + this.offset_x + x1 * 100, 200 + this.offset_y + y1 * 100);
        ctx.lineTo(200 + this.offset_x + x2 * 100, 200 + this.offset_y + y2 * 100);
        ctx.stroke();

        // Draw additional elements to increase load
        ctx.beginPath();
        ctx.arc(200 + this.offset_x + x1 * 100, 200 + this.offset_y + y1 * 100, 5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.beginPath();
        ctx.arc(200 + this.offset_x + x2 * 100, 200 + this.offset_y + y2 * 100, 5, 0, 2 * Math.PI);
        ctx.fill();

        // Additional drawing to increase complexity
        for (let i = 0; i < 10; i++) {
            ctx.beginPath();
            ctx.arc(200 + this.offset_x + x1 * 100 + i * 10, 200 + this.offset_y + y1 * 100 + i * 10, 3, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
}

function runSimulation() {
    const canvas = document.getElementById('canvas-js');
    const ctx = canvas.getContext('2d');

    let pendulums = Array.from({ length: 10 }, () => new DoublePendulum(0, 0));
    let lastTime = Date.now();
    let frameCount = 0;
    let totalTime = 0;

    function updateAndDraw() {
        const now = Date.now();
        const deltaTime = now - lastTime;
        lastTime = now;
        totalTime += deltaTime;
        frameCount++;

        ctx.clearRect(0, 0, canvas.width, canvas.height);
        for (const pendulum of pendulums) {
            pendulum.update();
            pendulum.draw(ctx);
        }

        if (frameCount % 60 === 0) {
            const fps = (frameCount / totalTime) * 1000;
            const avgTime = totalTime / frameCount;
            document.getElementById('fps-js').textContent = fps.toFixed(2);
            document.getElementById('avg-time-js').textContent = avgTime.toFixed(2);
        }

        requestAnimationFrame(updateAndDraw);
    }

    updateAndDraw();

    document.getElementById('pendulum-count-js').addEventListener('input', (event) => {
        const count = event.target.valueAsNumber;
        document.getElementById('pendulum-count-display-js').textContent = count.toString();
        pendulums = Array.from({ length: count }, (_, i) => new DoublePendulum((i % 10) * 5, Math.floor(i / 10) * 5));

        // Reset performance counters when the number of pendulums changes
        lastTime = Date.now();
        frameCount = 0;
        totalTime = 0;
    });
}

document.addEventListener('DOMContentLoaded', runSimulation);