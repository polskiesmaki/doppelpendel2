use wasm_bindgen::prelude::*;
use web_sys::{CanvasRenderingContext2d, HtmlInputElement, HtmlCanvasElement, Event};
use std::rc::Rc;
use std::cell::RefCell;
use js_sys::Date;

#[wasm_bindgen]
#[derive(Clone)]
pub struct DoublePendulum {
    theta1: f64,
    theta2: f64,
    p1: f64,
    p2: f64,
    m1: f64,
    m2: f64,
    l1: f64,
    l2: f64,
    g: f64,
    offset_x: f64,
    offset_y: f64,
}

#[wasm_bindgen]
impl DoublePendulum {
    pub fn new(offset_x: f64, offset_y: f64) -> DoublePendulum {
        DoublePendulum {
            theta1: std::f64::consts::PI / 2.0,
            theta2: std::f64::consts::PI / 2.0,
            p1: 0.0,
            p2: 0.0,
            m1: 1.0,
            m2: 1.0,
            l1: 1.0,
            l2: 1.0,
            g: 9.81,
            offset_x,
            offset_y,
        }
    }

    pub fn rk4_step(&mut self) {
        let dt = 0.01;

        let dtheta1 = |p1: f64, p2: f64, theta1: f64, theta2: f64| {
            (6.0 / (self.m1 * self.l1 * self.l1)) * (2.0 * p1 - 3.0 * (theta1 - theta2).cos() * p2) / (16.0 - 9.0 * (theta1 - theta2).cos().powi(2))
        };

        let dtheta2 = |p1: f64, p2: f64, theta1: f64, theta2: f64| {
            (6.0 / (self.m2 * self.l2 * self.l2)) * (8.0 * p2 - 3.0 * (theta1 - theta2).cos() * p1) / (16.0 - 9.0 * (theta1 - theta2).cos().powi(2))
        };

        let dp1 = |p1: f64, p2: f64, theta1: f64, theta2: f64| {
            -(self.m1 + self.m2) * self.g * self.l1 * theta1.sin() - dtheta1(p1, p2, theta1, theta2) * dtheta2(p1, p2, theta1, theta2) * (theta1 - theta2).sin()
        };

        let dp2 = |p1: f64, p2: f64, theta1: f64, theta2: f64| {
            -self.m2 * self.g * self.l2 * theta2.sin() + dtheta1(p1, p2, theta1, theta2) * dtheta2(p1, p2, theta1, theta2) * (theta1 - theta2).sin()
        };

        let k1_theta1 = dt * dtheta1(self.p1, self.p2, self.theta1, self.theta2);
        let k1_theta2 = dt * dtheta2(self.p1, self.p2, self.theta1, self.theta2);
        let k1_p1 = dt * dp1(self.p1, self.p2, self.theta1, self.theta2);
        let k1_p2 = dt * dp2(self.p1, self.p2, self.theta1, self.theta2);

        let k2_theta1 = dt * dtheta1(self.p1 + 0.5 * k1_p1, self.p2 + 0.5 * k1_p2, self.theta1 + 0.5 * k1_theta1, self.theta2 + 0.5 * k1_theta2);
        let k2_theta2 = dt * dtheta2(self.p1 + 0.5 * k1_p1, self.p2 + 0.5 * k1_p2, self.theta1 + 0.5 * k1_theta1, self.theta2 + 0.5 * k1_theta2);
        let k2_p1 = dt * dp1(self.p1 + 0.5 * k1_p1, self.p2 + 0.5 * k1_p2, self.theta1 + 0.5 * k1_theta1, self.theta2 + 0.5 * k1_theta2);
        let k2_p2 = dt * dp2(self.p1 + 0.5 * k1_p1, self.p2 + 0.5 * k1_p2, self.theta1 + 0.5 * k1_theta1, self.theta2 + 0.5 * k1_theta2);

        let k3_theta1 = dt * dtheta1(self.p1 + 0.5 * k2_p1, self.p2 + 0.5 * k2_p2, self.theta1 + 0.5 * k2_theta1, self.theta2 + 0.5 * k2_theta2);
        let k3_theta2 = dt * dtheta2(self.p1 + 0.5 * k2_p1, self.p2 + 0.5 * k2_p2, self.theta1 + 0.5 * k2_theta1, self.theta2 + 0.5 * k2_theta2);
        let k3_p1 = dt * dp1(self.p1 + 0.5 * k2_p1, self.p2 + 0.5 * k2_p2, self.theta1 + 0.5 * k2_theta1, self.theta2 + 0.5 * k2_theta2);
        let k3_p2 = dt * dp2(self.p1 + 0.5 * k2_p1, self.p2 + 0.5 * k2_p2, self.theta1 + 0.5 * k2_theta1, self.theta2 + 0.5 * k2_theta2);

        let k4_theta1 = dt * dtheta1(self.p1 + k3_p1, self.p2 + k3_p2, self.theta1 + k3_theta1, self.theta2 + k3_theta2);
        let k4_theta2 = dt * dtheta2(self.p1 + k3_p1, self.p2 + k3_p2, self.theta1 + k3_theta1, self.theta2 + k3_theta2);
        let k4_p1 = dt * dp1(self.p1 + k3_p1, self.p2 + k3_p2, self.theta1 + k3_theta1, self.theta2 + k3_theta2);
        let k4_p2 = dt * dp2(self.p1 + k3_p1, self.p2 + k3_p2, self.theta1 + k3_theta1, self.theta2 + k3_theta2);

        self.theta1 += (k1_theta1 + 2.0 * k2_theta1 + 2.0 * k3_theta1 + k4_theta1) / 6.0;
        self.theta2 += (k1_theta2 + 2.0 * k2_theta2 + 2.0 * k3_theta2 + k4_theta2) / 6.0;
        self.p1 += (k1_p1 + 2.0 * k2_p1 + 2.0 * k3_p1 + k4_p1) / 6.0;
        self.p2 += (k1_p2 + 2.0 * k2_p2 + 2.0 * k3_p2 + k4_p2) / 6.0;
    }

    pub fn update(&mut self) {
        self.rk4_step();
    }

    pub fn draw(&self, ctx: &CanvasRenderingContext2d) {
        let x1 = self.l1 * self.theta1.sin();
        let y1 = self.l1 * self.theta1.cos();
        let x2 = x1 + self.l2 * self.theta2.sin();
        let y2 = y1 + self.l2 * self.theta2.cos();

        ctx.begin_path();
        ctx.move_to(200.0 + self.offset_x, 200.0 + self.offset_y);
        ctx.line_to(200.0 + self.offset_x + x1 * 100.0, 200.0 + self.offset_y + y1 * 100.0);
        ctx.line_to(200.0 + self.offset_x + x2 * 100.0, 200.0 + self.offset_y + y2 * 100.0);
        ctx.stroke();
    }
}

#[wasm_bindgen]
pub fn run_simulation() {
    let window = web_sys::window().expect("no global `window` exists");
    
    let document = window.document().expect("should have a document on window");
    let document_clone = document.clone();
    let document_clone_for_closure = document.clone();
    let canvas = document.get_element_by_id("canvas-wasm").unwrap();
    let canvas: HtmlCanvasElement = canvas.dyn_into::<HtmlCanvasElement>().unwrap();
    let ctx = canvas.get_context("2d").unwrap().unwrap().dyn_into::<CanvasRenderingContext2d>().unwrap();

    let pendulums = Rc::new(RefCell::new(vec![DoublePendulum::new(0.0, 0.0); 10]));
    let pendulums_clone = Rc::clone(&pendulums);
    let mut last_time = Date::now();
    let mut frame_count = 0;
    let mut total_time = 0.0;

    let f = Rc::new(RefCell::new(None));
    let g = f.clone();

    *g.borrow_mut() = Some(Closure::wrap(Box::new(move || {
        let now = Date::now();
        let delta_time = now - last_time;
        last_time = now;
        total_time += delta_time;
        frame_count += 1;

        ctx.clear_rect(0.0, 0.0, canvas.width().into(), canvas.height().into());
        for pendulum in &mut *pendulums.borrow_mut() {
            pendulum.update();
            pendulum.draw(&ctx);
        }

        if frame_count % 60 == 0 {
            let fps = (frame_count as f64 / total_time) * 1000.0;
            let avg_time = total_time / frame_count as f64;
            document_clone.get_element_by_id("fps-wasm").unwrap().set_text_content(Some(&format!("{:.2}", fps)));
            document_clone.get_element_by_id("avg-time-wasm").unwrap().set_text_content(Some(&format!("{:.2}", avg_time)));
        }

        request_animation_frame(f.borrow().as_ref().unwrap());
    }) as Box<dyn FnMut()>));

    request_animation_frame(g.borrow().as_ref().unwrap());

    let closure = Closure::wrap(Box::new(move |event: Event| {
        let input = event.target().unwrap().dyn_into::<HtmlInputElement>().unwrap();
        let count = input.value_as_number() as usize;
        document_clone_for_closure.get_element_by_id("pendulum-count-display-wasm").unwrap().set_text_content(Some(&count.to_string()));
        let mut new_pendulums = vec![];
        for i in 0..count {
            let offset_x = (i % 10) as f64 * 5.0;
            let offset_y = (i / 10) as f64 * 5.0;
            new_pendulums.push(DoublePendulum::new(offset_x, offset_y));
        }
        *pendulums_clone.borrow_mut() = new_pendulums;
    }) as Box<dyn FnMut(_)>);

    document.get_element_by_id("pendulum-count-wasm").unwrap().add_event_listener_with_callback("input", closure.as_ref().unchecked_ref()).unwrap();
    closure.forget();

}

fn request_animation_frame(f: &Closure<dyn FnMut()>) {
    web_sys::window().unwrap().request_animation_frame(f.as_ref().unchecked_ref()).unwrap();
}
