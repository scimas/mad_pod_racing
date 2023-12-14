use std::{
    f32::consts::PI,
    fmt, io,
    ops::{Add, Div, Mul, Neg, Sub},
};

const MAX_ACCELERAION: f32 = 100.0;
const POD_RADIUS: f32 = 400.0;
const OPPONENTS: usize = 2;
const FUTURE_TIME: f32 = 4.0;
const DRAG_COEF: f32 = 0.85;

macro_rules! parse_input {
    ($x:expr, $t:ident) => {
        $x.trim().parse::<$t>().unwrap()
    };
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Vec2 {
    x: f32,
    y: f32,
}

impl Vec2 {
    fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    fn rotate(self, angle: f32) -> Self {
        let cosine = angle.cos();
        let sine = angle.sin();
        Self::new(
            cosine * self.x - sine * self.y,
            sine * self.x + cosine * self.y,
        )
    }

    fn rotate_deg(self, angle: f32) -> Self {
        self.rotate(angle / 180.0 * PI)
    }

    // This will always be out-of-plane. The "z" component of the 3D vector.
    // That's why it is fine to represent it as a signed scalar for the purposes
    // of this project; at least for now.
    fn outer_product(self, rhs: Self) -> f32 {
        self.x * rhs.y - self.y * rhs.x
    }

    fn inner_product(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    fn norm(self) -> f32 {
        self.inner_product(self).sqrt()
    }

    fn normalized(self) -> Self {
        self / {
            if self.norm() == 0.0 {
                1.0
            } else {
                self.norm()
            }
        }
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}
impl Div<f32> for Vec2 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

#[derive(Debug, Clone, PartialEq)]
struct RaceParameters {
    checkpoints: Vec<Vec2>,
    opponents: Vec<Pod>,
    laps: u8,
}

impl RaceParameters {
    fn new(checkpoints: Vec<Vec2>, opponents: Vec<Pod>, laps: u8) -> Self {
        Self {
            checkpoints,
            opponents,
            laps,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Pod {
    pos: Vec2,
    vel: Vec2,
    accel: f32,
    orientation: Vec2,
    checkpoint_idx: usize,
    lap: u8,
    role: Role,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    Racer,
    Attacker,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Action {
    Accelerate(f32),
    Boost,
    Shield,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Action::Boost => write!(f, "BOOST"),
            Action::Shield => write!(f, "SHIELD"),
            Action::Accelerate(accel) => write!(f, "{:.0}", accel.round()),
        }
    }
}

impl Pod {
    fn new(
        x: f32,
        y: f32,
        vx: f32,
        vy: f32,
        orient_angle: f32,
        checkpoint_idx: usize,
        role: Role,
    ) -> Self {
        Self {
            pos: Vec2::new(x, y),
            vel: Vec2::new(vx, vy),
            accel: MAX_ACCELERAION,
            orientation: Vec2::new(1.0, 0.0).rotate_deg(orient_angle),
            checkpoint_idx,
            lap: 0,
            role,
        }
    }

    fn attacker() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0, Role::Attacker)
    }

    fn racer() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0, Role::Racer)
    }

    fn update(
        &mut self,
        x: f32,
        y: f32,
        vx: f32,
        vy: f32,
        orient_angle: f32,
        checkpoint_idx: usize,
    ) {
        self.lap = if checkpoint_idx != self.checkpoint_idx {
            self.lap + 1
        } else {
            self.lap
        };
        self.pos = Vec2::new(x, y);
        self.vel = Vec2::new(vx, vy);
        self.orientation = Vec2::new(1.0, 0.0).rotate_deg(orient_angle);
        self.checkpoint_idx = checkpoint_idx;
    }

    fn navigate(&mut self, parameters: &RaceParameters) -> (Vec2, Action) {
        let nav_target;
        let rel_vel;
        match self.role {
            Role::Racer => {
                let current_cp = parameters.checkpoints[self.checkpoint_idx];
                let next_cp = parameters.checkpoints
                    [(self.checkpoint_idx + 1) % parameters.checkpoints.len()];
                nav_target = current_cp + (next_cp - current_cp).normalized() * POD_RADIUS;
                rel_vel = -self.vel;
            }
            Role::Attacker => {
                let pod = prioritize_opponent(parameters);
                rel_vel = pod.vel - self.vel;
                if (pod.pos - self.pos)
                    .normalized()
                    .inner_product(self.vel.normalized())
                    > 0.8
                {
                    let cp_diff = parameters.checkpoints
                        [(pod.checkpoint_idx + 1) % parameters.checkpoints.len()]
                        - parameters.checkpoints[pod.checkpoint_idx];
                    nav_target = parameters.checkpoints[pod.checkpoint_idx]
                        + cp_diff.normalized() * (cp_diff.norm() / 2.0);
                } else {
                    let cp_range = parameters.checkpoints[pod.checkpoint_idx] - pod.pos;
                    nav_target = pod.pos + pod.vel + cp_range.normalized() * POD_RADIUS;
                }
            }
        };
        let range = nav_target - self.pos;
        let rotation_vec = range.outer_product(rel_vel) / range.inner_product(range);
        let acc_norm = Vec2::new(rel_vel.y * rotation_vec, -rel_vel.x * rotation_vec);

        let mut steer_vec = self.pos
            + if acc_norm.norm() < MAX_ACCELERAION {
                range.normalized()
                    * (MAX_ACCELERAION.powi(2) - acc_norm.inner_product(acc_norm)).sqrt()
                    + acc_norm
            } else {
                acc_norm.normalized() * MAX_ACCELERAION
            };

        let accel = match self.role {
            Role::Racer => {
                if self.flight_time(range.norm()) < FUTURE_TIME {
                    steer_vec = parameters.checkpoints
                        [(self.checkpoint_idx + 1) % parameters.checkpoints.len()];
                }
                (self
                    .orientation
                    .inner_product((steer_vec - self.pos).normalized())
                    .powi(4)
                    * 16.0)
                    .tanh()
                    * MAX_ACCELERAION
            }
            Role::Attacker => {
                (self
                    .orientation
                    .inner_product((steer_vec - self.pos).normalized())
                    .powi(4)
                    * 16.0)
                    .tanh()
                    * MAX_ACCELERAION
            }
        };
        self.accel = accel;

        let action = if parameters
            .opponents
            .iter()
            .map(|pod| {
                (
                    (pod.pos + pod.vel) - (self.pos + self.vel),
                    self.orientation,
                )
            })
            .any(|(range, orientation)| {
                (orientation.inner_product(range) > 0.0) && (range.norm() <= POD_RADIUS * 2.2)
            }) {
            Action::Shield
        } else {
            Action::Accelerate(accel)
        };
        (steer_vec, action)
    }

    /// Approximate time it will take to travel `distance` assuming current
    /// thrust with no direction change.
    fn flight_time(&self, distance: f32) -> f32 {
        // This will never be smaller than 1.0.
        let accel = self.accel.max(1.0);
        distance * (1.0 - DRAG_COEF) / DRAG_COEF / accel - self.vel.norm() / accel
            + DRAG_COEF / (1.0 - DRAG_COEF)
    }
}

fn main() {
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let laps = parse_input!(input_line, u8);
    input_line.clear();
    io::stdin().read_line(&mut input_line).unwrap();
    let checkpoint_n = parse_input!(input_line, usize);
    let mut checkpoints = Vec::with_capacity(checkpoint_n);
    for _ in 0..checkpoint_n {
        input_line.clear();
        io::stdin().read_line(&mut input_line).unwrap();
        let inputs = input_line.split(' ').collect::<Vec<_>>();
        let x = parse_input!(inputs[0], f32);
        let y = parse_input!(inputs[1], f32);
        checkpoints.push(Vec2::new(x, y));
    }

    let mut racer = Pod::racer();
    update_pod(&mut racer);

    let mut attacker = Pod::attacker();
    update_pod(&mut attacker);

    let opponents: Vec<Pod> = (0..OPPONENTS)
        .map(|_| {
            let mut opponent = Pod::racer();
            update_pod(&mut opponent);
            opponent
        })
        .collect();

    let mut parameters = RaceParameters::new(checkpoints, opponents, laps);
    println!(
        "{:.0} {:.0} {}",
        parameters.checkpoints[racer.checkpoint_idx].x,
        parameters.checkpoints[racer.checkpoint_idx].y,
        Action::Boost
    );
    println!(
        "{:.0} {:.0} {}",
        parameters.checkpoints[racer.checkpoint_idx].x,
        parameters.checkpoints[racer.checkpoint_idx].y,
        Action::Accelerate(100.0)
    );
    loop {
        update_pod(&mut racer);
        update_pod(&mut attacker);
        parameters
            .opponents
            .iter_mut()
            .for_each(|opponent| update_pod(opponent));

        let (racer_steer_vec, racer_action) = racer.navigate(&parameters);
        let (attacker_steer_vec, attacker_action) = attacker.navigate(&parameters);

        println!(
            "{:.0} {:.0} {}",
            racer_steer_vec.x.round(),
            racer_steer_vec.y.round(),
            racer_action
        );
        println!(
            "{:.0} {:.0} {}",
            attacker_steer_vec.x.round(),
            attacker_steer_vec.y.round(),
            attacker_action
        );
    }
}

fn update_pod(pod: &mut Pod) {
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let inputs = input_line.split(' ').collect::<Vec<_>>();
    let x = parse_input!(inputs[0], f32);
    let y = parse_input!(inputs[1], f32);
    let vx = parse_input!(inputs[2], f32);
    let vy = parse_input!(inputs[3], f32);
    let orient_angle = parse_input!(inputs[4], f32);
    let checkpoint_idx = parse_input!(inputs[5], usize);
    pod.update(x, y, vx, vy, orient_angle, checkpoint_idx)
}

fn prioritize_opponent(parameters: &RaceParameters) -> &Pod {
    let max_lap = parameters
        .opponents
        .iter()
        .map(|pod| pod.lap)
        .max()
        .unwrap();
    let max_checkpoint = parameters
        .opponents
        .iter()
        .filter(|pod| pod.lap == max_lap)
        .map(|pod| pod.checkpoint_idx)
        .max()
        .unwrap();
    parameters
        .opponents
        .iter()
        .filter(|pod| (pod.lap == max_lap) && (pod.checkpoint_idx == max_checkpoint))
        .min_by(|pod1, pod2| {
            (pod1.pos - parameters.checkpoints[max_checkpoint])
                .norm()
                .partial_cmp(&(pod2.pos - parameters.checkpoints[max_checkpoint]).norm())
                .unwrap()
        })
        .unwrap()
}
