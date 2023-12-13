use std::{
    f32::consts::PI,
    fmt, io,
    ops::{Add, Div, Mul, Neg, Sub},
};

const MAX_THRUST: f32 = 100.0;
const POD_RADIUS: f32 = 400.0;
const OPPONENTS: usize = 2;

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

#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Pod {
    pos: Vec2,
    vel: Vec2,
    orientation: Vec2,
    checkpoint_idx: usize,
    lap: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Target {
    Pod(Pod),
    Checkpoint(Vec2),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Action {
    Thrust(f32),
    Boost,
    Shield,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Action::Boost => write!(f, "BOOST"),
            Action::Shield => write!(f, "SHIELD"),
            Action::Thrust(thrust) => write!(f, "{:.0}", thrust.round()),
        }
    }
}

impl Pod {
    fn update(
        self,
        x: f32,
        y: f32,
        vx: f32,
        vy: f32,
        orient_angle: f32,
        checkpoint_idx: usize,
    ) -> Self {
        let lap = if checkpoint_idx != self.checkpoint_idx {
            self.lap + 1
        } else {
            self.lap
        };
        Self {
            pos: Vec2::new(x, y),
            vel: Vec2::new(vx, vy),
            orientation: Vec2::new(1.0, 0.0).rotate_deg(orient_angle),
            checkpoint_idx,
            lap,
        }
    }

    fn navigate(&self, target: Target, checkpoints: &[Vec2], opponents: &[Pod]) -> (Vec2, Action) {
        let actual_target = match target {
            Target::Checkpoint(cp) => {
                let next_cp = checkpoints[(self.checkpoint_idx + 1) % checkpoints.len()];
                cp + (next_cp - cp).normalized() * (POD_RADIUS / 2.0)
            }
            Target::Pod(pod) => {
                if (pod.pos - self.pos)
                    .normalized()
                    .inner_product(self.vel.normalized())
                    > 0.8
                {
                    let cp_diff = checkpoints[(pod.checkpoint_idx + 1) % checkpoints.len()]
                        - checkpoints[pod.checkpoint_idx];
                    checkpoints[pod.checkpoint_idx] + cp_diff.normalized() * (cp_diff.norm() / 2.0)
                } else {
                    let cp_range = checkpoints[pod.checkpoint_idx] - pod.pos;
                    pod.pos + pod.vel + cp_range.normalized() * POD_RADIUS
                }
            }
        };
        let mut range = actual_target - self.pos;
        let rel_vel = match target {
            Target::Checkpoint(_) => -self.vel,
            Target::Pod(pod) => pod.vel - self.vel,
        };
        let rotation_vec = range.outer_product(rel_vel) / range.inner_product(range);
        let acc_norm = Vec2::new(rel_vel.y * rotation_vec, -rel_vel.x * rotation_vec);
        let mut steer_vec = self.pos
            + if acc_norm.norm() < MAX_THRUST {
                range.normalized() * (MAX_THRUST.powi(2) - acc_norm.inner_product(acc_norm)).sqrt()
                    + acc_norm
            } else {
                acc_norm.normalized() * MAX_THRUST
            };
        let expected_time = range.norm() / self.vel.norm();
        if expected_time < 3.0 {
            steer_vec = checkpoints[(self.checkpoint_idx + 1) % checkpoints.len()];
            range = checkpoints[(self.checkpoint_idx + 1) % checkpoints.len()] - self.pos;
        }
        let thrust =
            (self.orientation.inner_product(range.normalized()).max(0.0) * 6.0).tanh() * MAX_THRUST;
        let action = if opponents
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
            Action::Thrust(thrust)
        };
        (steer_vec, action)
    }
}

fn main() {
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let _laps = parse_input!(input_line, u8);
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

    let mut racer = Pod::default();
    racer = update_pod(racer);

    let mut attacker = Pod::default();
    attacker = update_pod(attacker);

    let mut opponents: Vec<Pod> = (0..OPPONENTS)
        .map(|_| {
            let opponent = Pod::default();
            update_pod(opponent)
        })
        .collect();

    println!(
        "{:.0} {:.0} {}",
        checkpoints[racer.checkpoint_idx].x,
        checkpoints[racer.checkpoint_idx].y,
        Action::Boost
    );
    println!(
        "{:.0} {:.0} {}",
        checkpoints[racer.checkpoint_idx].x,
        checkpoints[racer.checkpoint_idx].y,
        Action::Thrust(100.0)
    );
    loop {
        racer = update_pod(racer);
        attacker = update_pod(attacker);
        opponents
            .iter_mut()
            .for_each(|opponent| *opponent = update_pod(*opponent));

        let target = Target::Checkpoint(checkpoints[racer.checkpoint_idx]);
        let (racer_steer_vec, racer_action) = racer.navigate(target, &checkpoints, &opponents);

        let target = Target::Pod(*prioritize_opponent(&opponents, &checkpoints));
        let (attacker_steer_vec, attacker_action) =
            attacker.navigate(target, &checkpoints, &opponents);

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

fn update_pod(pod: Pod) -> Pod {
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

fn prioritize_opponent<'a>(opponents: &'a [Pod], checkpoints: &[Vec2]) -> &'a Pod {
    let max_lap = opponents.iter().map(|pod| pod.lap).max().unwrap();
    let max_checkpoint = opponents
        .iter()
        .filter(|pod| pod.lap == max_lap)
        .map(|pod| pod.checkpoint_idx)
        .max()
        .unwrap();
    opponents
        .iter()
        .filter(|pod| (pod.lap == max_lap) && (pod.checkpoint_idx == max_checkpoint))
        .min_by(|pod1, pod2| {
            (pod1.pos - checkpoints[max_checkpoint])
                .norm()
                .partial_cmp(&(pod2.pos - checkpoints[max_checkpoint]).norm())
                .unwrap()
        })
        .unwrap()
}
