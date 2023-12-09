use std::{
    f32::consts::PI,
    io,
    ops::{Add, Div, Mul, Neg, Sub},
};

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
    pos_1: Vec2,
    pos_2: Vec2,
    orientation: Vec2,
    orientation_1: Vec2,
    orientation_2: Vec2,
    los: Vec2,
    los_1: Vec2,
    los_2: Vec2,
}

impl Pod {
    fn update(
        self,
        x: f32,
        y: f32,
        checkpoint_x: f32,
        checkpoint_y: f32,
        checkpoint_angle: f32,
    ) -> Self {
        let pos = Vec2::new(x, y);
        let checkpoint = Vec2::new(checkpoint_x, checkpoint_y);
        let orientation = (checkpoint - pos)
            .rotate_deg(-checkpoint_angle)
            .normalized();
        Self {
            pos,
            pos_1: self.pos,
            pos_2: self.pos_1,
            orientation,
            orientation_1: self.orientation,
            orientation_2: self.orientation_1,
            los: checkpoint - pos,
            los_1: self.los,
            los_2: self.los_1,
        }
    }

    fn pro_nav(&self, target: Vec2) -> (Vec2, f32) {
        let range = target - self.pos;
        let rel_vel = -(self.pos - self.pos_1);
        let rotation_vec = range.outer_product(rel_vel) / range.inner_product(range);
        let acc_norm = Vec2::new(rel_vel.y * rotation_vec, -rel_vel.x * rotation_vec);
        let steer_vec = if acc_norm.norm() < 100.0 {
            range.normalized() * (100.0f32.powi(2) - acc_norm.inner_product(acc_norm)).sqrt()
                + acc_norm
        } else {
            acc_norm.normalized() * 100.0
        };
        let mut thrust =
            (self.orientation.inner_product(range.normalized()).max(0.0) * 5.0).tanh() * 100.0;
        let expected_time = range.norm() / rel_vel.norm();
        if expected_time < 4.0 {
            thrust /= expected_time.max(1.0);
        }
        (steer_vec, thrust)
    }
}

fn main() {
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    let inputs = input_line.split(" ").collect::<Vec<_>>();
    let x = parse_input!(inputs[0], f32);
    let y = parse_input!(inputs[1], f32);
    let checkpoint_x = parse_input!(inputs[2], f32); // x position of the next check point
    let checkpoint_y = parse_input!(inputs[3], f32); // y position of the next check point
                                                     // let next_checkpoint_dist = parse_input!(inputs[4], f32); // distance to the next checkpoint
    let checkpoint_angle = parse_input!(inputs[5], f32); // angle between your pod orientation and the direction of the next checkpoint
    let mut input_line = String::new();
    io::stdin().read_line(&mut input_line).unwrap();
    // let inputs = input_line.split(" ").collect::<Vec<_>>();
    // let opponent_x = parse_input!(inputs[0], i32);
    // let opponent_y = parse_input!(inputs[1], i32);
    let mut pod = Pod::default();
    pod = pod.update(x, y, checkpoint_x, checkpoint_y, checkpoint_angle);
    println!("{:.0} {:.0} BOOST", checkpoint_x, checkpoint_y);
    loop {
        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let inputs = input_line.split(" ").collect::<Vec<_>>();
        let x = parse_input!(inputs[0], f32);
        let y = parse_input!(inputs[1], f32);
        let checkpoint_x = parse_input!(inputs[2], f32); // x position of the next check point
        let checkpoint_y = parse_input!(inputs[3], f32); // y position of the next check point
                                                         // let next_checkpoint_dist = parse_input!(inputs[4], f32); // distance to the next checkpoint
        let checkpoint_angle = parse_input!(inputs[5], f32); // angle between your pod orientation and the direction of the next checkpoint
        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        // let inputs = input_line.split(" ").collect::<Vec<_>>();
        // let opponent_x = parse_input!(inputs[0], i32);
        // let opponent_y = parse_input!(inputs[1], i32);

        pod = pod.update(x, y, checkpoint_x, checkpoint_y, checkpoint_angle);
        let target = Vec2::new(checkpoint_x, checkpoint_y);
        let (steer_vec, thrust) = pod.pro_nav(target);
        // The ten multiplication is just for visuals, it doesn't matter to the navigation
        let point_to = pod.pos + steer_vec;
        println!(
            "{:.0} {:.0} {:.0}",
            point_to.x.round(),
            point_to.y.round(),
            thrust.round()
        );
    }
}
