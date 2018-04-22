extern crate rulinalg;
extern crate piston_window;

use ik::rulinalg::matrix::Matrix;
use rulinalg::matrix::BaseMatrix;
use std::cell::RefCell;
use std::rc::Rc;
use ik::rulinalg::norm::Euclidean;

use piston_window::*;

#[derive(Debug)]
pub struct Rotator {
    angle: f64,
    pub position: Matrix<f64>,
    pub child: Option<Rc<RefCell<Rotator>>>
}

impl Rotator {
    pub fn new(position: Matrix<f64>) -> Rotator {
        // 2D Column Vector
        assert!(position.cols() == 1 && position.rows() == 2);

        Rotator {
            angle: 0.0,
            position,
            child: None,
        }
    }

    pub fn set_child(&mut self, child: Rc<RefCell<Rotator>>) {
        self.child = Some(child);
    }

    pub fn rotate(&mut self, change_angle: f64) {
        self.angle += change_angle;

        //a raw pointer that traverses down "linked list" of rotator assembly
        let mut current = (&mut self.child) as *mut Option<Rc<RefCell<Rotator>>>;
        let rotation_matrix = Matrix::new(2, 2, vec![
            change_angle.cos(), -1.0 * change_angle.sin(),
            change_angle.sin(), change_angle.cos()
        ]);

        unsafe {
            while let Some(ref mut c) = *current {
                let new_position: Matrix<f64> = &rotation_matrix * (&c.borrow().position - &self.position) + &self.position;
                c.borrow_mut().position = new_position;

                current = (&mut c.borrow_mut().child) as *mut Option<Rc<RefCell<Rotator>>>;
            }
        }
    }

    pub fn reset(&mut self) {
        let reset_angle = -1.0 * self.angle;
        self.rotate(reset_angle);
    }

    pub fn create_jacobian(&self, end_effector: &Rotator) -> Matrix<f64> {
        let slope: Matrix<f64> = &end_effector.position - &self.position;
        let x_slope = slope.data()[0];
        let y_slope = slope.data()[1];

        //Get normalized 2d vector perpendicular to slope that follows right hand rule
        return Matrix::new(2, 1, vec![-1.0 * y_slope, x_slope]) / slope.norm(Euclidean);

    }
}

pub struct Assembly {
    pub rotators: Vec<Rc<RefCell<Rotator>>>
}

impl Assembly {
    pub fn new() -> Assembly {
        Assembly{
            rotators: Vec::new()
        }
    }

    pub fn add_rotator(&mut self, position: Matrix<f64>) {
        // 2D Column Vector
        assert!(position.cols() == 1 && position.rows() == 2);

        let rotator: Rotator = Rotator::new(position);
        let rotator_mut = Rc::new(RefCell::new(rotator));

        if self.rotators.len() > 0 {
            self.rotators.last().unwrap().borrow_mut().set_child(rotator_mut.clone());
            self.rotators.push(rotator_mut);
        } else {
            self.rotators.push(rotator_mut);
        }
    }

    pub fn solve(&mut self, goal: Matrix<f64>, window: &mut PistonWindow, window_settings: &RenderWindow) {
        assert!(self.rotators.len() >= 1);

        let step_size = 0.01;
        let convergence_threshold = 1e-3;

        loop {
            //Set delta_angles and last_position to dummy values ... they will be changed
            let mut delta_angles: Matrix<f64> = Matrix::new(0,0,vec![]);
            let mut last_position: Matrix<f64> = Matrix::new(0,0,vec![]);

            //In separate block to dereference joints in assembly so that update step will not violate Rust's mutation rules
            {
                //Generate Jacobian Matrix
                let end_effector: &Rotator = &self.rotators.last().unwrap().borrow();
                let mut jacobian_matrix: Matrix<f64> = self.rotators[0].borrow().create_jacobian(&end_effector);

                //Go from [1, rotators.len-1) as 0th was already added and the end effector cannot manipulate its own position
                for i in 1..(self.rotators.len() - 1) {
                    jacobian_matrix = jacobian_matrix.hcat(&self.rotators[i].borrow().create_jacobian(&end_effector));
                }

                //Use Jacobian Transpose method to find the delta angles
                delta_angles = &jacobian_matrix.transpose() * (&goal - &end_effector.position);

                last_position = end_effector.position.clone();
            }

            //Jacobian IK update step
            for (idx, angle) in delta_angles.data().iter().enumerate() {
                self.rotators[idx].borrow_mut().rotate(step_size * *angle);

                //Render the assembly at this step
                if let Some(e) = window.next() {
                    window.draw_2d(&e, |c, g| {
                        clear([1.0; 4], g);
                        Assembly::render(&self.rotators, &window_settings, &c, g);
                    });
                }
            }

            let current_position: &Matrix<f64> = &self.rotators.last().unwrap().borrow().position;
//            println!("Current Position: {:?}", current_position.data());

            if (&last_position - current_position).norm(Euclidean).abs() < convergence_threshold {
                break;
            }
        }
        println!("Finished Solving!");
    }

    //Some goals cannot be obtained with Jacobian Transpose IK on first pass ... an intermediate point needs to be met
    //so that the solver can reach the goal
    pub fn create_path(&mut self, goal: Matrix<f64>, window: &mut PistonWindow, windows_settings: &RenderWindow) {
        assert!(self.rotators.len() >= 1);

        let current_position: Matrix<f64> = self.rotators.last().unwrap().borrow().position.clone();
        let max_y = current_position.data()[1].max(goal.data()[1]);
        let mid_x = (current_position.data()[0] + goal.data()[0]) / 2.0;

        self.solve(Matrix::new(2, 1, vec![mid_x, max_y + 2.0]), window, &windows_settings);
        self.solve(goal, window, &windows_settings);
    }

    //Function to render the assembly
    pub fn render(rotators: &Vec<Rc<RefCell<Rotator>>>,
                  windows_settings: &RenderWindow,
                  context: &Context, graphics: &mut G2d) {

        for rotator in rotators {
            let position_vec = &rotator.borrow().position;
            let position = windows_settings.convert_computer_coordinate(position_vec.data()[0], position_vec.data()[1]);
            let radius = 5.0;
            graphics.ellipse(&Ellipse::new([1.0,0.0,0.0,1.0]), [position.0 - radius, position.1 - radius, 2.0*radius, 2.0*radius], &context.draw_state, context.transform);
        }
    }
}

//Struct that handles camera looking at scene ... IK solver uses Cartesian plane and that coordinate system
//needs to be converted to the computer screen coordinate system
//
//Fields are like the window settings in graphing calculator
pub struct RenderWindow {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
    pub screen_width: f64,
    pub screen_height: f64
}

impl RenderWindow {
    pub fn new(x_min: f64, x_max: f64, y_min: f64, y_max: f64, screen_width: f64, screen_height: f64) -> RenderWindow {
        RenderWindow {
            x_min,
            x_max,
            y_min,
            y_max,
            screen_width,
            screen_height
        }
    }

    //Convert Cartesian coordinate to computer screen coordinate
    pub fn convert_computer_coordinate(&self, cartesian_x: f64, cartesian_y: f64) -> (f64, f64) {
        let new_x = ((self.screen_width - 0.0) / (self.x_max - self.x_min))*(cartesian_x - self.x_min) + 0.0;
        let new_y = -1.0 * ((self.screen_height - 0.0) / (self.y_max - self.y_min))*(cartesian_y - self.y_max) + 0.0;

        return (new_x , new_y);
    }

    //Convert computer screen coordinate to Cartesian coordinate
    pub fn convert_cartesian(&self, screen_x: f64, screen_y: f64) -> (f64, f64) {
        let new_x = ((self.x_max - self.x_min) / (self.screen_width - 0.0))*(screen_x - 0.0) + self.x_min;
        let new_y = -1.0 * ((self.y_max - self.y_min) / (self.screen_height - 0.0))*(screen_y - 0.0) + self.y_max;

        return (new_x , new_y);
    }

}