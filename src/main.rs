mod ik;
extern crate rulinalg;
extern crate piston_window;

use piston_window::*;

use ik::Assembly;
use ik::RenderWindow;
use self::rulinalg::matrix::Matrix;

fn main() {
    /***Demonstration of rotation calculation for Rotator***/

    //    let a = Rotator::new(Matrix::new(2, 1, vec![0.0, 0.0]));
    //    let b = Rotator::new(Matrix::new(2, 1, vec![1.0, 0.0]));
    //    let c = Rotator::new(Matrix::new(2, 1, vec![2.0, 0.0]));
    //    let d = Rotator::new(Matrix::new(2, 1, vec![3.0, 0.0]));
    //
    //    let a_mut = Rc::new(RefCell::new(a));
    //    let b_mut = Rc::new(RefCell::new(b));
    //    let c_mut = Rc::new(RefCell::new(c));
    //    let d_mut = Rc::new(RefCell::new(d));
    //
    //    a_mut.borrow_mut().set_child(b_mut.clone());
    //    b_mut.borrow_mut().set_child(c_mut.clone());
    //    c_mut.borrow_mut().set_child(d_mut.clone());
    //
    //
    //    a_mut.borrow_mut().rotate(0.0002590399289101735);
    //    b_mut.borrow_mut().rotate(0.22152217440613245);
    //    c_mut.borrow_mut().rotate(3.1415926535 / 2.0);
    //
    //    println!("{:#?}", a_mut);

    let mut window: PistonWindow =
        WindowSettings::new("IK Solver", [500, 500])
            .exit_on_esc(true).build().unwrap();

    let mut assembly = Assembly::new();
    assembly.add_rotator(Matrix::new(2, 1, vec![0.0, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![1.0, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![2.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![3.333, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![4.0, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![5.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![6.666, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![7.0, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![8.0, 0.0]));
//    assembly.add_rotator(Matrix::new(2, 1, vec![9.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![10.0, 0.0]));

    let mut target: Matrix<f64> = Matrix::new(2, 1, vec![10.0, 0.0]);

    //Sequence of paths that form a square
    let parametric_path1 = |t: f64| -> Matrix<f64> {
        let x = -1.0*t + 5.0;
        let y = 5.0;

        return Matrix::new(2, 1, vec![x, y]);
    };
    let parametric_path2 = |t: f64| -> Matrix<f64> {
        let x = -5.0;
        let y = -1.0*t + 5.0;

        return Matrix::new(2, 1, vec![x, y]);
    };
    let parametric_path3 = |t: f64| -> Matrix<f64> {
        let x = t - 5.0;
        let y = -5.0;

        return Matrix::new(2, 1, vec![x, y]);
    };
    let parametric_path4 = |t: f64| -> Matrix<f64> {
        let x = 5.0;
        let y = t - 5.0;

        return Matrix::new(2, 1, vec![x, y]);
    };

    let path1 = Assembly::generate_path(&parametric_path1, 0.0, 10.0, 0.1);
    let path2 = Assembly::generate_path(&parametric_path2, 0.0, 10.0, 0.1);
    let path3 = Assembly::generate_path(&parametric_path3, 0.0, 10.0, 0.1);
    let path4 = Assembly::generate_path(&parametric_path4, 0.0, 10.0, 0.1);

    let mut events = Events::new(EventSettings::new().lazy(true));
    while let Some(e) = events.next(&mut window) {
        let window_size = window.size();
        let camera = RenderWindow::new(-10.0, 10.0, -10.0, 10.0, window_size.width as f64, window_size.height as f64);

        //Create a target for rig
        if let Some(Button::Mouse(MouseButton::Left)) = e.press_args() {
            assembly.solve(&target, &mut window, &camera);
        }
        //Track Mouse Position
        e.mouse_cursor(|x, y| {
            let converted_target = camera.convert_cartesian(x, y);
            target = Matrix::new(2, 1, vec![converted_target.0, converted_target.1]);
        });
        //Reset joints back to original position
        if let Some(Button::Keyboard(Key::Space)) = e.press_args() {
            for rotator in &assembly.rotators {
                rotator.borrow_mut().reset();
            }
        }

        //Follow Path ... comment out if you want an ik playground
        assembly.follow_path(&path1, &mut window, &camera);
        assembly.follow_path(&path2, &mut window, &camera);
        assembly.follow_path(&path3, &mut window, &camera);
        assembly.follow_path(&path4, &mut window, &camera);

        //Render Assembly
        if let Some(e_w) = window.next() {
            window.draw_2d(&e_w, |c, g| {
                clear([1.0; 4], g);
                Assembly::render(&assembly.rotators, &camera, &c, g);
            });
        }
    }


    println!("{:#?}", assembly.rotators.get(0).unwrap());
}