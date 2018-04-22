mod ik;
extern crate rulinalg;
extern crate piston_window;

use piston_window::*;

use ik::Assembly;
use ik::RenderWindow;
use self::rulinalg::matrix::Matrix;

fn main() {
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
    assembly.add_rotator(Matrix::new(2, 1, vec![1.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![2.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![3.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![4.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![5.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![6.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![7.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![8.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![9.0, 0.0]));
    assembly.add_rotator(Matrix::new(2, 1, vec![10.0, 0.0]));

    let mut target: Matrix<f64> = Matrix::new(2, 1, vec![10.0, 0.0]);

    let mut events = Events::new(EventSettings::new().lazy(true));
    while let Some(e) = events.next(&mut window) {
        let window_size = window.size();
        let camera = RenderWindow::new(-10.0, 10.0, -10.0, 10.0, window_size.width as f64, window_size.height as f64);

        //Create a target for rig
        if let Some(Button::Mouse(MouseButton::Left)) = e.press_args() {
            assembly.solve(target.clone(), &mut window, &camera);
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