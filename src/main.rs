use anyhow::Result;
use robot_pingpong::predictor::Predictor;

fn main() -> Result<()> {
    #[cfg(feature = "dry-run")]
    {
        eprintln!("dry-run: skipping hardware/camera main loop");
        let mut predictor = Predictor::new();
        predictor.add_ball_position(robot_pingpong::predictor::Vec3::new(0.5, 0.7, 0.2));
        predictor.add_ball_position(robot_pingpong::predictor::Vec3::new(0.8, 0.75, 0.25));
        predictor.add_ball_position(robot_pingpong::predictor::Vec3::new(1.1, 0.8, 0.22));
        println!(
            "dry-run ok: y={:?} z={:?} hit={}",
            predictor.predict_y(),
            predictor.predict_z(),
            predictor.hit_target()
        );
        return Ok(());
    }

    #[cfg(not(feature = "dry-run"))]
    {
        run_hardware_loop()
    }
}

#[cfg(not(feature = "dry-run"))]
fn run_hardware_loop() -> Result<()> {
    use robot_pingpong::constants::Y_TABLE_SIZE;
    use robot_pingpong::control::{Arm, LinearMotor};
    use robot_pingpong::utils::Timer;
    use robot_pingpong::vision::{Vision, Visualizer};

    let arm = Arm::new()?;
    arm.init()?;
    let mut lm = LinearMotor::new(0)?;
    let mut predictor = Predictor::new();
    let mut visualizer = Visualizer::new()?;
    let mut vision = Vision::new()?;
    vision.init(&mut visualizer, false)?;

    lm.on();

    let mut timer = Timer::new();
    loop {
        match vision.track()? {
            Some(ball_position) => {
                predictor.add_ball_position(ball_position);
                visualizer.set_ball_position(ball_position);
            }
            None => predictor.add_missing_ball_position(),
        }

        if let Some(y) = predictor.predict_y() {
            let mapped = lm.map(y, Y_TABLE_SIZE - 0.18, 0.18);
            lm.set_position(mapped, false);
        }

        if let Some(z) = predictor.predict_z() {
            let y = predictor.predict_y().unwrap_or(0.0);
            arm.move_to(y, z * 1000.0 - 180.0, predictor.hit_target());
        } else {
            arm.reset_by_z(250.0);
        }

        visualizer.set_machine_position(lm.get_mapped_position(Y_TABLE_SIZE - 0.18, 0.18));
        visualizer.set_screen(vision.get_screen())?;
        visualizer.render(&predictor, timer.get_fps())?;
        lm.update();

        if visualizer.stopped() {
            break;
        }
    }
    Ok(())
}
