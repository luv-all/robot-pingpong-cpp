use crate::predictor::Vec3;
use crate::vision::tracker::Tracker;
use crate::vision::visualizer::Visualizer;
use anyhow::Result;
use opencv::core::Mat;

pub struct Vision {
    screen: Mat,
    tracker: Tracker,
}

impl Vision {
    pub fn new() -> Result<Self> {
        let mut screen = Mat::default();
        let tracker = Tracker::new(&mut screen)?;
        Ok(Self { screen, tracker })
    }

    pub fn init(&mut self, visualizer: &mut Visualizer, skip: bool) -> Result<()> {
        self.tracker.set_mask(skip)?;
        self.tracker.set_table_area(visualizer, skip)?;
        Ok(())
    }

    pub fn track(&mut self) -> Result<Option<Vec3>> {
        if self.tracker.render(&mut self.screen)? {
            Ok(Some(self.tracker.pos))
        } else {
            Ok(None)
        }
    }

    pub fn get_screen(&self) -> &Mat {
        &self.screen
    }
}
