pub const CAPTURE_WIDTH: i32 = 1280;
pub const CAPTURE_HEIGHT: i32 = 800;
pub const CANNY_THRESHOLD: i32 = 400;
pub const MIN_AREA: f64 = 15.0;
pub const MAX_AREA: f64 = 500.0;
pub const MIN_COLOR_SIMILARITY: f64 = 130.0;
pub const CIRCULARITY_THRESHOLD: f64 = 0.3;
pub const CONVEXITY_THRESHOLD: f64 = 0.8;
pub const AREA_THRESHOLD: f64 = 0.7;

pub const X_TABLE_SIZE: f64 = 2.74;
pub const Y_TABLE_SIZE: f64 = 1.525;

/// Six world-frame calibration landmarks (metres).
pub const OBJECT_POINTS: [[f64; 3]; 6] = [
    [0.0, Y_TABLE_SIZE, 0.0],
    [X_TABLE_SIZE, Y_TABLE_SIZE, 0.0],
    [X_TABLE_SIZE, 0.0, 0.0],
    [0.0, 0.0, 0.0],
    [X_TABLE_SIZE / 2.0, -0.1525, 0.15],
    [X_TABLE_SIZE / 2.0, Y_TABLE_SIZE + 0.1525, 0.15],
];

#[cfg(target_os = "windows")]
pub const PORT_NAME: &str = "COM8";
#[cfg(target_os = "linux")]
pub const PORT_NAME: &str = "/dev/ttyUSB0";
#[cfg(target_os = "macos")]
pub const PORT_NAME: &str = "/dev/tty.usbserial-FT0G1Q2B";
#[cfg(not(any(target_os = "windows", target_os = "linux", target_os = "macos")))]
pub const PORT_NAME: &str = "/dev/ttyUSB0";

pub const SHOULDER_ID: u8 = 1;
pub const SHOULDER_PRIME_ID: u8 = 2;
pub const SHOULDER_YAW_ID: u8 = 3;
pub const ELBOW_ID: u8 = 4;
pub const WRIST_ID: u8 = 5;

pub const ORANGE_HSV: [f64; 3] = [20.0, 200.0, 200.0];
pub const ORANGE_HSV_LOWER: [u8; 3] = [12, 140, 160];
pub const ORANGE_HSV_UPPER: [u8; 3] = [20, 255, 255];
