use anyhow::Result;
use image::{DynamicImage, ImageBuffer, Rgb};
use rand::{rngs::StdRng, RngCore, SeedableRng};

pub fn noisy_square(width: u32, seed: Option<u64>, blur_radius: Option<f32>) -> Result<DynamicImage> {
    let mut data = vec![0u8; (width * width * 3) as usize];

    match seed {
        Some(seed_value) => {
            let mut rng = StdRng::seed_from_u64(seed_value);
            rng.fill_bytes(&mut data);
        }
        None => {
            let mut rng = rand::thread_rng();
            rng.fill_bytes(&mut data);
        }
    }

    let img: ImageBuffer<Rgb<u8>, Vec<u8>> =
        ImageBuffer::from_vec(width, width, data).expect("buffer dimensions must match");
    let dynamic = DynamicImage::ImageRgb8(img);

    let output = match blur_radius {
        Some(radius) if radius > 0.0 => dynamic.blur(radius),
        _ => dynamic,
    };

    Ok(output)
}
