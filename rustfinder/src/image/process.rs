use anyhow::Result;
use image::{DynamicImage, ImageBuffer, Luma};
use ndarray::Array2;

pub fn get_red_channel(img: &DynamicImage) -> DynamicImage {
    let rgb = img.to_rgb8();
    let (width, height) = rgb.dimensions();
    let mut red = ImageBuffer::<Luma<u8>, Vec<u8>>::new(width, height);

    for (x, y, pixel) in rgb.enumerate_pixels() {
        red.put_pixel(x, y, Luma([pixel[0]]));
    }

    DynamicImage::ImageLuma8(red)
}

pub fn normalize_to_ndarray(img: &DynamicImage) -> Result<Array2<f32>> {
    let gray = img.to_luma8();
    let (width, height) = gray.dimensions();
    let mut data = Vec::with_capacity((width * height) as usize);

    for y in 0..height {
        for x in 0..width {
            let value = gray.get_pixel(x, y)[0] as f32 / 255.0;
            data.push(value);
        }
    }

    Ok(Array2::from_shape_vec((height as usize, width as usize), data)?)
}
