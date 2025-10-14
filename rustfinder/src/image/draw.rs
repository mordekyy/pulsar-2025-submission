use image::{DynamicImage, Rgb, RgbImage};

pub fn draw_path(
    img: &DynamicImage,
    path: &[(usize, usize)],
    blocked: &[(isize, isize)],
) -> DynamicImage {
    let mut canvas: RgbImage = match img.color() {
        image::ColorType::Rgb8 | image::ColorType::Rgba8 => img.to_rgb8(),
        _ => img.to_rgb8(),
    };

    let (width, height) = canvas.dimensions();

    for &(row, col) in blocked {
        if row < 0 || col < 0 {
            continue;
        }
        let (r, c) = (row as u32, col as u32);
        if r >= height || c >= width {
            continue;
        }
        canvas.put_pixel(c, r, Rgb([0, 0, 0]));
    }

    for &(row, col) in path {
        let (r, c) = (row as u32, col as u32);
        if r >= height || c >= width {
            continue;
        }
        canvas.put_pixel(c, r, Rgb([255, 0, 0]));
    }

    if let Some(&(row, col)) = path.first() {
        let (r, c) = (row as u32, col as u32);
        if r < height && c < width {
            canvas.put_pixel(c, r, Rgb([0, 255, 0]));
        }
    }

    if let Some(&(row, col)) = path.last() {
        let (r, c) = (row as u32, col as u32);
        if r < height && c < width {
            canvas.put_pixel(c, r, Rgb([128, 0, 128]));
        }
    }

    DynamicImage::ImageRgb8(canvas)
}
