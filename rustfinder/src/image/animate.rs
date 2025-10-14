use std::collections::HashSet;
use std::fs::File;
use std::io::BufWriter;

use anyhow::Result;
use image::codecs::gif::{GifEncoder, Repeat};
use image::{Delay, DynamicImage, Frame, Rgb, RgbImage, RgbaImage};

use crate::finder::messaging::SearchStep;

fn make_frame(image: RgbaImage, delay_ms: u16) -> Frame {
    let delay = Delay::from_numer_denom_ms(delay_ms as u32, 1);
    Frame::from_parts(image, 0, 0, delay)
}

fn in_bounds(row: usize, col: usize, height: usize, width: usize) -> bool {
    row < height && col < width
}

pub fn render_search_gif(
    base_img: &DynamicImage,
    trace: &[SearchStep],
    final_path: Option<&[(usize, usize)]>,
    output_path: &str,
    max_frames: usize,
    frame_duration_ms: u16,
    final_frame_duration_ms: u16,
) -> Result<()> {
    if trace.is_empty() {
        return Ok(());
    }

    let canvas: RgbImage = base_img.to_rgb8();
    let (width, height) = canvas.dimensions();
    let width_usize = width as usize;
    let height_usize = height as usize;

    let stride = if max_frames > 0 && trace.len() > max_frames {
        let step = trace.len() / max_frames;
        if step == 0 { 1 } else { step }
    } else {
        1
    };

    let mut frames: Vec<Frame> = Vec::new();
    let mut blocked_seen: HashSet<(usize, usize)> = HashSet::new();

    for index in (0..trace.len()).step_by(stride) {
        let snapshot = &trace[index];
        let mut frame_img = canvas.clone();

        for &(row, col) in &snapshot.blocked {
            if row < 0 || col < 0 {
                continue;
            }
            let (r, c) = (row as usize, col as usize);
            if !in_bounds(r, c, height_usize, width_usize) {
                continue;
            }
            blocked_seen.insert((r, c));
            frame_img.put_pixel(c as u32, r as u32, Rgb([0, 0, 0]));
        }

        for &(row, col) in &snapshot.visited {
            if !in_bounds(row, col, height_usize, width_usize) {
                continue;
            }
            frame_img.put_pixel(col as u32, row as u32, Rgb([0, 0, 255]));
        }

        for &(row, col) in &snapshot.open_nodes {
            if !in_bounds(row, col, height_usize, width_usize) {
                continue;
            }
            frame_img.put_pixel(col as u32, row as u32, Rgb([255, 255, 0]));
        }

        let (row, col) = snapshot.current;
        if in_bounds(row, col, height_usize, width_usize) {
            frame_img.put_pixel(col as u32, row as u32, Rgb([50, 205, 50]));
        }

        frames.push(make_frame(
            DynamicImage::ImageRgb8(frame_img).into_rgba8(),
            frame_duration_ms,
        ));
    }

    let final_blocked: Vec<(isize, isize)> = blocked_seen
        .into_iter()
        .map(|(r, c)| (r as isize, c as isize))
        .collect();

    let base_dynamic = DynamicImage::ImageRgb8(canvas.clone());
    let final_image = crate::image::draw::draw_path(
        &base_dynamic,
        final_path.unwrap_or(&[]),
        &final_blocked,
    );
    frames.push(make_frame(
        final_image.into_rgba8(),
        final_frame_duration_ms,
    ));

    let file = File::create(output_path)?;
    let writer = BufWriter::new(file);
    let mut encoder = GifEncoder::new(writer);
    encoder.set_repeat(Repeat::Infinite)?;
    for frame in frames {
        encoder.encode_frame(frame)?;
    }

    Ok(())
}
