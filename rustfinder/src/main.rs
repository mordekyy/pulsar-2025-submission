mod config;
mod finder;
mod image;

use std::collections::HashSet;
use std::fs;
use std::time::Instant;

use anyhow::Result;

use config::{FIELD_CONFIG, ROBOT_CONFIG};
use finder::{a_star, SearchStep};
use image::{draw_path, get_red_channel, noisy_square, normalize_to_ndarray, render_search_gif};

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut trace_enabled = false;
    let mut trace_interval: Option<usize> = None;

    for arg in args.iter().skip(1) {
        if arg == "--trace" {
            trace_enabled = true;
        } else if let Some(value) = arg.strip_prefix("--trace-step=") {
            if let Ok(parsed) = value.parse::<usize>() {
                if parsed > 0 {
                    trace_interval = Some(parsed);
                }
            }
        }
    }

    let field_config = &FIELD_CONFIG;
    let robot_config = &ROBOT_CONFIG;

    let pipeline_start = Instant::now();

    println!("Generating image with {}", field_config.image_size);
    fs::create_dir_all("output")?;
    let base_img = noisy_square(field_config.image_size, None, Some(field_config.blur_size))?;
    base_img.save("output/noisy.png")?;

    let red_img = get_red_channel(&base_img);
    red_img.save("output/red.png")?;

    let map_normalized = normalize_to_ndarray(&red_img)?;

    println!("Max slope is {} deg", robot_config.max_slope_deg);

    let map_meters = finder::convert_heightmap_to_meters(&map_normalized, field_config);
    let cost_map = finder::compute_gradient_cost_map(&map_meters, field_config, robot_config);

    let mut min_height = f32::INFINITY;
    let mut max_height = f32::NEG_INFINITY;
    for value in map_meters.iter() {
        if *value < min_height {
            min_height = *value;
        }
        if *value > max_height {
            max_height = *value;
        }
    }
    println!(
        "Height map range: {:.3} m - {:.3} m",
        min_height, max_height
    );

    let mut trace: Vec<SearchStep> = Vec::new();
    let mut blocked_seen: HashSet<(isize, isize)> = HashSet::new();

    let max_frames = 200usize;
    let sample_stride = trace_interval
        .unwrap_or(robot_config.trace_sample_stride)
        .max(1);

    let destination = (
        (field_config.image_size - 1) as usize,
        (field_config.image_size - 1) as usize,
    );

    let search_start = Instant::now();
    let (path, blocked) = if trace_enabled {
        let mut callback = |mut snapshot: SearchStep| {
            if snapshot.step_index % sample_stride != 0 && !snapshot.is_goal {
                return;
            }

            snapshot.blocked.retain(|cell| blocked_seen.insert(*cell));
            if snapshot.step_index % (sample_stride * 10) == 0 && snapshot.step_index > 0 {
                println!(
                    "{} steps processed (sampling every {})",
                    snapshot.step_index, sample_stride
                );
            }
            trace.push(snapshot);
        };

        a_star(
            &map_meters,
            robot_config.start_pos,
            destination,
            robot_config.movement_mode,
            Some(&cost_map),
            field_config,
            robot_config,
            Some(&mut callback),
        )
    } else {
        a_star(
            &map_meters,
            robot_config.start_pos,
            destination,
            robot_config.movement_mode,
            Some(&cost_map),
            field_config,
            robot_config,
            None,
        )
    };
    let search_duration = search_start.elapsed();

    match path.as_ref() {
        Some(path_cells) => println!("Path length: {} nodes", path_cells.len()),
        None => println!("Path not found"),
    }
    println!("Blocked samples recorded: {}", blocked.len());
    if !trace_enabled {
        println!("Hint: pass --trace to capture full search playback (slower).");
    }

    if let Some(path_cells) = path.as_ref() {
        let path_image = draw_path(&red_img, path_cells, &blocked);
        path_image.save("output/path.png")?;
        if trace_enabled {
            render_search_gif(
                &red_img,
                &trace,
                Some(path_cells.as_slice()),
                "output/search.gif",
                max_frames,
                50,
                700,
            )?;
        }
    } else {
        println!("Path not found.");
    }

    let pipeline_duration = pipeline_start.elapsed();
    println!("Search time: {:.3}s", search_duration.as_secs_f32());
    println!("Total pipeline time: {:.3}s", pipeline_duration.as_secs_f32());

    Ok(())
}
