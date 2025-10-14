from typing import Mapping, Sequence

from PIL import Image, ImageDraw

from image.draw import draw_path


def in_bounds(r: int, c: int, height: int, width: int) -> bool:
    return 0 <= r < height and 0 <= c < width


TraceSnapshot = Mapping[str, object]


def render_search_gif(
    base_img: Image.Image,
    trace: Sequence[TraceSnapshot],
    final_path: Sequence[tuple[int, int]] | None,
    output_path: str,
    *,
    max_frames: int = 200,
    frame_duration_ms: int = 50,
    final_frame_duration_ms: int = 700,
) -> None:
    if not trace:
        return

    canvas = base_img.convert("RGB") if base_img.mode not in {
        "RGB", "RGBA"} else base_img.copy()
    width, height = canvas.size

    stride = max(1, len(trace) // max_frames) if len(trace) > max_frames else 1
    frames: list[Image.Image] = []
    durations: list[int] = []

    blocked_seen: set[tuple[int, int]] = set()

    for index in range(0, len(trace), stride):
        snapshot = trace[index]
        frame = canvas.copy()
        draw = ImageDraw.Draw(frame)

        for (r, c) in snapshot.get("blocked", ()):
            if not in_bounds(r, c, height, width):
                continue
            blocked_seen.add((r, c))
            draw.point((c, r), fill="black")

        for (r, c) in snapshot.get("visited", ()):
            if not in_bounds(r, c, height, width):
                continue
            draw.point((c, r), fill="blue")

        for (r, c) in snapshot.get("open", ()):
            if not in_bounds(r, c, height, width):
                continue
            draw.point((c, r), fill="yellow")

        current = snapshot.get("current")
        if current:
            r, c = current
            if in_bounds(r, c, height, width):
                draw.point((c, r), fill="lime")

        frames.append(frame)
        durations.append(frame_duration_ms)

    final_blocked = []
    for cell in blocked_seen:
        if in_bounds(cell[0], cell[1], height, width):
            final_blocked.append(cell)

    final_frame = draw_path(canvas, list(final_path or []), final_blocked)
    frames.append(final_frame)
    durations.append(final_frame_duration_ms)

    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        format="GIF",
        duration=durations,
        loop=0,
    )
