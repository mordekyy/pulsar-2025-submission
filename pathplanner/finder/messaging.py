from typing import Callable, Iterable, NamedTuple, Optional, Sequence, Tuple


class SearchStep(NamedTuple):
    step_index: int
    current: tuple[int, int]
    visited: tuple[tuple[int, int], ...]
    open_nodes: tuple[tuple[int, int], ...]
    blocked: tuple[tuple[int, int], ...]


SearchStepCallback = Optional[Callable[[SearchStep], None]]


def normalize_cells(cells: Iterable[tuple[int, int]]) -> tuple[tuple[int, int], ...]:
    return tuple(sorted(set(cells)))


def collect_open_nodes(
    openq: Sequence[tuple[float, tuple[int, int]]],
    visited: Iterable[tuple[int, int]],
) -> tuple[tuple[int, int], ...]:
    visited_set = set(visited)
    return tuple(node for _, node in openq if node not in visited_set)


def emit_step(
    on_step: SearchStepCallback,
    step_index: int,
    current: tuple[int, int],
    visited: Iterable[tuple[int, int]],
    openq: Sequence[tuple[float, tuple[int, int]]],
    blocked: Iterable[tuple[int, int]],
) -> None:
    if not on_step:
        return

    visited_tuple = normalize_cells(visited)
    open_nodes = collect_open_nodes(openq, visited_tuple)
    blocked_tuple: Tuple[tuple[int, int], ...] = tuple(sorted(blocked))

    on_step(
        SearchStep(
            step_index=step_index,
            current=current,
            visited=visited_tuple,
            open_nodes=open_nodes,
            blocked=blocked_tuple,
        )
    )
