use std::collections::BTreeSet;

#[derive(Debug, Clone)]
pub struct SearchStep {
    pub step_index: usize,
    pub current: (usize, usize),
    pub visited: Vec<(usize, usize)>,
    pub open_nodes: Vec<(usize, usize)>,
    pub blocked: Vec<(isize, isize)>,
}

pub type SearchStepCallback<'a> = Option<&'a mut dyn FnMut(SearchStep)>;

fn normalize_cells<I>(cells: I) -> Vec<(usize, usize)>
where
    I: IntoIterator<Item = (usize, usize)>,
{
    let mut set: BTreeSet<(usize, usize)> = BTreeSet::new();
    for cell in cells {
        set.insert(cell);
    }
    set.into_iter().collect()
}

fn collect_open_nodes(
    open_nodes: &[(usize, usize)],
    visited: &[(usize, usize)],
) -> Vec<(usize, usize)> {
    let visited_set: BTreeSet<(usize, usize)> = visited.iter().copied().collect();
    open_nodes
        .iter()
        .copied()
        .filter(|node| !visited_set.contains(node))
        .collect()
}

pub fn emit_step(
    callback: &mut SearchStepCallback,
    step_index: usize,
    current: (usize, usize),
    visited: impl IntoIterator<Item = (usize, usize)>,
    open_nodes: &[(usize, usize)],
    blocked: &[(isize, isize)],
) {
    let Some(handler) = callback else {
        return;
    };

    let visited_vec = normalize_cells(visited);
    let open_filtered = collect_open_nodes(open_nodes, &visited_vec);
    let mut blocked_vec: Vec<(isize, isize)> = blocked.to_vec();
    blocked_vec.sort();

    handler(SearchStep {
        step_index,
        current,
        visited: visited_vec,
        open_nodes: open_filtered,
        blocked: blocked_vec,
    });
}
