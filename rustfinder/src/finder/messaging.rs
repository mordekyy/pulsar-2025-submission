use std::collections::HashSet;

#[derive(Debug, Clone)]
pub struct SearchStep {
    pub step_index: usize,
    pub current: (usize, usize),
    pub visited: Vec<(usize, usize)>,
    pub open_nodes: Vec<(usize, usize)>,
    pub blocked: Vec<(isize, isize)>,
    pub is_goal: bool,
}

pub type SearchStepCallback<'a> = Option<&'a mut dyn FnMut(SearchStep)>;

pub fn emit_step(
    callback: &mut SearchStepCallback,
    step_index: usize,
    current: (usize, usize),
    visited: &[(usize, usize)],
    open_nodes: &[(usize, usize)],
    blocked: &[(isize, isize)],
    is_goal: bool,
) {
    let Some(handler) = callback else {
        return;
    };

    let visited_vec = visited.to_vec();
    let mut visited_set: HashSet<(usize, usize)> = HashSet::with_capacity(visited_vec.len());
    for cell in &visited_vec {
        visited_set.insert(*cell);
    }

    let open_filtered: Vec<(usize, usize)> = open_nodes
        .iter()
        .copied()
        .filter(|node| !visited_set.contains(node))
        .collect();

    let mut blocked_vec: Vec<(isize, isize)> = blocked.to_vec();
    blocked_vec.sort_unstable();
    blocked_vec.dedup();

    handler(SearchStep {
        step_index,
        current,
        visited: visited_vec,
        open_nodes: open_filtered,
        blocked: blocked_vec,
        is_goal,
    });
}
