use std::ops::{Div, SubAssign};

pub fn newton_find_root<T>(
    initial_guess: T,
    iterations: u32,
    mut function: impl FnMut(T) -> T,
    mut derivative: impl FnMut(T) -> T,
) -> T
where
    T: Copy + SubAssign + Div<Output = T>,
{
    let iterations = iterations.max(1);

    let mut guess = initial_guess;
    for _ in 0..iterations {
        guess -= function(guess) / derivative(guess);
    }
    guess
}

pub fn newton_find_all_roots(
    start: f64,
    end: f64,
    iterations: u32,
    step_count: u32,
    mut function: impl FnMut(f64) -> f64,
    mut derivative: impl FnMut(f64) -> f64,
) -> Vec<f64> {
    let tolerance = 0.0001;

    let (start, end) = (start.min(end), start.max(end));
    let step_count = step_count.max(2);
    let iterations = iterations.max(1);

    let mut roots = Vec::<f64>::with_capacity(step_count as usize);
    for i in 0..step_count {
        let t = start + (end - start) * i as f64 / (step_count - 1) as f64;
        let root = newton_find_root(t, iterations, &mut function, &mut derivative);
        if root >= start && root <= end && function(root) <= tolerance {
            roots.push(root);
        }
    }
    roots.sort_floats();

    let mut reduced_roots = Vec::<f64>::with_capacity(roots.len());
    let mut current_root_accum = 0.0;
    let mut current_root_accum_count = 0;
    for i in 0..roots.len() {
        let root = roots[i];
        if i == 0 {
            current_root_accum = root;
            current_root_accum_count = 1;
        } else {
            let prev_root = roots[i - 1];
            if (root - prev_root).abs() < tolerance {
                current_root_accum += root;
                current_root_accum_count += 1;
            } else {
                reduced_roots.push(current_root_accum / current_root_accum_count as f64);
                current_root_accum = root;
                current_root_accum_count = 1;
            }
        }
    }

    if current_root_accum_count > 0 {
        reduced_roots.push(current_root_accum / current_root_accum_count as f64);
    }

    reduced_roots
}
