use cgmath::{vec2, vec3, Angle, Deg, InnerSpace, Quaternion, Rad, Vector3};
use derive_more::*;

use crate::{
    orbital_mechanics::{LocalBody, LocalNode, LocalOrbit},
    util::get_relative_ascending_node_direction,
};

pub fn node_change_velocity(orbit: &LocalOrbit, ut: f64, velocity: Vector3<f64>) -> LocalNode {
    LocalNode {
        ut,
        burn_vector: orbit
            .absolute_to_prograde_normal_radial(ut, velocity - orbit.velocity_at_ut(ut)),
    }
}

pub fn node_change_speed(orbit: &LocalOrbit, ut: f64, speed: f64) -> LocalNode {
    LocalNode {
        ut,
        burn_vector: vec3(speed - orbit.velocity_at_ut(ut).magnitude(), 0.0, 0.0),
    }
}

pub fn node_circularize(orbit: &LocalOrbit, ut: f64) -> LocalNode {
    let position = orbit.position_at_ut(ut);
    let circular_speed = orbit.body.circular_orbit_speed(position.magnitude());
    let velocity = position.cross(orbit.normal()).normalize_to(circular_speed);

    node_change_velocity(orbit, ut, velocity)
}

pub fn node_circularize_at_apsis(
    orbit: &LocalOrbit,
    is_periapsis: bool,
    current_ut: f64,
) -> LocalNode {
    node_circularize(
        orbit,
        if is_periapsis {
            orbit.next_periapsis_ut(current_ut)
        } else {
            orbit.next_apoapsis_ut(current_ut)
        },
    )
}

pub fn node_change_orbit_normal(
    orbit: &LocalOrbit,
    target_orbit_normal: Vector3<f64>,
    save_fuel: bool,
    current_ut: f64,
) -> LocalNode {
    let periapsis_ut = orbit.most_recent_periapsis_ut(current_ut);
    let ascending_direction =
        get_relative_ascending_node_direction(orbit.normal(), target_orbit_normal);
    let ascending_true_anomaly = orbit.true_anomaly_of_position(ascending_direction);
    let descending_true_anomaly =
        (ascending_true_anomaly + Rad::full_turn() / 2.0).normalize_signed();

    let ascending_ut = periapsis_ut + orbit.true_anomaly_time_delay(ascending_true_anomaly);
    let ascending_ut = if ascending_ut > current_ut {
        ascending_ut
    } else {
        ascending_ut + orbit.period()
    };
    let descending_ut = periapsis_ut + orbit.true_anomaly_time_delay(descending_true_anomaly);
    let descending_ut = if descending_ut > current_ut {
        descending_ut
    } else {
        descending_ut + orbit.period()
    };

    let ut = if save_fuel {
        if orbit.position_at_ut(ascending_ut).magnitude()
            > orbit.position_at_ut(descending_ut).magnitude()
        {
            ascending_ut
        } else {
            descending_ut
        }
    } else {
        ascending_ut.min(descending_ut)
    };

    let rotation = Quaternion::from_arc(orbit.normal(), target_orbit_normal, None);
    node_change_velocity(orbit, ut, rotation * orbit.velocity_at_ut(ut))
}

pub fn node_change_opposite_radius(orbit: &LocalOrbit, target_radius: f64, ut: f64) -> LocalNode {
    let position = orbit.position_at_ut(ut);
    let radius = position.magnitude();
    let target_sma = (radius + target_radius) / 2.0;

    node_change_speed(
        orbit,
        ut,
        orbit
            .body
            .orbital_speed_from_sma_and_radius(target_sma, radius),
    )
}

pub fn node_hohmann_transfer(
    orbit: &LocalOrbit,
    target_orbit: &LocalOrbit,
    offset: f64,
    current_ut: f64,
) -> LocalNode {
    let current_true_anomaly = orbit.true_anomaly_at_ut(current_ut);

    let get_guess_error = |guess: Rad<f64>| {
        let guess_true_anomaly_offset = current_true_anomaly.normalize() + guess;
        let guess_ut = orbit.most_recent_periapsis_ut(current_ut)
            + orbit.true_anomaly_time_delay(guess_true_anomaly_offset);
        let guess_position = orbit.position_at_ut(guess_ut);

        let target_ideal_true_anomaly = target_orbit.true_anomaly_of_position(-guess_position);
        let target_ideal_ut = target_orbit.most_recent_periapsis_ut(current_ut)
            + target_orbit.true_anomaly_time_delay(target_ideal_true_anomaly);
        let target_ideal_intercept_position = target_orbit.position_at_ut(target_ideal_ut);

        let intercept_ut = guess_ut
            + orbit.body.orbital_period_of_sma(
                (guess_position.magnitude() + target_ideal_intercept_position.magnitude()) / 2.0,
            ) / 2.0;
        let target_actual_intercept_position = target_orbit.position_at_ut(intercept_ut)
            + target_orbit
                .velocity_at_ut(intercept_ut)
                .normalize_to(offset);

        (target_ideal_true_anomaly
            - target_orbit.true_anomaly_of_position(target_actual_intercept_position))
        .normalize_signed()
    };

    let min_guess = Rad::from(Deg(5.0));
    let mut guess = min_guess;
    let mut best_guess = min_guess;
    let mut best_guess_error: Rad<f64> = Rad::full_turn();
    for _ in 0..30 {
        let diff = Rad::from(Deg(1.0));
        let guess_error = get_guess_error(guess);
        let guess_diff_error = get_guess_error(guess + diff);
        let derivative = (guess_diff_error - guess_error) / diff;

        let next_guess = guess - guess_error / derivative;
        guess = if next_guess >= min_guess {
            next_guess
        } else {
            guess - (Rad::full_turn() - guess_error) / -derivative
        };

        if guess_error.0.abs() < best_guess_error.0.abs() {
            best_guess = guess;
            best_guess_error = guess_error;
        }
    }

    let true_anomaly_offset = current_true_anomaly.normalize() + best_guess;
    let ut = orbit.most_recent_periapsis_ut(current_ut)
        + orbit.true_anomaly_time_delay(true_anomaly_offset);
    let position = orbit.position_at_ut(ut);

    let target_ideal_true_anomaly = target_orbit.true_anomaly_of_position(-position);
    let target_ideal_ut = target_orbit.most_recent_periapsis_ut(current_ut)
        + target_orbit.true_anomaly_time_delay(target_ideal_true_anomaly);
    let target_ideal_intercept_position = target_orbit.position_at_ut(target_ideal_ut);

    node_change_opposite_radius(orbit, target_ideal_intercept_position.magnitude(), ut)
}

pub fn node_tune_closest_approach(
    orbit: &LocalOrbit,
    target_orbit: &LocalOrbit,
    ut: f64,
    desired_distance: f64,
    search_radius: f64,
) -> LocalNode {
    let search_radius = search_radius.max(0.1);

    // this needs to be an odd number so that a speed change of 0 is always included in at least the initial search
    let increments = 15;

    let mut start = -search_radius;
    let mut end = search_radius;

    let mut closest_speed_change = 0.0;
    let mut closest_distance_error = f64::INFINITY;

    for _ in 0..10 {
        let step_size = (end - start) / (increments - 1) as f64;

        for i in 0..increments {
            let speed_change = start + i as f64 * step_size;
            let changed_orbit = orbit.body.create_orbit_from_state_vectors(
                ut,
                orbit.position_at_ut(ut),
                orbit
                    .velocity_at_ut(ut)
                    .normalize_to(orbit.velocity_at_ut(ut).magnitude() + speed_change),
            );
            let (_, approach_distance) =
                changed_orbit.find_closest_approach(target_orbit, ut, changed_orbit.period());
            let distance_error = (approach_distance - desired_distance).abs();

            if distance_error < closest_distance_error {
                closest_distance_error = distance_error;
                closest_speed_change = speed_change;
            }
        }

        start = closest_speed_change - step_size;
        end = closest_speed_change + step_size;
    }

    LocalNode {
        ut,
        burn_vector: vec3(closest_speed_change, 0.0, 0.0),
    }
}

pub fn node_tune_body_periapsis(
    orbit: &LocalOrbit,
    target_body: &LocalBody,
    ut: f64,
    desired_periapsis: f64,
    search_radius: f64,
) -> LocalNode {
    let search_radius = search_radius.max(0.1);

    let target_orbit = target_body.orbit.as_ref().unwrap();
    // this needs to be an odd number so that a speed change of 0 is always included in at least the initial search
    let increments = 11;

    let mut start = -search_radius;
    let mut end = search_radius;

    let mut closest_speed_change = 0.0;
    let mut closest_distance_error = f64::INFINITY;

    for _ in 0..7 {
        let step_size = (end - start) / (increments - 1) as f64;

        for i in 0..increments {
            let speed_change = start + i as f64 * step_size;

            let changed_orbit = orbit.body.create_orbit_from_state_vectors(
                ut,
                orbit.position_at_ut(ut),
                orbit
                    .velocity_at_ut(ut)
                    .normalize_to(orbit.velocity_at_ut(ut).magnitude() + speed_change),
            );

            if let Some(enter_ut) =
                target_body.find_soi_enter(&changed_orbit, ut, changed_orbit.period())
            {
                let position =
                    changed_orbit.position_at_ut(enter_ut) - target_orbit.position_at_ut(enter_ut);
                let velocity =
                    changed_orbit.velocity_at_ut(enter_ut) - target_orbit.velocity_at_ut(enter_ut);

                let enter_orbit =
                    target_body.create_orbit_from_state_vectors(enter_ut, position, velocity);

                let periapsis = enter_orbit.periapsis()
                    * enter_orbit.normal().dot(changed_orbit.normal()).signum();
                let distance_error = (periapsis - desired_periapsis).abs();

                if distance_error < closest_distance_error {
                    closest_distance_error = distance_error;
                    closest_speed_change = speed_change;
                }
            }
        }

        start = closest_speed_change - step_size;
        end = closest_speed_change + step_size;
    }

    LocalNode {
        ut,
        burn_vector: vec3(closest_speed_change, 0.0, 0.0),
    }
}

pub fn node_hohmann_transfer_to_body(
    orbit: &LocalOrbit,
    target_body: &LocalBody,
    desired_periapsis: f64,
    current_ut: f64,
) -> LocalNode {
    let target_orbit = target_body.orbit.as_ref().unwrap();
    let node = node_hohmann_transfer(orbit, target_orbit, 0.0, current_ut);
    let transfer_orbit = orbit.after_node(node);
    let tune_node = node_tune_body_periapsis(
        &transfer_orbit,
        target_body,
        node.ut,
        desired_periapsis,
        node.burn_vector.magnitude() / 2.0,
    );

    LocalNode {
        ut: node.ut,
        burn_vector: node.burn_vector + tune_node.burn_vector,
    }
}

pub fn node_return_to_parent_body(
    orbit: &LocalOrbit,
    desired_periapsis: f64,
    current_ut: f64,
) -> LocalNode {
    assert!(
        orbit.body.orbit.is_some(),
        "no parent body exists, you are probably orbiting the sun"
    );
    let child_orbit = orbit.body.orbit.as_ref().unwrap();
    let parent_body = &child_orbit.body;

    // step 1: find a node that's pretty close (but not precise)
    // i tried to make this the only step, but oh well

    let step_count = 360;
    let iterations = 5;

    let mut start = Rad(0.0);
    let mut end = Rad::full_turn();
    let mut exit_ut = current_ut + orbit.period() * 2.0;

    let mut best_difference = f64::INFINITY;
    let mut best_exit_orbit = orbit.clone();
    let mut best_exit_burn_ut = current_ut;
    let mut best_angle = Rad(0.0);

    for iteration in 0..iterations {
        let angle_step = (end - start) / step_count as f64;

        for step in 0..step_count {
            let angle = start + angle_step * step as f64;

            let exit_position = orbit.local_to_absolute_vector(
                vec2(angle.cos(), angle.sin()) * orbit.body.sphere_of_influence,
            );
            let exit_parent_position = child_orbit.position_at_ut(exit_ut) + exit_position;

            let exit_parent_velocity = exit_parent_position
                .cross(child_orbit.normal())
                .normalize_to(parent_body.orbital_speed_from_sma_and_radius(
                    (desired_periapsis + exit_parent_position.magnitude()) / 2.0,
                    exit_parent_position.magnitude(),
                ));

            let exit_velocity = exit_parent_velocity - child_orbit.velocity_at_ut(exit_ut);
            if exit_velocity.dot(exit_position).is_sign_negative() {
                continue;
            }

            let exit_orbit =
                orbit
                    .body
                    .create_orbit_from_state_vectors(exit_ut, exit_position, exit_velocity);
            let exit_periapsis_position = exit_orbit.periapsis_direction() * exit_orbit.periapsis();

            let exit_burn_ut = orbit.most_recent_periapsis_ut(current_ut)
                + orbit.true_anomaly_time_delay(
                    orbit.true_anomaly_of_position(exit_periapsis_position),
                );
            let exit_burn_ut = if exit_burn_ut < current_ut {
                exit_burn_ut + orbit.period()
            } else {
                exit_burn_ut
            };

            if exit_orbit
                .velocity_at_ut(exit_orbit.periapsis_epoch())
                .dot(orbit.velocity_at_ut(exit_burn_ut))
                .is_sign_negative()
            {
                continue;
            }

            let difference =
                (exit_periapsis_position - orbit.position_at_ut(exit_burn_ut)).magnitude();
            if difference < best_difference {
                best_difference = difference;
                best_exit_orbit = exit_orbit;
                best_exit_burn_ut = exit_burn_ut;
                best_angle = angle;
            }
        }

        let best_exit_orbit_time_difference =
            best_exit_orbit.most_recent_periapsis_ut(exit_ut) - best_exit_burn_ut;
        exit_ut -= best_exit_orbit_time_difference;

        // let the first iteration find a more accurate exit_ut before we shrink the area down
        if iteration >= 1 {
            start += (best_angle - start) / 2.0;
            end += (best_angle - end) / 2.0;
        }
    }

    // step 2: adjust burn until the periapsis is precise
    // not as elegant as i would have liked, but it works perfectly

    let new_speed = best_exit_orbit
        .velocity_at_ut(best_exit_orbit.periapsis_epoch())
        .magnitude();
    let current_speed_change = new_speed - orbit.velocity_at_ut(best_exit_burn_ut).magnitude();

    let search_radius = current_speed_change / 2.0;
    let burn_adjust_increments = 15;

    let mut start = current_speed_change - search_radius;
    let mut end = current_speed_change + search_radius;

    let mut best_speed_change = current_speed_change;
    let mut best_difference = f64::INFINITY;
    for _ in 0..10 {
        let step_size = (end - start) / (burn_adjust_increments - 1) as f64;

        for i in 0..burn_adjust_increments {
            let speed_change = start + i as f64 * step_size;
            let changed_orbit = orbit.after_node(LocalNode {
                ut: best_exit_burn_ut,
                burn_vector: vec3(speed_change, 0.0, 0.0),
            });

            if let Some(adjusted_exit_ut) = changed_orbit.find_soi_exit(best_exit_burn_ut) {
                let adjusted_parent_exit_orbit = parent_body.create_orbit_from_state_vectors(
                    adjusted_exit_ut,
                    child_orbit.position_at_ut(adjusted_exit_ut)
                        + changed_orbit.position_at_ut(adjusted_exit_ut),
                    child_orbit.velocity_at_ut(adjusted_exit_ut)
                        + changed_orbit.velocity_at_ut(adjusted_exit_ut),
                );
                let adjusted_periapsis = adjusted_parent_exit_orbit.periapsis();
                let difference = (adjusted_periapsis - desired_periapsis).abs();
                if difference < best_difference {
                    best_speed_change = speed_change;
                    best_difference = difference;
                }
            }
        }

        start = best_speed_change - step_size;
        end = best_speed_change + step_size;
    }

    LocalNode {
        ut: best_exit_burn_ut,
        burn_vector: vec3(best_speed_change, 0.0, 0.0),
    }
}
