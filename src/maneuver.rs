use anyhow::{anyhow, bail, Result};
use cgmath::{num_traits::Signed, Angle, Deg, InnerSpace, Quaternion, Rad, Vector3};
use derive_more::*;
use krpc_client::services::space_center::{Node, Vessel};
use log::debug;

use crate::{
    orbital_mechanics::{LocalBody, LocalOrbit, OrbitalStateChange, OrbitalTrajectory},
    util::{get_current_ut_from_orbit, get_relative_ascending_node_direction},
};

pub async fn node_change_velocity(
    vessel: &Vessel,
    ut: f64,
    velocity: Vector3<f64>,
) -> Result<Node> {
    let control = vessel.get_control().await?;
    let orbit = LocalOrbit::from_orbit(&vessel.get_orbit().await?).await?;
    let pnr = orbit.absolute_to_prograde_normal_radial(ut, velocity - orbit.velocity_at_ut(ut));

    Ok(control
        .add_node(ut, pnr.x as f32, pnr.y as f32, pnr.z as f32)
        .await?)
}

pub async fn node_change_speed(vessel: &Vessel, ut: f64, speed: f64) -> Result<Node> {
    let control = vessel.get_control().await?;
    let orbit = LocalOrbit::from_orbit(&vessel.get_orbit().await?).await?;

    Ok(control
        .add_node(
            ut,
            (speed - orbit.velocity_at_ut(ut).magnitude()) as f32,
            0.0,
            0.0,
        )
        .await?)
}

pub async fn node_circularize(vessel: &Vessel, ut: f64) -> Result<Node> {
    let orbit = LocalOrbit::from_orbit(&vessel.get_orbit().await?).await?;

    let position = orbit.position_at_ut(ut);
    let circular_speed = orbit.body.circular_orbit_speed(position.magnitude());
    let velocity = position.cross(orbit.normal()).normalize_to(circular_speed);

    node_change_velocity(vessel, ut, velocity).await
}

pub async fn node_circularize_at_apsis(vessel: &Vessel, is_periapsis: bool) -> Result<Node> {
    let non_local_orbit = vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
    let current_ut = get_current_ut_from_orbit(&non_local_orbit).await?;

    let apsis_ut = orbit.most_recent_periapsis_ut(current_ut)
        + if is_periapsis {
            0.0
        } else {
            orbit.period() / 2.0
        };
    let ut = if apsis_ut < current_ut {
        apsis_ut + orbit.period()
    } else {
        apsis_ut
    };

    node_circularize(vessel, ut).await
}

pub async fn node_change_orbit_normal(
    vessel: &Vessel,
    target_orbit_normal: Vector3<f64>,
    save_fuel: bool,
) -> Result<Node> {
    let non_local_orbit = vessel.get_orbit().await?;
    let current_ut = get_current_ut_from_orbit(&non_local_orbit).await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

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
    node_change_velocity(vessel, ut, rotation * orbit.velocity_at_ut(ut)).await
}

pub async fn node_change_opposite_radius(
    vessel: &Vessel,
    target_radius: f64,
    ut: f64,
) -> Result<Node> {
    let orbit = LocalOrbit::from_orbit(&vessel.get_orbit().await?).await?;

    let position = orbit.position_at_ut(ut);
    let radius = position.magnitude();
    let target_sma = (radius + target_radius) / 2.0;

    node_change_speed(
        vessel,
        ut,
        orbit
            .body
            .orbital_speed_from_sma_and_radius(target_sma, radius),
    )
    .await
}

pub async fn node_hohmann_transfer(
    vessel: &Vessel,
    target_orbit: &LocalOrbit,
    offset: f64,
) -> Result<Node> {
    let non_local_orbit = vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
    let current_ut = get_current_ut_from_orbit(&non_local_orbit).await?;
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
            guess + (Rad::full_turn() - guess_error) / derivative
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

    debug!(
        "hohmann transfer:\n\tbest guess = {:?}\n\terr = {:?}",
        Deg::from(best_guess),
        Deg::from(best_guess_error)
    );

    node_change_opposite_radius(vessel, target_ideal_intercept_position.magnitude(), ut).await
}

pub async fn node_hohmann_transfer_to_body(
    vessel: &Vessel,
    target_body: &LocalBody,
    periapsis: f64,
) -> Result<Node> {
    let non_local_orbit = vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
    let target_orbit = target_body
        .orbit
        .as_ref()
        .ok_or(anyhow!("target body has no orbit"))?;

    if orbit.body.name != target_orbit.body.name {
        bail!("parent bodies do not match")
    }

    let node = node_hohmann_transfer(vessel, target_orbit, 0.0).await?;
    let node_ut = node.get_ut().await?;
    let node_delta_v = node.get_delta_v().await?;
    let current_ut = get_current_ut_from_orbit(&non_local_orbit).await?;

    let mut trajectory = orbit.singleton_trajectory(current_ut);
    trajectory.add_node(&node).await?;

    let burn_direction = orbit
        .prograde_normal_radial_to_absolute(node_ut, node.burn_vector(None).await?.into())
        .normalize();
    let position = orbit.position_at_ut(node_ut);
    let velocity = orbit.velocity_at_ut(node_ut);

    let mut start = node_delta_v - 100.0;
    let mut end = node_delta_v + 100.0;
    for _ in 0..30 {
        let middle = (start + end) / 2.0;
        let transfer_orbit = orbit.body.create_orbit_from_state_vectors(
            node_ut,
            position,
            velocity + burn_direction * middle,
        );

        let middle_periapsis = if let Some(enter_ut) =
            target_body.find_soi_enter(&transfer_orbit, node_ut, transfer_orbit.period())
        {
            let enter_position =
                transfer_orbit.position_at_ut(enter_ut) - target_orbit.position_at_ut(enter_ut);
            let enter_velocity =
                transfer_orbit.velocity_at_ut(enter_ut) - target_orbit.velocity_at_ut(enter_ut);
            let enter_orbit = target_body.create_orbit_from_state_vectors(
                enter_ut,
                enter_position,
                enter_velocity,
            );

            if enter_orbit
                .normal()
                .dot(target_orbit.normal())
                .is_sign_positive()
            {
                enter_orbit.periapsis()
            } else {
                -enter_orbit.periapsis()
            }
        } else {
            let (approach_ut, approach_distance) = transfer_orbit.find_closest_approach(
                target_orbit,
                node_ut,
                transfer_orbit.period() / 2.0,
            );

            if transfer_orbit.position_at_ut(approach_ut).magnitude()
                > target_orbit.position_at_ut(approach_ut).magnitude()
            {
                -approach_distance
            } else {
                approach_distance
            }
        };

        println!("hdfgdfs {}", middle_periapsis);

        if middle_periapsis > periapsis {
            start = middle;
        } else {
            end = middle;
        }
    }

    node.set_delta_v((start + end) / 2.0).await?;

    Ok(node)
}
