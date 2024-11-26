use anyhow::{anyhow, Result};
use cgmath::{Angle, Deg, InnerSpace, Rad, Vector3, Zero};
use derive_more::*;
use krpc_client::services::space_center::{CelestialBody, Node, Orbit, ReferenceFrame, Vessel};
use log::debug;

use crate::{
    reference_frame_or_vessel_frame,
    util::{
        flatten_vector, get_apoapsis_speed, get_circular_orbit_speed,
        get_direction_of_true_anomaly, get_orbit_normal, get_orbital_period, get_periapsis_speed,
        get_prograde_normal_radial_directions_at, get_relative_ascending_node_ut,
        get_relative_descending_node_ut, get_true_anomaly_of_direction, orbital_velocity_at,
        ut_at_true_anomaly_offset,
    },
    FlightComputer,
};

pub async fn node_change_velocity(
    vessel: &Vessel,
    ut: f64,
    velocity: Vector3<f64>,
    reference_frame: Option<&ReferenceFrame>,
) -> Result<Node> {
    let control = vessel.get_control().await?;
    let orbit = vessel.get_orbit().await?;

    let reference_frame = reference_frame_or_vessel_frame!(vessel, reference_frame);

    let (prograde_direction, normal_direction, radial_direction) =
        get_prograde_normal_radial_directions_at(&orbit, ut, reference_frame).await?;

    let velocity_at_node = orbital_velocity_at(&orbit, ut, reference_frame).await?;
    let velocity_change = velocity - velocity_at_node;

    Ok(control
        .add_node(
            ut,
            velocity_change.dot(prograde_direction) as f32,
            velocity_change.dot(normal_direction) as f32,
            velocity_change.dot(radial_direction) as f32,
        )
        .await?)
}

pub async fn node_change_speed(vessel: &Vessel, ut: f64, speed: f64) -> Result<Node> {
    let control = vessel.get_control().await?;
    let orbit = vessel.get_orbit().await?;

    Ok(control
        .add_node(
            ut,
            (speed
                - orbital_velocity_at(&orbit, ut, &vessel.get_reference_frame().await?)
                    .await?
                    .magnitude()) as f32,
            0.0,
            0.0,
        )
        .await?)
}

pub async fn node_circularize(vessel: &Vessel, ut: f64) -> Result<Node> {
    let orbit = vessel.get_orbit().await?;
    let body = orbit.get_body().await?;

    let body_reference_frame = body.get_reference_frame().await?;

    let position_at: Vector3<f64> = orbit.position_at(ut, &body_reference_frame).await?.into();
    let radius_at = position_at.magnitude();
    let orbit_normal = get_orbit_normal(&orbit, &body_reference_frame).await?;

    let desired_speed = get_circular_orbit_speed(&body, radius_at).await?;

    node_change_velocity(
        vessel,
        ut,
        position_at.cross(orbit_normal).normalize_to(desired_speed),
        Some(&body_reference_frame),
    )
    .await
}

pub async fn node_change_orbit_normal(
    vessel: &Vessel,
    target_orbit_normal: Vector3<f64>,
    reference_frame: Option<&ReferenceFrame>,
    save_fuel: bool,
) -> Result<Node> {
    let orbit = vessel.get_orbit().await?;
    let body = orbit.get_body().await?;
    let body_reference_frame = body.get_non_rotating_reference_frame().await?;

    let reference_frame = reference_frame_or_vessel_frame!(vessel, reference_frame);
    let ascending_node_ut =
        get_relative_ascending_node_ut(&orbit, target_orbit_normal, reference_frame).await?;
    let descending_node_ut =
        get_relative_descending_node_ut(&orbit, target_orbit_normal, reference_frame).await?;

    let ut = if save_fuel {
        let ascending_position: Vector3<f64> = orbit
            .position_at(ascending_node_ut, &body_reference_frame)
            .await?
            .into();
        let descending_position: Vector3<f64> = orbit
            .position_at(descending_node_ut, &body_reference_frame)
            .await?
            .into();

        if ascending_position.magnitude2() > descending_position.magnitude2() {
            ascending_node_ut
        } else {
            descending_node_ut
        }
    } else {
        ascending_node_ut.min(descending_node_ut)
    };

    let old_velocity = orbital_velocity_at(&orbit, ut, reference_frame).await?;
    node_change_velocity(
        vessel,
        ut,
        flatten_vector(old_velocity, target_orbit_normal).normalize_to(old_velocity.magnitude()),
        Some(reference_frame),
    )
    .await
}

pub async fn node_change_opposite_radius(
    vessel: &Vessel,
    target_radius: f64,
    ut: f64,
) -> Result<Node> {
    let orbit = vessel.get_orbit().await?;
    let body = orbit.get_body().await?;
    let reference_frame = body.get_non_rotating_reference_frame().await?;

    let position: Vector3<f64> = orbit.position_at(ut, &reference_frame).await?.into();
    let radius = position.magnitude();

    let target_speed = if target_radius > radius {
        get_periapsis_speed(&body, radius, target_radius).await?
    } else {
        get_apoapsis_speed(&body, radius, target_radius).await?
    };

    node_change_speed(vessel, ut, target_speed).await
}

pub async fn node_hohmann_transfer(
    vessel: &Vessel,
    target_orbit: &Orbit,
    offset: f64,
) -> Result<Node> {
    let orbit = vessel.get_orbit().await?;
    let body = orbit.get_body().await?;
    let reference_frame = body.get_non_rotating_reference_frame().await?;

    let mut guess = Rad::from(Deg(5.0));
    let mut prev_guess: Option<Rad<f64>> = None;
    let mut prev_guess_error: Option<Rad<f64>> = None;

    let mut best_guess = Rad::from(Deg(5.0));
    let mut best_guess_error = Rad::full_turn();

    let iterations = 100;
    for _ in 0..iterations {
        let guess_ut = ut_at_true_anomaly_offset(&orbit, guess).await?;
        let guess_position: Vector3<f64> =
            orbit.position_at(guess_ut, &reference_frame).await?.into();

        let guess_arrival_target_true_anomaly =
            get_true_anomaly_of_direction(target_orbit, -guess_position, &reference_frame).await?;
        let guess_arrival_target_ut = target_orbit
            .ut_at_true_anomaly(guess_arrival_target_true_anomaly.0)
            .await?;
        let guess_arrival_position: Vector3<f64> = target_orbit
            .position_at(guess_arrival_target_ut, &reference_frame)
            .await?
            .into();

        let guess_arrival_ut = guess_ut
            + get_orbital_period(
                &body,
                guess_position.magnitude(),
                guess_arrival_position.magnitude(),
            )
            .await?
                / 2.0;
        let guess_arrival_target_velocity =
            orbital_velocity_at(target_orbit, guess_arrival_ut, &reference_frame).await?;
        let guess_arrival_target_position: Vector3<f64> = target_orbit
            .position_at(guess_arrival_ut, &reference_frame)
            .await?
            .into();

        let guess_error = guess_arrival_position.angle(
            guess_arrival_target_position + guess_arrival_target_velocity.normalize_to(offset),
        );

        // println!("err={:?}", Deg::from(guess_error));
        // println!("guess={:?}", Deg::from(guess));

        let next_guess =
            if let Some((prev_guess, prev_guess_error)) = prev_guess.zip(prev_guess_error) {
                let derivative = ((guess_error - prev_guess_error) / (guess - prev_guess))
                    .clamp(-1000.0, 1000.0);
                let derivative = 1.0 / (1.0 / derivative).clamp(-10.0, 10.0);

                let estimate = guess - guess_error / derivative;

                // println!("d/dg={:?}", derivative);

                if estimate >= Rad::from(Deg(5.0)) {
                    estimate
                } else {
                    let derivative = ((prev_guess_error - guess_error) / (guess - prev_guess))
                        .clamp(-1000.0, 1000.0);
                    let derivative = 1.0 / (1.0 / derivative).clamp(-10.0, 10.0);

                    guess - (Rad::full_turn() - guess_error) / derivative
                }
            } else {
                guess + Rad::from(Deg(15.0))
            };

        // println!();

        if best_guess_error >= guess_error {
            best_guess = guess;
            best_guess_error = guess_error;
        }

        prev_guess = Some(guess);
        prev_guess_error = Some(guess_error);

        guess = next_guess;
    }

    let best_guess_ut = ut_at_true_anomaly_offset(&orbit, best_guess).await?;
    let best_guess_position: Vector3<f64> = orbit
        .position_at(best_guess_ut, &reference_frame)
        .await?
        .into();

    let best_guess_arrival_target_true_anomaly =
        get_true_anomaly_of_direction(target_orbit, -best_guess_position, &reference_frame).await?;
    let best_guess_arrival_target_ut = target_orbit
        .ut_at_true_anomaly(best_guess_arrival_target_true_anomaly.0)
        .await?;
    let best_guess_arrival_position: Vector3<f64> = target_orbit
        .position_at(best_guess_arrival_target_ut, &reference_frame)
        .await?
        .into();

    debug!(
        "hohmann transfer:\n\tbest guess = {:?}\n\terr = {:?}",
        Deg::from(best_guess),
        Deg::from(best_guess_error)
    );

    node_change_opposite_radius(
        vessel,
        best_guess_arrival_position.magnitude(),
        best_guess_ut,
    )
    .await
}

#[derive(From)]
pub enum Orbiter<'a> {
    Vessel(&'a Vessel),
    Body(&'a CelestialBody),
}

impl<'a> Orbiter<'a> {
    pub async fn get_orbit(&self) -> Result<Orbit> {
        Ok(match self {
            Orbiter::Vessel(v) => v.get_orbit().await?,
            Orbiter::Body(b) => b
                .get_orbit()
                .await?
                .ok_or(anyhow!("no orbit (it's the sun lol)"))?,
        })
    }
}

impl FlightComputer {
    pub async fn node_circularize_at_apsis(&self, periapsis: bool) -> Result<Node> {
        let orbit = self.vessel.get_orbit().await?;
        let current_ut = self.space_center.get_ut().await?;
        let apsis_ut = current_ut
            + if periapsis {
                orbit.get_time_to_periapsis().await?
            } else {
                orbit.get_time_to_apoapsis().await?
            };

        node_circularize(&self.vessel, apsis_ut).await
    }
}
