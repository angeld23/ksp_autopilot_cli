use std::time::Duration;

use anyhow::Result;
use cgmath::{prelude::*, vec3, Deg, Quaternion, Rad, Vector3};
use clap::Args;
use log::debug;
use tokio::time;

use crate::{
    flight_computer::FlightComputer,
    landing::LandingDescriptor,
    orbital_mechanics::LocalOrbit,
    trajectory_prediction::{SurfaceTrajectory, TrajectoryPredictionConfig},
    util::{flatten_vector, surface_position_at_coordinates},
};

#[derive(Debug, Args)]
pub struct BsfpArgs {
    /// Whether to skip the boostback.
    #[clap(long, short = 'b', default_value_t = false)]
    no_boostback: bool,
    /// Whether to warp until close to landing.
    #[clap(long, short = 'w', default_value_t = false)]
    no_warp: bool,
}

pub async fn bsfp_command(computer: &FlightComputer, args: &BsfpArgs) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let body = non_local_orbit.get_body().await?;
    let body_reference_frame = body.get_reference_frame().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    let pad_latitude = Deg(-0.09720332);
    let pad_longitude = Deg(-74.55738155);
    let target_position =
        surface_position_at_coordinates(&body, pad_latitude, pad_longitude).await?;

    debug!("beginning BSFP landing sequence");

    let control = computer.vessel.get_control().await?;
    let auto_pilot = computer.vessel.get_auto_pilot().await?;
    let flight = computer.vessel.flight(Some(&body_reference_frame)).await?;

    if !args.no_boostback {
        control.toggle_action_group(1).await?;
        control.set_rcs(true).await?;
    }

    auto_pilot
        .set_reference_frame(&body_reference_frame)
        .await?;
    auto_pilot.set_roll_threshold(0.0).await?;
    auto_pilot.engage().await?;

    let mut within_margin = false;
    let enter_margin_threshold: Rad<f64> = Deg(1.0).into();
    let exit_margin_threshold: Rad<f64> = Deg(90.0).into();

    let mut boostback_complete = args.no_boostback;
    let mut warp_complete = args.no_warp;

    if !args.no_warp {
        computer.space_center.set_physics_warp_factor(3).await?;
    }

    let mut interval = time::interval(Duration::from_secs_f64(1.0 / 10.0));
    loop {
        interval.tick().await;
        let current_position: Vector3<f64> = computer
            .vessel
            .position(&body_reference_frame)
            .await?
            .into();
        let rotation: Quaternion<f64> = flight.get_rotation().await?.into();
        let forward = rotation * vec3(0.0, 1.0, 0.0);
        let velocity: Vector3<f64> = flight.get_velocity().await?.into();
        let surface_altitude = flight.get_surface_altitude().await?;
        let sea_level_altitude = flight.get_mean_altitude().await?;

        let trajectory = SurfaceTrajectory::predict(
            &computer.vessel,
            TrajectoryPredictionConfig {
                look_ahead: orbit.period(),
                time_step: 1.0,
                ..Default::default()
            },
        )
        .await?;

        if let Some(impact) = trajectory.impact {
            let direction_towards_target =
                flatten_vector(target_position - current_position, current_position).normalize();
            let overshoot_target_position = target_position + direction_towards_target * 500.0;

            let impact_error = overshoot_target_position - impact.position;
            let impact_adjust_direction =
                flatten_vector(impact_error, current_position).normalize();

            if !boostback_complete {
                auto_pilot
                    .set_target_direction(impact_adjust_direction.into())
                    .await?;
                let impact_angle_error = forward.angle(impact_adjust_direction);

                if within_margin {
                    if impact_angle_error >= exit_margin_threshold {
                        within_margin = false;
                    }
                } else if impact_angle_error <= enter_margin_threshold {
                    within_margin = true;
                    if !args.no_warp {
                        computer.space_center.set_physics_warp_factor(0).await?;
                    }
                }

                if within_margin {
                    control
                        .set_throttle(if impact_error.magnitude() > 10000.0 {
                            1.0
                        } else {
                            0.1
                        })
                        .await?;
                } else {
                    control.set_throttle(0.0).await?;
                }

                if direction_towards_target
                    .dot(impact_adjust_direction)
                    .is_sign_negative()
                {
                    debug!("boostback complete, waiting for chutes to deploy...");
                    boostback_complete = true;
                    control.set_throttle(0.0).await?;
                    control.set_rcs(false).await?;
                }

                continue;
            }

            if !args.no_warp && !warp_complete {
                if sea_level_altitude < 70000.0 {
                    computer.space_center.set_physics_warp_factor(3).await?;
                } else {
                    computer.space_center.set_rails_warp_factor(3).await?;
                }
            }

            if sea_level_altitude < 5000.0 && !warp_complete {
                warp_complete = true;
                computer.space_center.set_physics_warp_factor(0).await?;
                computer.space_center.set_rails_warp_factor(0).await?;
            }

            // aim retrograde until chutes deploy and slow us down
            auto_pilot
                .set_target_direction((-velocity.normalize()).into())
                .await?;
            if surface_altitude < 300.0 {
                // cut chutes
                control.toggle_action_group(2).await?;
                control.set_rcs(true).await?;
                break;
            }
        } else {
            debug!("no impact, canceling landing");
            break;
        }
    }

    debug!("chutes cut, entering standard landing sequence");
    computer
        .land(LandingDescriptor {
            body_position: target_position,
            full_stop_max_tilt: Deg(7.5),
            final_hop_max_tilt: Deg(3.0),
            hover_max_tilt: Deg(3.0),
            suicide_burn_start_factor: 5.0,
            max_stopped_velocity: 2.0,
            ..Default::default()
        })
        .await?;

    control.set_brakes(true).await?;
    control.set_brakes(false).await?;

    computer.reset().await?;
    debug!("BSFP landing complete");

    Ok(())
}
