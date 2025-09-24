use anyhow::Result;
use clap::Args;
use log::warn;

use crate::{
    flight_computer::FlightComputer,
    maneuver::{node_tune_body_periapsis, node_tune_closest_approach},
    orbital_mechanics::{LocalBody, LocalOrbit},
    util::get_target_orbit,
};

#[derive(Debug, Args)]
pub struct TuneClosestApproachArgs {
    /// The desired distance.
    /// If your target is a celestial body, this defaults to either 10km above the atmosphere or 30km above the surface, whichever is greater.
    /// Otherwise, it defaults to 0.
    #[clap(long, short)]
    distance: Option<f64>,
    /// The amount of seconds from now to place the maneuver node.
    #[clap(long, short = 't', default_value_t = 30.0)]
    seconds_from_now: f64,
    /// The maximum amount of Δv the maneuver is allowed to use.
    #[clap(long, short, default_value_t = 100.0)]
    max_delta_v: f64,
}

pub async fn tune_closest_approach_command(
    computer: &FlightComputer,
    args: &TuneClosestApproachArgs,
) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    let ut = computer.space_center.get_ut().await? + args.seconds_from_now;

    if let Some(non_local_body) = computer.space_center.get_target_body().await? {
        let target_body = LocalBody::from_celestial_body(&non_local_body).await?;

        let atmosphere_depth = non_local_body.get_atmosphere_depth().await?;
        let altitude = args
            .distance
            .unwrap_or((atmosphere_depth + 10000.0).max(30000.0));

        node_tune_body_periapsis(
            &orbit,
            &target_body,
            ut,
            target_body.radius + altitude,
            args.max_delta_v,
        )
        .to_node(&computer.vessel)
        .await?;
    } else if let Some(non_local_target_orbit) = get_target_orbit(&computer.space_center).await? {
        let target_orbit = LocalOrbit::from_orbit(&non_local_target_orbit).await?;

        let offset = args.distance.unwrap_or(0.0);
        node_tune_closest_approach(&orbit, &target_orbit, ut, offset, args.max_delta_v)
            .to_node(&computer.vessel)
            .await?;
    } else {
        warn!("no target selected")
    }

    Ok(())
}
