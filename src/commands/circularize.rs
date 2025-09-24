use anyhow::Result;
use cgmath::InnerSpace;
use clap::Args;

use crate::{
    flight_computer::FlightComputer, maneuver::node_change_opposite_radius,
    orbital_mechanics::LocalOrbit,
};

#[derive(Debug, Args)]
pub struct CircularizeArgs {
    /// Whether to perform the circularization at apoapsis instead of periapsis.
    #[clap(long, short, default_value_t = false)]
    at_apoapsis: bool,
    /// The altitude to adjust the opposite side of your orbit to. If not provided, performs a normal circularization.
    #[clap(long, short = 'o')]
    altitude: Option<f64>,
    /// If provided, at_apoapsis is ignored and the node will be placed this many seconds from now.
    #[clap(long, short = 't')]
    seconds_from_now: Option<f64>,
    /// If provided, altitude will be ignored and the node will instead multiply the oribtal period by this factor.
    #[clap(long, short = 'm')]
    period_multiplier: Option<f64>,
}

pub async fn circularize_command(computer: &FlightComputer, args: &CircularizeArgs) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    let current_ut = computer.space_center.get_ut().await?;
    let node_ut = if let Some(seconds_from_now) = args.seconds_from_now {
        current_ut + seconds_from_now
    } else if args.at_apoapsis {
        orbit.next_apoapsis_ut(current_ut)
    } else {
        orbit.next_periapsis_ut(current_ut)
    };

    let radius = if let Some(period_multiplier) = args.period_multiplier {
        let new_sma = orbit
            .body
            .sma_of_orbital_period(orbit.period() * period_multiplier);
        new_sma * 2.0 - orbit.position_at_ut(node_ut).magnitude()
    } else if let Some(altitude) = args.altitude {
        orbit.body.radius + altitude
    } else {
        orbit.position_at_ut(node_ut).magnitude()
    };

    node_change_opposite_radius(&orbit, radius, node_ut)
        .to_node(&computer.vessel)
        .await?;

    Ok(())
}
