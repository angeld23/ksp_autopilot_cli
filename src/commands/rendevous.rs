use anyhow::Result;
use clap::Args;
use log::warn;

use crate::{
    flight_computer::FlightComputer, orbital_mechanics::LocalOrbit, rendevous::RendevousDescriptor,
    util::get_target_orbit,
};

#[derive(Debug, Args)]
pub struct RendevousArgs {
    /// The maximum amount of orbits it should take to begin the rendevous. If the next rendevous opportunity is further away than
    /// this, the vessel will be moved to a better phasing orbit.
    #[clap(long, short = 'o', default_value_t = 5.0)]
    max_orbits: f64,
    /// How far away you want to end up from the target.
    #[clap(long, short, default_value_t = 100.0)]
    distance: f64,
    /// Whether to perform an extra burn to tune the closest approach after the initial intercept burn.
    #[clap(long, short = 't')]
    no_tune_closest_approach: bool,
    /// Whether to match the velocity of the target at intercept.
    #[clap(long, short = 'v')]
    no_match_velocity: bool,
}

pub async fn rendevous_command(computer: &FlightComputer, args: &RendevousArgs) -> Result<()> {
    if let Some(target_orbit) = get_target_orbit(&computer.space_center).await? {
        computer
            .rendevous(
                &LocalOrbit::from_orbit(&target_orbit).await?,
                RendevousDescriptor {
                    max_orbits: args.max_orbits,
                    distance: args.distance,
                    tune_closest_approach: !args.no_tune_closest_approach,
                    match_velocity: !args.no_match_velocity,
                },
            )
            .await?;
    } else {
        warn!("no target selected");
    }

    Ok(())
}
