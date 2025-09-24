use anyhow::Result;
use clap::Args;
use log::warn;

use crate::{
    flight_computer::FlightComputer,
    maneuver::{node_hohmann_transfer, node_hohmann_transfer_to_body},
    orbital_mechanics::{LocalBody, LocalOrbit},
    util::get_target_orbit,
};

#[derive(Debug, Args)]
pub struct TransferArgs {
    /// The desired distance.
    /// If your target is a celestial body, this defaults to either 10km above the atmosphere or 30km above the surface, whichever is greater.
    /// Otherwise, it defaults to 0.
    #[clap(long, short)]
    distance: Option<f64>,
}

pub async fn transfer_command(computer: &FlightComputer, args: &TransferArgs) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    if let Some(non_local_body) = computer.space_center.get_target_body().await? {
        let target_body = LocalBody::from_celestial_body(&non_local_body).await?;

        let atmosphere_depth = non_local_body.get_atmosphere_depth().await?;
        let altitude = args
            .distance
            .unwrap_or((atmosphere_depth + 10000.0).max(30000.0));

        node_hohmann_transfer_to_body(
            &orbit,
            &target_body,
            target_body.radius + altitude,
            computer.space_center.get_ut().await?,
        )
        .to_node(&computer.vessel)
        .await?;
    } else if let Some(non_local_target_orbit) = get_target_orbit(&computer.space_center).await? {
        let target_orbit = LocalOrbit::from_orbit(&non_local_target_orbit).await?;

        let offset = args.distance.unwrap_or(0.0);
        node_hohmann_transfer(
            &orbit,
            &target_orbit,
            offset,
            computer.space_center.get_ut().await?,
        )
        .to_node(&computer.vessel)
        .await?;
    } else {
        warn!("no target selected")
    }

    Ok(())
}
