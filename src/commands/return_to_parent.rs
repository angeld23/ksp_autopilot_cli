use anyhow::Result;
use clap::Args;

use crate::{
    flight_computer::FlightComputer, maneuver::node_return_to_parent_body,
    orbital_mechanics::LocalOrbit,
};

#[derive(Debug, Args)]
pub struct ReturnToParentArgs {
    /// The desired periapsis. Defaults to either 10km above the atmosphere or 30km above the surface, whichever is greater.
    #[clap(long, short)]
    periapsis: Option<f64>,
}

pub async fn return_to_parent_command(
    computer: &FlightComputer,
    args: &ReturnToParentArgs,
) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
    let parent_body = non_local_orbit
        .get_body()
        .await?
        .get_orbit()
        .await?
        .unwrap()
        .get_body()
        .await?;

    let atmosphere_depth = parent_body.get_atmosphere_depth().await?;
    let altitude = args
        .periapsis
        .unwrap_or((atmosphere_depth + 10000.0).max(30000.0));

    node_return_to_parent_body(
        &orbit,
        parent_body.get_equatorial_radius().await? + altitude,
        computer.space_center.get_ut().await?,
    )
    .to_node(&computer.vessel)
    .await?;

    Ok(())
}
