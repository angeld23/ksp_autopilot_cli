use anyhow::Result;
use cgmath::{vec3, Deg, Quaternion, Rotation3};
use clap::Args;

use crate::{
    flight_computer::FlightComputer,
    maneuver::node_change_orbit_normal,
    orbital_mechanics::LocalOrbit,
    util::{get_orbit_normal, get_target_orbit},
};

#[derive(Debug, Args)]
pub struct AdjustPlaneArgs {
    /// The desired inclination, in degrees. If none is provided and a target is selected, the new plane will match that
    /// of the target's orbit. If no target is selected, this defaults to the orbital inclination of the body being orbited.
    #[clap(long, short)]
    inclination: Option<f64>,
    /// If this is enabled, the maneuver will be placed on whichever of the two possibilities requires less Δv. Otherwise,
    /// the sooner one is chosen.
    #[clap(long, short, default_value_t = false)]
    save_fuel: bool,
}

pub async fn adjust_plane_command(computer: &FlightComputer, args: &AdjustPlaneArgs) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    let normal = if let Some(inclination) = args.inclination {
        Quaternion::from_axis_angle(orbit.ascending_node_direction(), Deg(inclination))
            * vec3(0.0, 1.0, 0.0)
    } else if let Some(target_orbit) = get_target_orbit(&computer.space_center).await? {
        get_orbit_normal(
            &target_orbit,
            &non_local_orbit
                .get_body()
                .await?
                .get_non_rotating_reference_frame()
                .await?,
        )
        .await?
    } else {
        let body = non_local_orbit.get_body().await?;
        get_orbit_normal(
            &body.get_orbit().await?.unwrap(),
            &body.get_non_rotating_reference_frame().await?,
        )
        .await?
    };

    node_change_orbit_normal(
        &orbit,
        normal,
        args.save_fuel,
        computer.space_center.get_ut().await?,
    )
    .to_node(&computer.vessel)
    .await?;

    Ok(())
}
