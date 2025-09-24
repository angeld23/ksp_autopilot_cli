use anyhow::Result;
use cgmath::Deg;
use clap::Args;

use crate::{
    ascent::{AscentDescriptor, InclinationTarget},
    flight_computer::FlightComputer,
    util::{get_orbit_normal, get_target_orbit},
};

#[derive(Debug, Args)]
pub struct AscentCommandArgs {
    /// The desired altitude. Defaults to either 10km above the atmosphere or 30km above the surface, whichever is greater.
    #[clap(long, short)]
    altitude: Option<f64>,
    /// The desired inclination of the orbit, in degrees. If none is provided and a target is selected, the ascent will match the
    /// plane of the target's orbit. If no target is selected, this defaults to 0.
    #[clap(long, short)]
    inclination: Option<f64>,
    /// The altitude from the ground to start the gravity turn at.
    #[clap(long, short, default_value_t = 200.0)]
    turn_start_height: f64,
    /// The maximum angle (in degrees) between the direction the craft is facing and surface prograde.
    #[clap(long, short, default_value_t = 15.0)]
    max_prograde_error: f64,
    /// Whether to circularize at apoapsis.
    #[clap(long, short = 'c', default_value_t = false)]
    no_circularize: bool,
    /// Whether to do an extra maneuver after circularization to correct a lobsided orbit.
    #[clap(long, short = 'l', default_value_t = false)]
    no_correct_lobsided_orbit: bool,
    /// Whether to warp while in the atmosphere during ascent.
    #[clap(long, short = 'w', default_value_t = false)]
    no_atmosphere_warp: bool,
    /// Whether to match the orbital plane of the body being launched from. This is useful if you plan to return to a parent body.
    #[clap(long, short = 'p', default_value_t = false)]
    match_body_plane: bool,
}

pub async fn ascent_command(computer: &FlightComputer, args: &AscentCommandArgs) -> Result<()> {
    let body = computer.vessel.get_orbit().await?.get_body().await?;
    let atmosphere_depth = body.get_atmosphere_depth().await?;

    let inclination_target = if let Some(inclination) = args.inclination {
        InclinationTarget::Inclination(Deg(inclination))
    } else if args.match_body_plane {
        let frame = body.get_non_rotating_reference_frame().await?;
        InclinationTarget::OrbitNormal(
            get_orbit_normal(&body.get_orbit().await?.unwrap(), &frame).await?,
            frame,
        )
    } else if let Some(target_orbit) = get_target_orbit(&computer.space_center).await? {
        let frame = body.get_non_rotating_reference_frame().await?;
        InclinationTarget::OrbitNormal(get_orbit_normal(&target_orbit, &frame).await?, frame)
    } else {
        InclinationTarget::Inclination(Deg(0.0))
    };

    let altitude = args
        .altitude
        .unwrap_or((atmosphere_depth + 10000.0).max(30000.0));
    computer
        .ascent(AscentDescriptor {
            altitude,
            turn_start_height: args.turn_start_height,
            inclination_target,
            max_prograde_error: Deg(args.max_prograde_error),
            circularize: !args.no_circularize,
            do_atmosphere_warp: !args.no_atmosphere_warp,
            correct_lobsided_orbit: !args.no_correct_lobsided_orbit,
        })
        .await
}
