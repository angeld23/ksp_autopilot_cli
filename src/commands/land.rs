use anyhow::Result;
use cgmath::{Angle, Deg, Quaternion, Rad, Rotation3, Vector3};
use clap::Args;
use log::debug;

use crate::{
    flight_computer::FlightComputer,
    landing::LandingDescriptor,
    orbital_mechanics::LocalOrbit,
    trajectory_prediction::{SurfaceTrajectory, TrajectoryPredictionConfig},
    util::{
        coordinates_of_body_position, get_orbit_normal, surface_position,
        surface_position_at_coordinates,
    },
};

#[derive(Debug, Args)]
pub struct LandArgs {
    /// The geocoordinates (in degrees) of the desired landing position. If an impact already exists, this is ignored and the impact
    /// point is used instead. If still in orbit and this value is omitted, this is set to the latitude of the current
    /// target vessel. If no target vessel is selected, this is set to an arbitrary point somewhat ahead in the direction of
    /// the vessel's orbit.
    #[clap(
        long,
        short,
        value_delimiter = ' ',
        num_args = 2,
        allow_negative_numbers = true
    )]
    coordinates: Option<Vec<f64>>,
    /// The offset (in meters, but in the directions of latitude/longitude) to apply to the landing position if a vessel is
    /// targeted. This is to prevent landing directly on top of the targeted vessel.
    #[clap(
        long,
        short,
        value_delimiter = ' ',
        num_args = 2,
        default_value = "15 0",
        allow_negative_numbers = true
    )]
    vessel_offset: Vec<f64>,
    /// The angle (in degrees) between the deorbit point and landing point.
    #[clap(long, short, default_value_t = 30.0)]
    deorbit_angle: f64,
    /// The allowed margin of error (in meters) for the final landing position. Defaults to 5 meters if the landing point was *not*
    /// automatically determined, and 10,000 meters if it was.
    #[clap(long, short)]
    margin: Option<f64>,
    /// The height above the ground to hover at before the final descent.
    #[clap(long, short, default_value_t = 30.0)]
    hover_height: f64,
    /// The final speed that the vessel should be descending at when touching down.
    #[clap(long, short, default_value_t = 1.0)]
    touchdown_speed: f64,
    /// Whether to use RCS thrusters during landing maneuvers.
    #[clap(long, short = 'r', default_value_t = false)]
    no_rcs: bool,
    /// The maximum angle (in degrees) the vessel is allowed to tilt when initially killing its velocity.
    #[clap(long, default_value_t = 75.0, visible_alias = "fsmt")]
    full_stop_max_tilt: f64,
    /// The maximum angle (in degrees) the vessel is allowed to tilt when adjusting its impact point after the initial velocity kill.
    #[clap(long, default_value_t = 30.0, visible_alias = "fhmt")]
    final_hop_max_tilt: f64,
    /// The maximum angle (in degrees) the vessel is allowed to tilt when hovering above the landing point.
    #[clap(long, default_value_t = 30.0, visible_alias = "hmt")]
    hover_max_tilt: f64,
    /// The maximum angle (in degrees) the vessel is allowed to tilt when performing the final descent.
    #[clap(long, default_value_t = 1.0, visible_alias = "fdmt")]
    final_descent_max_tilt: f64,
    /// Whether to skip the suicide burn / velocity killing step.
    #[clap(long, short = 'S', default_value_t = false)]
    skip_suicide_burn: bool,
    /// Whether the suicide burn / velocity killing step should only consider the vertical component of velocity for
    /// determining when to move on to the next step.
    #[clap(long, short = 'V', default_value_t = false)]
    only_kill_vertical_velocity: bool,
    /// The factor to multiply the expected impact time by to determine when to start killing velocity.
    #[clap(long, short = 'f', default_value_t = 1.1)]
    suicide_burn_start_factor: f64,
    /// The maximum speed that the vessel is allowed to travel at when performing the final hover approach.
    #[clap(long, short = 'a', default_value_t = 500.0)]
    max_hover_approach_speed: f64,
    /// The maximum speed that is considered stationary.
    #[clap(long, short = 's')]
    max_stopped_velocity: f64,
    /// Whether to precisely adjust the impact point after deorbiting. If not provided, this is enabled unless the landing point was
    /// decided automatically.
    #[clap(long, short)]
    perform_orbital_correction: Option<bool>,
    /// Whether to slightly overshoot the impact point to cancel out the undershoot caused by the horizontal component of the
    /// suicide burn.
    #[clap(long, short = 'u', default_value_t = false)]
    no_undershoot_compensation: bool,
}

pub async fn land_command(computer: &FlightComputer, args: &LandArgs) -> Result<()> {
    let non_local_orbit = computer.vessel.get_orbit().await?;
    let body = non_local_orbit.get_body().await?;
    let radius = body.get_equatorial_radius().await?;
    let body_frame = body.get_reference_frame().await?;
    let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

    let approx_trajectory = SurfaceTrajectory::predict(
        &computer.vessel,
        TrajectoryPredictionConfig {
            look_ahead: orbit.period(),
            time_step: orbit.period() / 100.0,
            ..Default::default()
        },
    )
    .await?;

    let mut landing_is_precise = true;
    let target_position = if let Some(ref coordinates) = args.coordinates {
        let latitude = Deg(coordinates[0]);
        let longitude = Deg(coordinates[1]);

        surface_position_at_coordinates(&body, latitude, longitude).await?
    } else if let Some(target_vessel) = computer.space_center.get_target_vessel().await? {
        let target_body_position: Vector3<f64> = target_vessel.position(&body_frame).await?.into();
        let (latitude, longitude) = coordinates_of_body_position(target_body_position);
        let latitude = latitude + Rad(args.vessel_offset[0] / radius);
        let longitude = longitude + Rad(args.vessel_offset[1] / (radius * latitude.cos()));

        surface_position_at_coordinates(&body, latitude, longitude).await?
    } else if let Some(impact) = approx_trajectory.impact {
        debug!("impact exists, landing there...");
        landing_is_precise = false;
        impact.position
    } else {
        let current_position: Vector3<f64> = computer.vessel.position(&body_frame).await?.into();
        let orbit_normal = get_orbit_normal(&non_local_orbit, &body_frame).await?;
        let rotation = Quaternion::from_axis_angle(orbit_normal, Deg(-args.deorbit_angle));

        landing_is_precise = false;
        surface_position(&body, rotation * current_position).await?
    };

    if approx_trajectory.impact.is_none() {
        computer
            .deorbit(target_position, Deg(args.deorbit_angle))
            .await?;
    }

    computer
        .land(LandingDescriptor {
            body_position: target_position,
            margin: args
                .margin
                .unwrap_or(if landing_is_precise { 5.0 } else { 10000.0 }),
            hover_height: args.hover_height,
            touchdown_speed: args.touchdown_speed,
            use_rcs: !args.no_rcs,
            full_stop_max_tilt: Deg(args.full_stop_max_tilt),
            final_hop_max_tilt: Deg(args.final_hop_max_tilt),
            hover_max_tilt: Deg(args.hover_max_tilt),
            final_descent_max_tilt: Deg(args.final_descent_max_tilt),
            skip_suicide_burn: args.skip_suicide_burn,
            only_kill_vertical_velocity: args.only_kill_vertical_velocity,
            suicide_burn_start_factor: args.suicide_burn_start_factor,
            max_hover_approach_speed: args.max_hover_approach_speed,
            max_stopped_velocity: args.max_stopped_velocity,
            perform_orbital_correction: args
                .perform_orbital_correction
                .unwrap_or(landing_is_precise),
            undershoot_compensation: !args.no_undershoot_compensation,
        })
        .await?;

    Ok(())
}
