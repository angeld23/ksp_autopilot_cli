#![feature(box_into_inner, sort_floats)]
#![allow(dead_code)]

use core::{f32, f64};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::Result;
use ascent::{AscentDescriptor, InclinationTarget};
use cgmath::{vec3, Angle, Deg, InnerSpace, Rad, Vector3, Zero};
use flight_computer::FlightComputer;
use krpc_client::{
    services::{
        drawing::Drawing,
        space_center::{SpaceCenter, Vessel, VesselSituation},
    },
    Client,
};
use landing::LandingDescriptor;
use maneuver::{
    node_change_opposite_radius, node_change_orbit_normal, node_circularize, node_hohmann_transfer,
    node_hohmann_transfer_to_body,
};
use orbital_mechanics::{
    LocalBody, LocalOrbit, LocalUniverse, OrbitalStateChange, OrbitalTrajectory,
};
use root_finding::newton_find_all_roots;
use tokio::{spawn, sync::Mutex, time};
use trajectory_prediction::SurfaceTrajectory;
use translation::{TranslationController, TranslationTarget};
use util::{
    get_apoapsis_speed, get_ascending_node_direction, get_current_ut_from_orbit, get_next_node,
    get_orbit_normal, get_orbital_period, get_periapsis_direction, get_periapsis_speed,
    get_target_orbit, orbital_velocity_at, ut_at_true_anomaly_offset,
};

mod ascent;
mod execute_node;
mod flight_computer;
mod landing;
mod maneuver;
mod numerical_integration;
mod orbital_mechanics;
mod root_finding;
mod scripts;
mod trajectory_prediction;
mod translation;
mod util;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let client = Client::new("rust client", "127.0.0.1", 50000, 50001).await?;
    let space_center = Arc::new(SpaceCenter::new(client.clone()));
    let drawing = Arc::new(Drawing::new(client.clone()));
    let computer = Arc::new(FlightComputer {
        space_center: space_center.clone(),
        drawing: drawing.clone(),
        local_universe: LocalUniverse::create(&space_center).await?,
        vessel: space_center.get_active_vessel().await?,
        translation_controller: Mutex::new(TranslationController {
            target: TranslationTarget::Acceleration(Vector3::zero()),
            reference_frame: None,
            vtol_enabled: false,
            vtol_max_tilt: Deg(30.0),
            rcs_enabled: false,
            enabled: false,
        }),
        auto_stage: true,
        execute_node_use_rcs: true,
        saved_control_config: Mutex::new(None),
    });

    {
        let computer = Arc::clone(&computer);
        spawn(async move {
            let target = space_center.get_target_body().await?.unwrap();
            let target_body = LocalBody::from_celestial_body(&target).await?;

            node_hohmann_transfer_to_body(&computer.vessel, &target_body, 230000.0).await?;

            Ok::<(), anyhow::Error>(())
        });
    }

    let mut interval = time::interval(Duration::from_secs_f64(1.0 / 20.0));
    loop {
        interval.tick().await;
        computer.update().await?;
    }
}
