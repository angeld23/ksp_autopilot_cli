#![feature(box_into_inner)]

use core::{f32, f64};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::Result;
use ascent::{AscentDescriptor, InclinationTarget};
use cgmath::{vec3, Angle, Deg, InnerSpace, Rad, Vector3, Zero};
use krpc_client::{
    services::{
        drawing::Drawing,
        space_center::{SpaceCenter, Vessel, VesselSituation},
    },
    Client,
};
use maneuver::{
    node_change_opposite_radius, node_change_orbit_normal, node_circularize, node_hohmann_transfer,
};
use orbital_mechanics::{
    LocalBody, LocalOrbit, LocalUniverse, OrbitalStateChange, OrbitalTrajectory,
};
use tokio::{spawn, sync::Mutex, time};
use translation::{TranslationController, TranslationTarget};
use util::{
    get_apoapsis_speed, get_ascending_node_direction, get_next_node, get_orbit_normal,
    get_orbital_period, get_periapsis_direction, get_periapsis_speed, orbital_velocity_at,
    ut_at_true_anomaly_offset,
};

mod ascent;
mod execute_node;
mod maneuver;
mod orbital_mechanics;
mod translation;
mod util;

#[derive(Debug, Default, Clone, Copy)]
pub struct ControlConfig {
    pub rcs: bool,
    pub sas: bool,
}

pub struct FlightComputer {
    pub space_center: Arc<SpaceCenter>,
    pub drawing: Arc<Drawing>,
    pub local_universe: LocalUniverse,
    pub vessel: Vessel,
    pub translation_controller: Mutex<TranslationController>,
    pub auto_stage: bool,
    pub execute_node_use_rcs: bool,
    pub saved_control_config: Mutex<Option<ControlConfig>>,
}

impl FlightComputer {
    pub async fn update(&self) -> Result<()> {
        let control = self.vessel.get_control().await?;

        // let auto_pilot = self.vessel.get_auto_pilot().await?;
        let current_stage = control.get_current_stage().await?;
        let parts = self.vessel.get_parts().await?;
        let engines = parts.get_engines().await?;

        // auto-staging
        if self.auto_stage
            && !matches!(
                self.vessel.get_situation().await?,
                VesselSituation::PreLaunch
            )
            && current_stage > 0
            && !control.get_stage_lock().await?
        {
            let mut next_decoupled_engines_all_depleted = true;
            let mut engine_in_any_future_stage = false;
            for engine in engines.iter() {
                let part = engine.get_part().await?;
                let active_stage = part.get_stage().await?;
                let decouple_stage = part.get_decouple_stage().await?;
                let has_fuel = engine.get_has_fuel().await?;
                let name = part.get_name().await?;

                // this counts engines that never get activated via staging (their stage is -1), but that's fine
                if active_stage < current_stage {
                    engine_in_any_future_stage = true;
                }

                if decouple_stage == current_stage - 1 && has_fuel && name != "sepMotor1" {
                    next_decoupled_engines_all_depleted = false;
                }
            }

            if next_decoupled_engines_all_depleted && engine_in_any_future_stage {
                control.activate_next_stage().await?;
            }
        }

        self.translation_controller
            .lock()
            .await
            .update(&self.vessel)
            .await?;

        Ok(())
    }

    pub async fn save_control_config(&self) -> Result<()> {
        let control = self.vessel.get_control().await?;
        *self.saved_control_config.lock().await = Some(ControlConfig {
            rcs: control.get_rcs().await?,
            sas: control.get_sas().await?,
        });

        Ok(())
    }

    pub async fn load_control_config(&self) -> Result<()> {
        if let Some(config) = self.saved_control_config.lock().await.as_ref() {
            let control = self.vessel.get_control().await?;
            control.set_rcs(config.rcs).await?;
            control.set_sas(config.sas).await?;
        }

        Ok(())
    }

    pub async fn reset(&self) -> Result<()> {
        self.translation_controller
            .lock()
            .await
            .reset(&self.vessel)
            .await?;

        let reference_frame = self.vessel.get_reference_frame().await?;
        let control = self.vessel.get_control().await?;
        let auto_pilot = self.vessel.get_auto_pilot().await?;

        auto_pilot.set_reference_frame(&reference_frame).await?;
        auto_pilot.disengage().await?;
        control.set_throttle(0.0).await?;

        self.load_control_config().await?;

        Ok(())
    }

    pub async fn wait_until(&self, ut: f64) -> Result<()> {
        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 30.0));
        loop {
            interval.tick().await;
            if self.space_center.get_ut().await? >= ut {
                break;
            }
        }

        Ok(())
    }

    pub async fn wait(&self, time: f64) -> Result<()> {
        let start_ut = self.space_center.get_ut().await?;
        self.wait_until(start_ut + time).await
    }
}

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
            target: TranslationTarget::Throttle(Vector3::zero()),
            reference_frame: Some(
                space_center
                    .get_bodies()
                    .await?
                    .get("Kerbin")
                    .unwrap()
                    .get_reference_frame()
                    .await?,
            ),
            target_position_max_speed: 0.0,
            enabled: true,
        }),
        auto_stage: true,
        execute_node_use_rcs: true,
        saved_control_config: Mutex::new(None),
    });

    let a = space_center.get_active_vessel().await?.get_orbit().await?;
    let b = space_center.get_active_vessel().await?.get_orbit().await?;

    unsafe {
        let c = std::mem::transmute::<_, &[u8; 16]>(&a);
        let d = std::mem::transmute::<_, &[u8; 16]>(&b);

        println!("a: {:?}", c);
        println!("b: {:?}", d);
    }

    {
        let computer = Arc::clone(&computer);
        spawn(async move {
            //computer.node_circularize_at_apsis(false).await.unwrap();
            // node_circularize(
            //     &computer.vessel,
            //     space_center.get_ut().await.unwrap() + 30.0,
            // )
            // .await
            // .unwrap();

            // let target = space_center.get_target_vessel().await.unwrap().unwrap();
            // let target_orbit = target.get_orbit().await.unwrap();
            // let reference_frame = computer.vessel.get_reference_frame().await.unwrap();

            let non_local_target = space_center.get_target_body().await.unwrap().unwrap();
            let target = LocalBody::from_celestial_body(&non_local_target)
                .await
                .unwrap();
            // let target_orbit = target.get_orbit().await.unwrap().unwrap();

            // node_hohmann_transfer(&computer.vessel, &target_orbit, 0.0)
            //     .await
            //     .unwrap();

            let orbit = computer.vessel.get_orbit().await.unwrap();
            // space_center
            //     .warp_to(
            //         ut_at_true_anomaly_offset(&orbit, Rad::full_turn() * 3.0)
            //             .await
            //             .unwrap(),
            //         f32::INFINITY,
            //         f32::INFINITY,
            //     )
            //     .await
            //     .unwrap();

            // computer
            //     .ascent(AscentDescriptor {
            //         inclination_target: InclinationTarget::OrbitNormal(
            //             get_orbit_normal(&target_orbit, &reference_frame)
            //                 .await
            //                 .unwrap(),
            //             reference_frame,
            //         ),
            //         ..Default::default()
            //     })
            //     .await
            //     .unwrap();

            let local_orbit = LocalOrbit::from_orbit(&orbit).await.unwrap();
            let ut = space_center.get_ut().await.unwrap();
            let reference_frame = orbit
                .get_body()
                .await
                .unwrap()
                .get_non_rotating_reference_frame()
                .await
                .unwrap();
            let vessel_reference_frame = computer.vessel.get_reference_frame().await.unwrap();

            // let node = get_next_node(&computer.vessel).await.unwrap().unwrap();
            // let node_ut = node.get_ut().await.unwrap();
            // let node_velocity_change = local_orbit.prograde_normal_radial_to_absolute(
            //     node_ut,
            //     vec3(
            //         node.get_prograde().await.unwrap(),
            //         node.get_normal().await.unwrap(),
            //         node.get_radial().await.unwrap(),
            //     ),
            // );

            macro_rules! compare_vectors {
                ($a:expr, $b:expr) => {
                    println!(
                        "a={:?}\nb={:?}\nerr={} (mdiff={})",
                        $a,
                        $b,
                        ($a - $b).magnitude(),
                        $a.magnitude() - $b.magnitude()
                    );
                };
            }

            let mut trajectory = local_orbit.create_trajectory(ut);
            // trajectory.state_changes.push(OrbitalStateChange {
            //     body: Box::into_inner(local_orbit.body.clone()),
            //     ut: ut,
            //     position: local_orbit.position_at_ut(node_ut),
            //     velocity: local_orbit.velocity_at_ut(node_ut) + node_velocity_change,
            // });
            computer
                .local_universe
                .fill_orbital_trajectory(&mut trajectory, ut + 384031.0);
            println!("{:#?}", trajectory);
            println!("{:#?}", trajectory.orbit_at_ut(ut + 384031.0));
            println!("---");
            println!(
                "{:#?} vs {:#?}",
                local_orbit,
                trajectory.orbit_at_ut(ut + 1.0)
            );
            let soi_enter_ut = target
                .find_soi_enter(&trajectory.orbit_at_ut(ut + 1.0), ut, 384031.0)
                .unwrap();
            println!("ut={}", ut);
            println!("soi_enter_ut={}", soi_enter_ut);
            let mun_orbit = trajectory.orbit_at_ut(soi_enter_ut + 1.0);
            let final_orbit = trajectory.orbit_at_ut(ut + 6.0 * 3600.0);

            compare_vectors!(
                mun_orbit.velocity_at_ut(soi_enter_ut)
                    + target.orbit.as_ref().unwrap().velocity_at_ut(soi_enter_ut),
                local_orbit.velocity_at_ut(soi_enter_ut)
            );
            compare_vectors!(
                mun_orbit.position_at_ut(soi_enter_ut),
                local_orbit.position_at_ut(soi_enter_ut)
                    - target.orbit.as_ref().unwrap().position_at_ut(soi_enter_ut)
            );

            println!("hey {:#?}", soi_enter_ut);

            // computer
            //     .vessel
            //     .get_control()
            //     .await
            //     .unwrap()
            //     .add_node(soi_enter_ut, 0.0, 0.0, 0.0)
            //     .await
            //     .unwrap();

            // let local_target_orbit = LocalOrbit::from_orbit(&target_orbit).await.unwrap();

            // let start_time = Instant::now();
            // let (closest_approach_ut, closest_approach_distance) = local_orbit
            //     .find_closest_approach(&local_target_orbit, ut, local_orbit.period() * 2.0)
            //     .unwrap();

            // let soi_enter_ut = computer
            //     .local_universe
            //     .get_local_body(&target)
            //     .await
            //     .unwrap()
            //     .find_soi_enter(&local_orbit, ut, local_orbit.period())
            //     .unwrap();

            // println!("enter soi in {:?}s", soi_enter_ut - ut);

            // let soi_exit_ut = local_orbit.find_soi_exit(ut).unwrap();

            // println!(
            //     "found in {}ms - closest approach of {}m occurs in {}s",
            //     start_time.elapsed().as_micros() as f64 / 1000.0,
            //     closest_approach_distance,
            //     closest_approach_ut - ut
            // );

            // space_center
            //     .warp_to(soi_enter_ut, f32::INFINITY, f32::INFINITY)
            //     .await
            //     .unwrap();

            // node_change_orbit_normal(
            //     &computer.vessel,
            //     get_orbit_normal(&target_orbit, &reference_frame)
            //         .await
            //         .unwrap(),
            //     Some(&reference_frame),
            //     false,
            // )
            // .await
            // .unwrap();

            // computer.execute_next_node().await.unwrap();
        });
    }

    let mut interval = time::interval(Duration::from_secs_f64(1.0 / 30.0));
    loop {
        interval.tick().await;
        computer.update().await?;
    }
}
