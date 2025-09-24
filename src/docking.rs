use std::time::Duration;

use anyhow::Result;
use cgmath::{vec3, InnerSpace, Quaternion, Rad, Rotation, Rotation3, Vector3};
use krpc_client::services::space_center::{DockingPort, DockingPortState};
use log::debug;
use tokio::time;

use crate::{
    flight_computer::FlightComputer,
    translation::TranslationTarget,
    util::{approximate_vessel_radius, flatten_vector, set_target_rotation},
};

#[derive(Debug, Clone, Copy)]
pub struct DockingDescriptor {
    pub alignment_speed: f64,
    pub pre_approach_distance: f64,
    pub approach_speed: f64,
    pub slow_approach_distance: f64,
    pub slow_approach_speed: f64,
    pub roll: Rad<f64>,
}

impl Default for DockingDescriptor {
    fn default() -> Self {
        Self {
            alignment_speed: 2.0,
            pre_approach_distance: 15.0,
            approach_speed: 1.0,
            slow_approach_distance: 3.0,
            slow_approach_speed: 0.25,
            roll: Rad(0.0),
        }
    }
}

impl FlightComputer {
    pub async fn dock(
        &self,
        port: &DockingPort,
        target_port: &DockingPort,
        descriptor: DockingDescriptor,
    ) -> Result<()> {
        debug!("beginning docking sequence...");

        self.save_control_config().await?;

        let port_part = port.get_part().await?;
        let target_part = target_port.get_part().await?;
        let target_vessel = target_part.get_vessel().await?;

        let orbit = self.vessel.get_orbit().await?;
        let body = orbit.get_body().await?;

        let non_rotating_frame = body.get_non_rotating_reference_frame().await?;
        let vessel_frame = self.vessel.get_reference_frame().await?;
        let target_vessel_frame = target_vessel.get_reference_frame().await?;
        let docking_frame = vessel_frame
            .static_create_hybrid(
                &target_vessel_frame,
                &non_rotating_frame,
                &target_vessel_frame,
                &non_rotating_frame,
            )
            .await?;

        let radius_0 = approximate_vessel_radius(&self.vessel).await?;
        let radius_1 = approximate_vessel_radius(&target_vessel).await?;
        let min_safe_distance = radius_0 + radius_1 + 10.0;

        let auto_pilot = self.vessel.get_auto_pilot().await?;
        auto_pilot.set_reference_frame(&docking_frame).await?;
        auto_pilot.set_attenuation_angle((5.0, 5.0, 5.0)).await?;
        auto_pilot.engage().await?;

        let control = self.vessel.get_control().await?;
        control.set_rcs(true).await?;

        {
            // initialize translation controller
            let mut translation_controller = self.translation_controller.lock().await;
            translation_controller.reset(&self.vessel).await?;
            // we create the docking frame a second time because the translation controller needs to own it
            translation_controller.reference_frame = Some(
                vessel_frame
                    .static_create_hybrid(
                        &target_vessel_frame,
                        &non_rotating_frame,
                        &target_vessel_frame,
                        &non_rotating_frame,
                    )
                    .await?,
            );
            translation_controller.rcs_enabled = true;
            translation_controller.enabled = true;
        }

        let mut sidestep_done = false;
        let mut passby_done = false;
        let mut approach_started = false;
        let mut auto_pilot_engaged = true;

        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 20.0));
        loop {
            interval.tick().await;
            if matches!(target_port.get_state().await?, DockingPortState::Docked) {
                break;
            }

            let mut translation_controller = self.translation_controller.lock().await;

            let vessel_position: Vector3<f64> = self.vessel.position(&docking_frame).await?.into();
            let vessel_velocity: Vector3<f64> = self.vessel.velocity(&docking_frame).await?.into();
            let vessel_rotation: Quaternion<f64> =
                self.vessel.rotation(&docking_frame).await?.into();

            let port_position: Vector3<f64> = port_part.position(&docking_frame).await?.into();
            let port_rotation: Quaternion<f64> = port_part.rotation(&docking_frame).await?.into();

            let target_port_position: Vector3<f64> =
                target_part.position(&docking_frame).await?.into();
            let target_port_rotation: Quaternion<f64> =
                target_part.rotation(&docking_frame).await?.into();
            let target_port_direction = target_port_rotation * vec3(0.0, 1.0, 0.0);

            let port_relative_rotation = vessel_rotation.invert() * port_rotation;
            let desired_vessel_rotation =
                Quaternion::from_arc(vec3(0.0, 1.0, 0.0), -target_port_direction, None)
                    * Quaternion::from_axis_angle(target_port_direction, descriptor.roll)
                    * port_relative_rotation.invert();
            let vessel_offset = desired_vessel_rotation
                * vessel_rotation.invert()
                * (vessel_position - port_position);

            set_target_rotation(&auto_pilot, desired_vessel_rotation).await?;

            let projected_target_port_position =
                flatten_vector(target_port_position, target_port_direction);
            let port_axis_target_port_position =
                target_port_position - projected_target_port_position;

            let projected_vessel_position = flatten_vector(vessel_position, target_port_direction);
            let port_axis_vessel_position = vessel_position - projected_vessel_position;

            if !sidestep_done {
                if projected_vessel_position.magnitude() > min_safe_distance {
                    sidestep_done = true;
                    translation_controller.reset_target();
                    continue;
                }

                if translation_controller.target_is_reset() {
                    debug!("getting out of the way...");
                }
                translation_controller.target = TranslationTarget::Position {
                    pos: port_axis_vessel_position
                        + projected_vessel_position.normalize_to(min_safe_distance + 3.0),
                    max_speed: descriptor.alignment_speed,
                    margin: 3.0,
                };

                continue;
            }

            if !passby_done {
                let distance = target_port_direction
                    .dot(port_axis_vessel_position - port_axis_target_port_position);

                if distance >= descriptor.pre_approach_distance - 3.0 {
                    passby_done = true;
                    translation_controller.reset_target();
                    continue;
                }

                if translation_controller.target_is_reset() {
                    debug!("moving to other side...");
                }
                translation_controller.target = TranslationTarget::Position {
                    pos: projected_vessel_position
                        + port_axis_target_port_position
                        + target_port_direction.normalize_to(descriptor.pre_approach_distance),
                    max_speed: descriptor.alignment_speed,
                    margin: 3.0,
                };

                continue;
            }

            if !approach_started {
                let pre_approach_position = target_port_position
                    + target_port_direction.normalize_to(descriptor.pre_approach_distance)
                    + vessel_offset;
                if (vessel_position - pre_approach_position).magnitude() < 0.5
                    && vessel_velocity.magnitude() < 0.25
                {
                    approach_started = true;
                    translation_controller.reset_target();
                    debug!("approaching...");
                    continue;
                }

                if translation_controller.target_is_reset() {
                    debug!("moving above port...");
                }
                translation_controller.target = TranslationTarget::Position {
                    pos: pre_approach_position,
                    max_speed: descriptor.alignment_speed,
                    margin: 0.5,
                };

                continue;
            }

            let port_distance = (target_port_position - port_position).magnitude();
            translation_controller.target = TranslationTarget::Position {
                pos: target_port_position + vessel_offset,
                max_speed: if port_distance > descriptor.slow_approach_distance {
                    descriptor.approach_speed
                } else {
                    descriptor.slow_approach_speed
                },
                margin: 1.0,
            };

            let auto_pilot_should_engage = port_distance > 1.0;
            if auto_pilot_should_engage != auto_pilot_engaged {
                if auto_pilot_should_engage {
                    auto_pilot.engage().await?;
                } else {
                    auto_pilot.disengage().await?;
                }
            }
            auto_pilot_engaged = auto_pilot_should_engage;
        }

        debug!("docked!");
        self.reset().await?;

        Ok(())
    }
}
