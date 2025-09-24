use std::time::Duration;

use anyhow::Result;
use cgmath::{vec3, Deg, InnerSpace, Quaternion, Rad};
use krpc_client::services::space_center::Node;
use log::debug;
use tokio::time;

use crate::{
    translation::TranslationTarget,
    util::{
        get_burn_info, get_next_node, get_translation_force, node_exists, throttle_for_acceleration,
    },
    FlightComputer,
};

impl FlightComputer {
    pub async fn execute_node(&self, node: &Node) -> Result<()> {
        let node_reference_frame = node.get_reference_frame().await?;
        let flight = self.vessel.flight(Some(&node_reference_frame)).await?;

        let control = self.vessel.get_control().await?;
        let auto_pilot = self.vessel.get_auto_pilot().await?;

        auto_pilot
            .set_reference_frame(&node_reference_frame)
            .await?;
        auto_pilot.set_roll_threshold(0.0).await?;
        auto_pilot.set_target_direction((0.0, 1.0, 0.0)).await?;

        auto_pilot.engage().await?;

        self.save_control_config().await?;

        let mut within_margin = false;
        let enter_margin_threshold: Rad<f64> = Deg(1.0).into();
        let exit_margin_threshold: Rad<f64> = Deg(3.0).into();

        let total_delta_v = node.get_remaining_delta_v().await?;
        let burn_info = get_burn_info(&self.vessel, total_delta_v, Some(0.0)).await?;

        debug!(
            "executing node:\n\tΔv = {:.3}m/s\n\tΔt = {:.2}s\n\tΔs = {:.1}m",
            total_delta_v, burn_info.burn_time, burn_info.distance_covered
        );
        debug!("pointing...");

        let mut warped = false;

        {
            let mut translation_controller = self.translation_controller.lock().await;
            translation_controller.reset(&self.vessel).await?;
        }

        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 20.0));
        loop {
            interval.tick().await;
            if !node_exists(node, &self.vessel).await? {
                debug!("node removed, execution canceled");
                break;
            }

            let mut translation_controller = self.translation_controller.lock().await;

            let rotation: Quaternion<f64> = flight.get_rotation().await?.into();
            let forward = rotation * vec3(0.0, 1.0, 0.0);

            let margin = forward.angle(vec3(0.0, 1.0, 0.0));
            if within_margin {
                if margin >= exit_margin_threshold {
                    within_margin = false;
                }
            } else if margin <= enter_margin_threshold {
                within_margin = true;
            }

            if within_margin && !warped {
                debug!("pointed. warping to node...");
                warped = true;
                self.space_center
                    .warp_to(
                        node.get_ut().await? - burn_info.burn_time / 2.0,
                        f32::INFINITY,
                        f32::INFINITY,
                    )
                    .await?;
            }

            let remaining_delta_v = node.get_remaining_delta_v().await?;
            let try_rcs = self.execute_node_use_rcs && remaining_delta_v <= 1.0;
            if try_rcs {
                control.set_rcs(true).await?;
            }

            let rcs_force = get_translation_force(
                &self.vessel,
                vec3(0.0, 1.0, 0.0),
                Some(&node_reference_frame),
            )
            .await?;

            let mass = self.vessel.get_mass().await? as f64;
            let rcs_burn_time = remaining_delta_v / (rcs_force / mass);
            let should_use_rcs = try_rcs && rcs_burn_time < 1.0;

            let time_to_node = node.get_time_to().await?;
            let burn_started = time_to_node <= burn_info.burn_time / 2.0;

            if burn_started {
                if should_use_rcs {
                    control.set_throttle(0.0).await?;

                    translation_controller.enabled = true;
                    translation_controller.rcs_enabled = true;
                    translation_controller.reference_frame =
                        Some(node.get_reference_frame().await?);
                    translation_controller.target =
                        TranslationTarget::Acceleration(vec3(0.0, 10.0, 0.0));
                } else if within_margin {
                    control
                        .set_throttle(if remaining_delta_v <= 1.0 {
                            throttle_for_acceleration(&self.vessel, 1.0).await?
                        } else if remaining_delta_v <= 10.0 {
                            throttle_for_acceleration(&self.vessel, 10.0).await?
                        } else {
                            1.0
                        } as f32)
                        .await?;
                    translation_controller.reset(&self.vessel).await?;
                } else {
                    control.set_throttle(0.0).await?;
                    translation_controller.reset(&self.vessel).await?;
                }
            } else {
                control.set_throttle(0.0).await?;
                translation_controller.reset(&self.vessel).await?;
            }

            if !should_use_rcs && remaining_delta_v < 0.1 || remaining_delta_v < 0.05 {
                debug!(
                    "finished executing node with {:.3}m/s leftover Δv.",
                    remaining_delta_v
                );
                break;
            }
        }

        node.remove().await?;
        self.reset().await?;

        Ok(())
    }

    pub async fn execute_next_node(&self) -> Result<bool> {
        match get_next_node(&self.vessel).await? {
            Some(node) => {
                self.execute_node(&node).await?;
                Ok(true)
            }
            None => Ok(false),
        }
    }
}
