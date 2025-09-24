use core::f64;
use std::time::Duration;

use anyhow::Result;
use cgmath::{vec3, Angle, Deg, InnerSpace, Quaternion, Rad, Rotation3, Vector3};
use log::debug;
use tokio::time;

use crate::{
    maneuver::{node_change_orbit_normal, node_change_speed},
    orbital_mechanics::LocalOrbit,
    trajectory_prediction::{
        SurfaceTrajectory, SurfaceTrajectoryKeyframe, TrajectoryPredictionConfig,
    },
    translation::TranslationTarget,
    util::{
        flatten_vector, get_burn_info, surface_position, throttle_for_acceleration,
        vessel_local_orbit,
    },
    FlightComputer,
};

#[derive(Debug, Clone, Copy)]
pub struct LandingDescriptor {
    pub body_position: Vector3<f64>,
    pub margin: f64,
    pub hover_height: f64,
    pub touchdown_speed: f64,
    pub use_rcs: bool,
    pub full_stop_max_tilt: Deg<f64>,
    pub final_hop_max_tilt: Deg<f64>,
    pub hover_max_tilt: Deg<f64>,
    pub final_descent_max_tilt: Deg<f64>,
    pub skip_suicide_burn: bool,
    pub only_kill_vertical_velocity: bool,
    pub suicide_burn_start_factor: f64,
    pub max_hover_approach_speed: f64,
    pub max_stopped_velocity: f64,
    pub perform_orbital_correction: bool,
    pub undershoot_compensation: bool,
}

impl Default for LandingDescriptor {
    fn default() -> Self {
        Self {
            body_position: vec3(0.0, 1.0, 0.0),
            margin: 5.0,
            hover_height: 30.0,
            touchdown_speed: 1.0,
            use_rcs: true,
            full_stop_max_tilt: Deg(75.0),
            final_hop_max_tilt: Deg(30.0),
            hover_max_tilt: Deg(30.0),
            final_descent_max_tilt: Deg(1.0),
            skip_suicide_burn: false,
            only_kill_vertical_velocity: false,
            suicide_burn_start_factor: 1.1,
            max_hover_approach_speed: 500.0,
            max_stopped_velocity: 0.25,
            perform_orbital_correction: true,
            undershoot_compensation: true,
        }
    }
}

impl FlightComputer {
    pub async fn deorbit(
        &self,
        target_position: Vector3<f64>,
        angle_offset: impl Into<Rad<f64>>,
    ) -> Result<()> {
        let angle_offset = angle_offset.into();
        let angle_offset = Rad(angle_offset.0.max(0.0));

        let non_local_orbit = self.vessel.get_orbit().await?;
        let body = non_local_orbit.get_body().await?;
        let body_reference_frame = body.get_reference_frame().await?;
        let non_rotating_reference_frame = body.get_non_rotating_reference_frame().await?;
        let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

        let body_orbit_normal: Vector3<f64> = self
            .space_center
            .transform_direction(
                orbit.normal().into(),
                &non_rotating_reference_frame,
                &body_reference_frame,
            )
            .await?
            .into();

        debug!("beginning deorbit procedure");
        let approx_trajectory = SurfaceTrajectory::predict(
            &self.vessel,
            TrajectoryPredictionConfig {
                look_ahead: orbit.period(),
                time_step: orbit.period() / 100.0,
                ..Default::default()
            },
        )
        .await?;
        if approx_trajectory.impact.is_some() {
            debug!("impact already detected, deorbit canceled");
            return Ok(());
        }

        let target_position = surface_position(&body, target_position).await?;

        let orbit_angle = vec3(0.0, 1.0, 0.0).angle(body_orbit_normal);
        let target_angle = target_position.angle(vec3(target_position.x, 0.0, target_position.z));

        if target_angle - orbit_angle > Rad::from(Deg(1.0)) {
            debug!("changing inclination to align with target...");
            let new_body_orbit_normal = Quaternion::from_axis_angle(
                vec3(0.0, 1.0, 0.0).cross(body_orbit_normal).normalize(),
                (target_angle - orbit_angle).normalize_signed(),
            ) * body_orbit_normal;

            let new_orbit_normal: Vector3<f64> = self
                .space_center
                .transform_direction(
                    new_body_orbit_normal.into(),
                    &body_reference_frame,
                    &non_rotating_reference_frame,
                )
                .await?
                .into();

            self.execute_node(
                &node_change_orbit_normal(
                    &vessel_local_orbit(&self.vessel).await?,
                    new_orbit_normal,
                    true,
                    self.space_center.get_ut().await?,
                )
                .to_node(&self.vessel)
                .await?,
            )
            .await?;
        }

        let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
        let alignment_wait_time = self
            .next_surface_orbit_alignment_delay(&body, target_position, orbit.normal(), Deg(2.5))
            .await?;

        if alignment_wait_time > 0.0 {
            debug!(
                "waiting for proper alignment... ({:.0}s)",
                alignment_wait_time
            );

            let ut = self.space_center.get_ut().await?;
            self.space_center
                .warp_to(ut + alignment_wait_time, f32::INFINITY, f32::INFINITY)
                .await?;
            self.wait_until(ut + alignment_wait_time).await?;
        }

        let current_ut = self.space_center.get_ut().await?;
        let periapsis_ut = orbit.most_recent_periapsis_ut(current_ut);
        let target_position_absolute: Vector3<f64> = self
            .space_center
            .transform_position(
                target_position.into(),
                &body_reference_frame,
                &non_rotating_reference_frame,
            )
            .await?
            .into();
        let deorbit_true_anomaly =
            orbit.true_anomaly_of_position(target_position_absolute) - angle_offset;

        let deorbit_ut =
            periapsis_ut + orbit.true_anomaly_time_delay(deorbit_true_anomaly.normalize());
        let deorbit_ut = if deorbit_ut < current_ut {
            deorbit_ut + orbit.period()
        } else {
            deorbit_ut
        };
        let deorbit_ut = if (deorbit_true_anomaly - orbit.true_anomaly_at_ut(current_ut))
            .normalize_signed()
            .0
            .abs()
            < Rad::from(Deg(1.0)).0
        {
            current_ut
        } else {
            deorbit_ut
        };

        let deorbit_body_position = self
            .body_position_of_absolute_position_at_ut(
                &body,
                orbit.position_at_ut(deorbit_ut),
                deorbit_ut,
            )
            .await?;
        let deorbit_body_velocity = self
            .body_velocity_of_absolute_velocity_at_ut(
                &body,
                orbit.position_at_ut(deorbit_ut),
                orbit.velocity_at_ut(deorbit_ut),
                deorbit_ut,
            )
            .await?;
        let body_orbit_normal = self
            .body_position_of_absolute_position_at_ut(&body, orbit.normal(), deorbit_ut)
            .await?;

        debug!("calculating deorbit burn...");
        let new_speed = {
            let mut start = -deorbit_body_velocity * 2.0;
            let mut end = deorbit_body_velocity * 2.0;
            for _ in 0..15 {
                let middle = (start + end) / 2.0;

                let trajectory = SurfaceTrajectory::predict(
                    &self.vessel,
                    TrajectoryPredictionConfig {
                        look_ahead: orbit.period(),
                        time_step: 1.0,
                        initial_keyframe: Some(SurfaceTrajectoryKeyframe {
                            ut: deorbit_ut,
                            position: deorbit_body_position,
                            velocity: middle,
                        }),
                        ..Default::default()
                    },
                )
                .await?;

                if let Some(impact) = trajectory.impact {
                    if (impact.position - target_position)
                        .dot(deorbit_body_position.cross(body_orbit_normal))
                        .is_sign_positive()
                    {
                        end = middle;
                    } else {
                        start = middle;
                    }
                } else {
                    end = middle;
                }
            }

            let new_velocity: Vector3<f64> = self
                .space_center
                .transform_velocity(
                    deorbit_body_position.into(),
                    ((start + end) / 2.0).into(),
                    &body_reference_frame,
                    &non_rotating_reference_frame,
                )
                .await?
                .into();
            new_velocity.magnitude()
        };

        self.execute_node(
            &node_change_speed(
                &vessel_local_orbit(&self.vessel).await?,
                deorbit_ut,
                new_speed,
            )
            .to_node(&self.vessel)
            .await?,
        )
        .await?;

        debug!("deorbit procedure finished");

        Ok(())
    }

    pub async fn land(&self, descriptor: LandingDescriptor) -> Result<()> {
        let margin = descriptor.margin.max(0.25);

        let non_local_orbit = self.vessel.get_orbit().await?;
        let body = non_local_orbit.get_body().await?;
        let vessel_reference_frame = self.vessel.get_reference_frame().await?;
        let body_reference_frame = body.get_reference_frame().await?;
        let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

        debug!("beginning landing procedure");

        {
            // initialize translation controller
            let mut translation_controller = self.translation_controller.lock().await;
            translation_controller.reset(&self.vessel).await?;
            translation_controller.reference_frame = Some(body.get_reference_frame().await?);
            translation_controller.rcs_enabled = descriptor.use_rcs;
        }

        // we'll need this to find the offset of the vessel's center of mass with its lowest point
        let lowest_part = {
            let mut parts = self.vessel.get_parts().await?.get_all().await?;

            if parts.is_empty() {
                debug!("the vessel has no parts???");
                return Ok(());
            }

            let mut y_min = f64::INFINITY;
            let mut lowest_part = parts.pop().unwrap();
            for part in parts.into_iter() {
                let y = part.position(&vessel_reference_frame).await?.1;
                if y < y_min {
                    y_min = y;
                    lowest_part = part;
                }
            }

            lowest_part
        };

        // the provided target position projected onto the surface
        // we need to store the original separately because undershoot compensation changes target_surface_position until
        // the velocity is killed
        let original_target_surface_position =
            surface_position(&body, descriptor.body_position).await?;
        let mut target_surface_position = original_target_surface_position;
        let mut overshoot_applied = !descriptor.undershoot_compensation;

        let control = self.vessel.get_control().await?;
        let auto_pilot = self.vessel.get_auto_pilot().await?;
        auto_pilot
            .set_reference_frame(&body_reference_frame)
            .await?;
        auto_pilot.set_roll_threshold(0.0).await?;
        auto_pilot.engage().await?;

        // used to detect when we've touched down
        let mut last_time_not_stationary = self.space_center.get_ut().await?;

        let mut impact_adjustment_active =
            descriptor.perform_orbital_correction && !descriptor.skip_suicide_burn;
        let mut full_stop_started = descriptor.skip_suicide_burn;
        let mut full_stop_complete = descriptor.skip_suicide_burn;
        let mut final_hop_complete = false;
        let mut final_descent_active = false;

        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 10.0));
        loop {
            interval.tick().await;
            let current_ut = self.space_center.get_ut().await?;

            // we just add 1 to the offset to hopefully increase the accuracy of the approximation
            let vessel_com_offset = -lowest_part.position(&vessel_reference_frame).await?.1 + 1.0;
            let target_position =
                target_surface_position + target_surface_position.normalize_to(vessel_com_offset);

            // start with a rough trajectory spanning the entire orbital period to check if there's an impact
            let approx_trajectory = SurfaceTrajectory::predict(
                &self.vessel,
                TrajectoryPredictionConfig {
                    look_ahead: orbit.period(),
                    time_step: orbit.period() / 100.0,
                    ..Default::default()
                },
            )
            .await?;

            if let Some(approx_impact) = approx_trajectory.impact {
                // once we know the impact exists, we can use a more precise trajectory without worrying about
                // wasting RPCs
                let step_count = 300.0;
                let trajectory = SurfaceTrajectory::predict(
                    &self.vessel,
                    TrajectoryPredictionConfig {
                        look_ahead: orbit.period(),
                        time_step: (approx_impact.ut - current_ut) / step_count,
                        ..Default::default()
                    },
                )
                .await?;

                if let Some(impact) = trajectory.impact {
                    let impact_position =
                        impact.position + impact.position.normalize_to(vessel_com_offset);
                    let impact_in = (impact.ut - current_ut)
                        - vessel_com_offset / impact.velocity.dot(impact.position.normalize());
                    let impact_error =
                        flatten_vector(target_position - impact_position, impact_position);

                    if impact_in < 0.0 {
                        continue;
                    }

                    let current_position: Vector3<f64> =
                        self.vessel.position(&body_reference_frame).await?.into();
                    let velocity: Vector3<f64> =
                        self.vessel.velocity(&body_reference_frame).await?.into();

                    if velocity.magnitude() > 0.05 {
                        last_time_not_stationary = current_ut;
                    }

                    // apply an offset to the target impact position to compensate for potential
                    // undershooting from the suicide burn
                    if !overshoot_applied {
                        let final_suicide_burn_info =
                            get_burn_info(&self.vessel, impact.velocity.magnitude(), None).await?;
                        if final_suicide_burn_info.distance_covered.is_finite() {
                            let expected_stop_position = impact.position
                                - impact
                                    .velocity
                                    .normalize_to(final_suicide_burn_info.distance_covered);

                            let offset = (surface_position(&body, expected_stop_position).await?
                                - impact.position)
                                .magnitude();
                            target_surface_position = surface_position(
                                &body,
                                target_surface_position
                                    + (target_surface_position
                                        - surface_position(&body, current_position).await?)
                                        .normalize_to(offset),
                            )
                            .await?;
                        }

                        overshoot_applied = true;
                    }

                    let suicide_burn_info =
                        get_burn_info(&self.vessel, velocity.magnitude(), None).await?;

                    // we have to kill the velocity when either the time is right or we're already close enough
                    if suicide_burn_info.burn_time * descriptor.suicide_burn_start_factor
                        >= impact_in
                        || ((current_position - target_position).magnitude()
                            < (margin * 10.0).max(100.0)
                            && velocity.magnitude() < 50.0)
                    {
                        if !full_stop_started {
                            debug!("killing velocity...");
                        }
                        target_surface_position = original_target_surface_position;
                        full_stop_started = true;
                        impact_adjustment_active = false;
                    }

                    if impact_adjustment_active {
                        if impact_error.magnitude() < (margin * 3.0).max(50.0) {
                            debug!("finished initial impact adjustment. coasting...");
                            impact_adjustment_active = false;
                            control.set_throttle(0.0).await?;
                        } else {
                            // orbital adjustment of impact point

                            auto_pilot
                                .set_target_direction(impact_error.normalize().into())
                                .await?;

                            let throttle = if impact_error.magnitude() > 5000.0 {
                                1.0
                            } else if impact_error.magnitude() > 1000.0 {
                                throttle_for_acceleration(&self.vessel, 10.0).await?
                            } else {
                                throttle_for_acceleration(&self.vessel, 1.0).await?
                            };

                            let direction: Vector3<f64> =
                                self.vessel.direction(&body_reference_frame).await?.into();
                            let direction_error = direction.angle(impact_error);

                            control
                                .set_throttle(if direction_error < Rad::from(Deg(2.0)) {
                                    throttle
                                } else {
                                    0.0
                                } as f32)
                                .await?;
                        }
                    }

                    {
                        let mut translation_controller = self.translation_controller.lock().await;

                        let speed_to_kill = if descriptor.only_kill_vertical_velocity {
                            velocity.dot(current_position.normalize()).abs()
                        } else {
                            velocity.magnitude()
                        };
                        if speed_to_kill < descriptor.max_stopped_velocity {
                            if !full_stop_complete {
                                debug!("velocity killed, performing final hop...")
                            }
                            full_stop_complete = true;
                        }

                        if full_stop_complete {
                            translation_controller.vtol_enabled = true;
                            if impact_error.magnitude() <= margin {
                                if !final_hop_complete {
                                    debug!("final hop complete");
                                }
                                final_hop_complete = true;
                            }

                            if final_hop_complete {
                                let target_hover_position = target_position
                                    + target_position.normalize_to(descriptor.hover_height);
                                if velocity.magnitude() <= descriptor.max_stopped_velocity
                                    && (current_position - target_hover_position).magnitude()
                                        <= margin
                                {
                                    if !final_descent_active {
                                        debug!("beginning final descent...")
                                    }
                                    final_descent_active = true;
                                }

                                if final_descent_active {
                                    translation_controller.vtol_max_tilt =
                                        descriptor.final_descent_max_tilt;
                                    let altitude_diff =
                                        current_position.magnitude() - impact.position.magnitude();
                                    translation_controller.target = TranslationTarget::Velocity(
                                        -current_position.normalize_to(
                                            descriptor.touchdown_speed + altitude_diff / 5.0,
                                        ),
                                    );

                                    if altitude_diff < 0.5
                                        || (altitude_diff < 20.0
                                            && current_ut - last_time_not_stationary >= 0.5)
                                    {
                                        break;
                                    }
                                } else {
                                    translation_controller.vtol_max_tilt =
                                        descriptor.hover_max_tilt;
                                    translation_controller.target = TranslationTarget::Position {
                                        pos: target_hover_position,
                                        max_speed: descriptor.max_hover_approach_speed,
                                        margin,
                                    };
                                }
                            } else {
                                translation_controller.vtol_max_tilt =
                                    descriptor.final_hop_max_tilt;
                                translation_controller.target = TranslationTarget::Acceleration(
                                    impact_error.normalize_to(impact_error.magnitude().min(100.0)),
                                );
                            }
                        } else if full_stop_started {
                            translation_controller.vtol_enabled = true;
                            translation_controller.vtol_max_tilt = descriptor.full_stop_max_tilt;
                            translation_controller.target =
                                TranslationTarget::Velocity(vec3(0.0, 0.0, 0.0));
                        } else {
                            translation_controller.target = TranslationTarget::Acceleration(
                                if impact_error.magnitude() < margin {
                                    vec3(0.0, 0.0, 0.0)
                                } else {
                                    Quaternion::from_arc(
                                        impact.velocity.normalize(),
                                        velocity.normalize(),
                                        None,
                                    ) * impact_error
                                        / 500.0
                                },
                            )
                        }

                        translation_controller.enabled = true;
                        if !full_stop_started && !impact_adjustment_active {
                            auto_pilot
                                .set_target_direction((-velocity.normalize()).into())
                                .await?;
                        }
                    }
                } else {
                    debug!("impact in approximate trajectory but no impact in precise trajectory, canceling landing");
                    break;
                }
            } else {
                debug!("no impact, canceling landing");
                break;
            }
        }

        self.reset().await?;
        debug!("landing finished");

        Ok(())
    }
}
