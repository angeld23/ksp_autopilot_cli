use core::f32;
use std::time::Duration;

use anyhow::Result;
use cgmath::{vec3, Deg, ElementWise, InnerSpace, Quaternion, Rad, Rotation3, Vector3, Zero};
use derive_more::*;
use krpc_client::services::space_center::ReferenceFrame;
use log::debug;
use tokio::time;

use crate::{
    maneuver::{node_change_velocity, node_circularize},
    util::{
        flatten_vector, get_circular_orbit_speed, get_orbit_normal,
        get_relative_ascending_node_direction, get_relative_descending_node_direction,
        is_below_plane,
    },
    FlightComputer,
};

#[derive(IsVariant, Unwrap)]
pub enum InclinationTarget {
    Inclination(Deg<f64>),
    OrbitNormal(Vector3<f64>, ReferenceFrame),
}

impl Default for InclinationTarget {
    fn default() -> Self {
        Self::Inclination(Deg(0.0))
    }
}

pub struct AscentDescriptor {
    pub altitude: f64,
    pub turn_start_height: f64,
    pub inclination_target: InclinationTarget,
    pub max_prograde_error: Deg<f64>,
    pub circularize: bool,
    pub correct_lobsided_orbit: bool,
}

impl Default for AscentDescriptor {
    fn default() -> Self {
        Self {
            altitude: 80000.0,
            turn_start_height: 200.0,
            inclination_target: Default::default(),
            max_prograde_error: Deg(15.0),
            circularize: true,
            correct_lobsided_orbit: true,
        }
    }
}

impl FlightComputer {
    /// ascent auto sequence
    pub async fn ascent(&self, descriptor: AscentDescriptor) -> Result<()> {
        let control = self.vessel.get_control().await?;
        let auto_pilot = self.vessel.get_auto_pilot().await?;

        let orbit = self.vessel.get_orbit().await?;
        let body = orbit.get_body().await?;
        let body_reference_frame = body.get_reference_frame().await?;

        let flight = self.vessel.flight(Some(&body_reference_frame)).await?;
        let atmosphere_height = body.get_atmosphere_depth().await?;

        let position: Vector3<f64> = self.vessel.position(&body_reference_frame).await?.into();
        let orbit_normal: Vector3<f64> = match descriptor.inclination_target {
            InclinationTarget::Inclination(inclination) => {
                Quaternion::from_axis_angle(
                    position.mul_element_wise(vec3(1.0, 0.0, 1.0)).normalize(),
                    -inclination,
                ) * vec3(0.0, 1.0, 0.0)
            }
            InclinationTarget::OrbitNormal(normal, reference_frame) => self
                .space_center
                .transform_direction(normal.into(), &reference_frame, &body_reference_frame)
                .await?
                .into(),
        };
        let orbit_normal = if orbit_normal.magnitude() > 0.0 {
            orbit_normal.normalize()
        } else {
            vec3(0.0, 1.0, 0.0)
        };

        // wait for alignment
        let normalized_position = position.normalize();

        let ascending_node_direction =
            get_relative_ascending_node_direction(vec3(0.0, 1.0, 0.0), orbit_normal);
        let descending_node_direction =
            get_relative_descending_node_direction(vec3(0.0, 1.0, 0.0), orbit_normal);

        let ascending_angle = ascending_node_direction.angle(normalized_position);
        let descending_angle = descending_node_direction.angle(normalized_position);

        let angle_tolerance = Rad::from(Deg(1.0));
        if orbit_normal.angle(vec3(0.0, 1.0, 0.0)) > angle_tolerance
            && Rad(ascending_angle.0.min(descending_angle.0)) > angle_tolerance
        {
            let is_before_ascending = normalized_position
                .cross(ascending_node_direction)
                .y
                .is_sign_negative();
            let angle = if is_before_ascending {
                ascending_angle
            } else {
                descending_angle
            };
            let wait_time = angle / Rad(body.get_rotational_speed().await?);

            debug!("waiting for proper alignment... ({:.0}s)", wait_time);

            let ut = self.space_center.get_ut().await?;
            self.space_center
                .warp_to(ut + wait_time, f32::INFINITY, f32::INFINITY)
                .await?;
            self.wait_until(ut + wait_time).await?;
        }

        debug!(
            "beginning ascent to target altitude of {}m.",
            descriptor.altitude
        );

        self.save_control_config().await?;

        auto_pilot
            .set_reference_frame(&body_reference_frame)
            .await?;
        auto_pilot.set_roll_threshold(0.0).await?;
        auto_pilot.engage().await?;

        let mut atmosphere_warp_active = false;

        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 30.0));
        loop {
            interval.tick().await;

            let position: Vector3<f64> = self.vessel.position(&body_reference_frame).await?.into();
            let rotation: Quaternion<f64> = flight.get_rotation().await?.into();
            let forward = rotation * vec3(0.0, 1.0, 0.0);

            let altitude = flight.get_surface_altitude().await?;
            let surface_velocity: Vector3<f64> = flight.get_velocity().await?.into();
            let surface_prograde = if surface_velocity.magnitude() > 1.0 {
                surface_velocity.normalize()
            } else {
                forward
            };

            let apoapsis = orbit.get_apoapsis_altitude().await?;
            let target_turn_angle = if altitude < descriptor.turn_start_height {
                Deg(0.0)
            } else {
                Deg(90.0) * (apoapsis / descriptor.altitude).clamp(0.0, 1.0)
            };

            let current_orbit_normal = get_orbit_normal(&orbit, &body_reference_frame).await?;
            let orbit_normal_error_rotation =
                Quaternion::from_arc(current_orbit_normal, orbit_normal, None);

            let target_rotation = Quaternion::from_axis_angle(
                orbit_normal_error_rotation * orbit_normal,
                -target_turn_angle,
            );
            let target_direction = target_rotation * position.normalize();

            let prograde_error: Deg<f64> = surface_prograde.angle(target_direction).into();
            let clamped_direction = if prograde_error <= descriptor.max_prograde_error {
                target_direction
            } else {
                let prograde_offset_rotation = Quaternion::from_axis_angle(
                    surface_prograde.cross(target_direction).normalize(),
                    descriptor.max_prograde_error,
                );
                prograde_offset_rotation * surface_prograde
            };

            auto_pilot
                .set_target_direction(
                    (if altitude < descriptor.turn_start_height {
                        position.normalize()
                    } else {
                        clamped_direction
                    })
                    .into(),
                )
                .await?;

            let throttle = if apoapsis < descriptor.altitude {
                1.0
            } else {
                0.0
            };
            control.set_throttle(throttle as f32).await?;

            if apoapsis >= descriptor.altitude {
                if !atmosphere_warp_active && altitude <= atmosphere_height {
                    atmosphere_warp_active = true;
                    self.space_center.set_physics_warp_factor(3).await?;
                } else if atmosphere_warp_active && altitude > atmosphere_height {
                    atmosphere_warp_active = false;
                    self.space_center.set_physics_warp_factor(0).await?;
                }

                if altitude > atmosphere_height {
                    break;
                }
            }
        }

        debug!("target apoapsis reached, out of atmosphere.");

        self.reset().await?;

        if descriptor.circularize {
            debug!("circularizing...");

            let ut = self.space_center.get_ut().await?;
            let time_to_apoapsis = orbit.get_time_to_apoapsis().await?;
            let apoapsis_radius = orbit.get_apoapsis().await?;
            let desired_speed = get_circular_orbit_speed(&body, apoapsis_radius).await?;
            let apoapsis_position: Vector3<f64> = orbit
                .position_at(ut + time_to_apoapsis, &body_reference_frame)
                .await?
                .into();

            self.execute_node(
                &node_change_velocity(
                    &self.vessel,
                    ut + time_to_apoapsis,
                    apoapsis_position
                        .cross(orbit_normal)
                        .normalize_to(desired_speed),
                    Some(&body_reference_frame),
                )
                .await?,
            )
            .await?;

            if descriptor.correct_lobsided_orbit {
                let apoapsis = orbit.get_apoapsis_altitude().await?;
                let periapsis = orbit.get_periapsis_altitude().await?;

                if (apoapsis - periapsis).abs() > 500.0 {
                    debug!("|apo - peri| > 500. correcting lobsided orbit...");
                    self.execute_node(
                        &node_circularize(&self.vessel, self.space_center.get_ut().await? + 30.0)
                            .await?,
                    )
                    .await?;
                }
            }
        }

        debug!("ascent complete.");

        Ok(())
    }
}
