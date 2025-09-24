use anyhow::Result;
use cgmath::{vec3, Angle, Deg, InnerSpace, Quaternion, Rad, Rotation3, Vector3, Zero};
use derive_more::*;
use krpc_client::services::space_center::{ReferenceFrame, Vessel};

use crate::{
    reference_frame_or_vessel_frame,
    util::{
        flatten_vector, get_thrust_info, get_translation_force, gravitational_acceleration,
        throttle_for_acceleration, ThrustInfo,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, IsVariant)]
pub enum TranslationTarget {
    Position {
        pos: Vector3<f64>,
        max_speed: f64,
        margin: f64,
    },
    Velocity(Vector3<f64>),
    Acceleration(Vector3<f64>),
}

pub struct TranslationController {
    pub target: TranslationTarget,
    pub reference_frame: Option<ReferenceFrame>,
    pub vtol_enabled: bool,
    pub vtol_max_tilt: Deg<f64>,
    pub rcs_enabled: bool,
    pub enabled: bool,
}

impl TranslationController {
    pub async fn update(&self, vessel: &Vessel) -> Result<()> {
        if !self.enabled {
            return Ok(());
        }

        let control = vessel.get_control().await?;
        let reference_frame = reference_frame_or_vessel_frame!(vessel, self.reference_frame);

        let rotation: Quaternion<f64> = vessel.rotation(reference_frame).await?.into();
        let forward_direction = rotation * vec3(0.0, 1.0, 0.0);
        let right_direction = rotation * vec3(1.0, 0.0, 0.0);
        let up_direction = rotation * vec3(0.0, 0.0, -1.0);

        let position: Vector3<f64> = vessel.position(reference_frame).await?.into();
        let velocity: Vector3<f64> = vessel.velocity(reference_frame).await?.into();

        let throttle = if !self.rcs_enabled {
            vec3(0.0, 0.0, 0.0)
        } else {
            match self.target {
                TranslationTarget::Position {
                    pos: target_position,
                    max_speed,
                    margin,
                } => {
                    let rcs_force = get_translation_force(
                        vessel,
                        target_position - position,
                        Some(reference_frame),
                    )
                    .await?;

                    let rcs_acceleration = rcs_force.abs() / vessel.get_mass().await? as f64 * 0.9;
                    let distance = (target_position - position).magnitude();

                    let target_speed = (2.0 * rcs_acceleration * distance).sqrt().min(max_speed);
                    let target_speed = if distance > margin { target_speed } else { 0.0 };

                    if distance > 50.0 && self.vtol_enabled {
                        // to not waste monoprop
                        vec3(0.0, 0.0, 0.0)
                    } else {
                        (target_position - position).normalize_to(target_speed) - velocity
                    }
                }
                TranslationTarget::Velocity(target_velocity) => {
                    let diff = target_velocity - velocity;
                    if self.vtol_enabled && diff.magnitude() > 50.0 {
                        vec3(0.0, 0.0, 0.0)
                    } else {
                        diff
                    }
                }
                TranslationTarget::Acceleration(acceleration) => {
                    if acceleration.is_zero() {
                        acceleration
                    } else {
                        let rcs_force = get_translation_force(
                            vessel,
                            acceleration.normalize(),
                            Some(reference_frame),
                        )
                        .await?;

                        acceleration * vessel.get_mass().await? as f64 / rcs_force
                    }
                }
            }
        };

        if self.vtol_enabled {
            let ThrustInfo {
                thrust,
                vessel_mass,
                ..
            } = get_thrust_info(vessel, None).await?;

            let auto_pilot = vessel.get_auto_pilot().await?;
            auto_pilot.set_reference_frame(reference_frame).await?;
            auto_pilot.set_target_roll(f32::NAN).await?;
            auto_pilot.engage().await?;

            let body = vessel.get_orbit().await?.get_body().await?;
            let position_of_body: Vector3<f64> = body.position(reference_frame).await?.into();
            let up = (position - position_of_body).normalize();
            let direction: Vector3<f64> = vessel.direction(reference_frame).await?.into();
            let mu = body.get_gravitational_parameter().await?;
            // doesn't account for centrifugal acceleration but we're dealing with an arbitrary reference frame so it'd be a headache
            let g_acceleration = gravitational_acceleration(mu, position - position_of_body);
            let velocity: Vector3<f64> = vessel.velocity(reference_frame).await?.into();
            let vertical_speed = velocity.dot(up);
            let horizontal_velocity = flatten_vector(velocity, up);
            let max_upwards_acceleration = direction.dot(up) * thrust / vessel_mass;

            let using_rcs = self.rcs_enabled && control.get_rcs().await?;

            let target_acceleration = match self.target {
                TranslationTarget::Position {
                    pos: target_position,
                    max_speed,
                    margin,
                } => {
                    let vertical_difference = (target_position - position_of_body).magnitude()
                        - (position - position_of_body).magnitude();
                    let horizontal_difference = flatten_vector(target_position - position, up);
                    let horizontal_distance = horizontal_difference.magnitude();

                    let max_vertical_approach_deceleration =
                        if vertical_difference.is_sign_negative() {
                            max_upwards_acceleration - g_acceleration.magnitude()
                        } else {
                            g_acceleration.magnitude()
                        } * 0.5;
                    let target_vertical_speed = vertical_difference.signum()
                        * (2.0 * max_vertical_approach_deceleration * vertical_difference.abs())
                            .sqrt()
                            .min(max_speed);
                    let expected_vertical_proper_acceleration =
                        target_vertical_speed - vertical_speed + g_acceleration.magnitude();

                    let max_horizontal_approach_deceleration =
                        (self.vtol_max_tilt / 8.0).tan() * expected_vertical_proper_acceleration;
                    let target_horizontal_speed = if expected_vertical_proper_acceleration < 0.001 {
                        0.0
                    } else {
                        (2.0 * max_horizontal_approach_deceleration * horizontal_distance)
                            .sqrt()
                            .min(max_speed)
                    };

                    let target_horizontal_velocity =
                        horizontal_difference.normalize_to(target_horizontal_speed);
                    let horizontal_acceleration = target_horizontal_velocity - horizontal_velocity;
                    let horizontal_acceleration =
                        if horizontal_velocity.magnitude() < 1.0 && using_rcs {
                            vec3(0.0, 0.0, 0.0)
                        } else {
                            horizontal_acceleration
                        };

                    let vertical_acceleration = up
                        * (target_vertical_speed - vertical_speed)
                        // * if velocity.magnitude() > 50.0 {
                        //     10.0
                        // } else {
                        //     1.0
                        // };
                        * velocity.magnitude().clamp(1.0, 50.0);

                    if (position - target_position).magnitude() > margin {
                        vertical_acceleration + horizontal_acceleration
                    } else {
                        -velocity
                    }
                }
                TranslationTarget::Velocity(target_velocity) => target_velocity - velocity,
                TranslationTarget::Acceleration(acceleration) => acceleration,
            };

            let target_proper_acceleration = target_acceleration - g_acceleration;
            let target_vertical_acceleration = target_proper_acceleration.dot(up);

            control
                .set_throttle(
                    throttle_for_acceleration(vessel, target_vertical_acceleration).await? as f32,
                )
                .await?;

            let tilt = Rad(target_proper_acceleration
                .angle(up)
                .0
                .min(Rad::from(self.vtol_max_tilt).0));
            let target_direction = if tilt.0 < 0.0000001 {
                up
            } else {
                Quaternion::from_axis_angle(up.cross(target_proper_acceleration).normalize(), tilt)
                    * up
            };

            auto_pilot
                .set_target_direction(target_direction.into())
                .await?;
        }

        control
            .set_forward(throttle.dot(forward_direction) as f32)
            .await?;
        control
            .set_right(throttle.dot(right_direction) as f32)
            .await?;
        control.set_up(throttle.dot(up_direction) as f32).await?;

        Ok(())
    }

    pub fn reset_target(&mut self) {
        self.target = TranslationTarget::Acceleration(Vector3::zero());
    }

    pub fn target_is_reset(&self) -> bool {
        self.target == TranslationTarget::Acceleration(Vector3::zero())
    }

    pub async fn reset(&mut self, vessel: &Vessel) -> Result<()> {
        if self.enabled {
            let control = vessel.get_control().await?;
            control.set_forward(0.0).await?;
            control.set_right(0.0).await?;
            control.set_up(0.0).await?;
            control.set_throttle(0.0).await?;

            if self.vtol_enabled {
                let auto_pilot = vessel.get_auto_pilot().await?;
                auto_pilot.disengage().await?;
            }
        }

        self.enabled = false;
        self.rcs_enabled = false;
        self.vtol_enabled = false;
        self.vtol_max_tilt = Deg(30.0);
        self.target = TranslationTarget::Acceleration(Vector3::zero());
        self.reference_frame = None;

        Ok(())
    }
}
