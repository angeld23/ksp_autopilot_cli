use anyhow::Result;
use cgmath::{vec3, InnerSpace, Quaternion, Vector3, Zero};
use krpc_client::services::space_center::Vessel;

use crate::{
    numerical_integration::runge_kutta_step,
    util::{
        get_current_ut_from_orbit, get_thrust_info, height_above_surface_at_position,
        surface_acceleration, ThrustInfo,
    },
};

#[derive(Debug, Clone, Copy)]
pub struct SurfaceTrajectoryKeyframe {
    pub ut: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

#[derive(Debug, Default, Clone)]
pub struct SurfaceTrajectory {
    pub keyframes: Vec<SurfaceTrajectoryKeyframe>,
    pub impact: Option<SurfaceTrajectoryKeyframe>,
}

#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPredictionConfig {
    pub look_ahead: f64,
    pub time_step: f64,
    pub direction_of_attack: Vector3<f64>,
    pub initial_keyframe: Option<SurfaceTrajectoryKeyframe>,
}

impl Default for TrajectoryPredictionConfig {
    fn default() -> Self {
        Self {
            look_ahead: 100.0,
            time_step: 1.0,
            direction_of_attack: vec3(0.0, -1.0, 0.0),
            initial_keyframe: None,
        }
    }
}

impl SurfaceTrajectory {
    pub async fn predict(vessel: &Vessel, config: TrajectoryPredictionConfig) -> Result<Self> {
        let TrajectoryPredictionConfig {
            look_ahead,
            time_step,
            direction_of_attack,
            initial_keyframe,
        } = config;

        let time_step = time_step.max(1.0 / 60.0);
        let direction_of_attack = if direction_of_attack.magnitude() > 0.0 {
            direction_of_attack.normalize()
        } else {
            vec3(0.0, -1.0, 0.0)
        };

        let orbit = vessel.get_orbit().await?;
        let body = orbit.get_body().await?;
        let mu = body.get_gravitational_parameter().await?;
        let radius = body.get_equatorial_radius().await?;
        let body_reference_frame = body.get_reference_frame().await?;
        let non_rotating_reference_frame = body.get_non_rotating_reference_frame().await?;
        let flight = vessel.flight(Some(&body_reference_frame)).await?;
        let mass = vessel.get_mass().await? as f64;
        let rotation: Quaternion<f64> = flight.get_rotation().await?.into();
        let angular_velocity: Vector3<f64> = body
            .angular_velocity(&non_rotating_reference_frame)
            .await?
            .into();

        let air_density_asl = body
            .atmospheric_density_at_position((radius, 0.0, 0.0), &body_reference_frame)
            .await?;
        let aero_force_asl: Vector3<f64> = flight
            .simulate_aerodynamic_force_at(
                &body,
                (radius, 0.0, 0.0),
                (rotation * direction_of_attack).into(),
            )
            .await?
            .into();

        let aero_base_vector = if air_density_asl.is_zero() {
            vec3(0.0, 0.0, 0.0)
        } else {
            Quaternion::from_arc(rotation * direction_of_attack, vec3(0.0, 1.0, 0.0), None)
                * aero_force_asl
                / air_density_asl
        }; // this equals F/(pv^2) for all air densities and velocities

        let current_ut = if let Some(init) = initial_keyframe {
            init.ut
        } else {
            get_current_ut_from_orbit(&orbit).await?
        };

        let steps = (look_ahead / time_step).floor() as usize;
        let mut keyframes = Vec::<SurfaceTrajectoryKeyframe>::with_capacity(steps + 1);
        keyframes.push(initial_keyframe.unwrap_or(SurfaceTrajectoryKeyframe {
            ut: current_ut,
            position: vessel.position(&body_reference_frame).await?.into(),
            velocity: vessel.velocity(&body_reference_frame).await?.into(),
        }));

        // to save RPCs, just pre-build a table of air densities for different altitudes
        let density_table_step = 250.0;
        let mut density_table = Vec::<f64>::with_capacity(
            (body.get_atmosphere_depth().await? / density_table_step).ceil() as usize,
        );
        let mut density_table_altitude = radius;
        loop {
            let density = body.density_at(density_table_altitude).await?;
            density_table.push(density);
            if density < 0.00001 {
                break;
            }

            density_table_altitude += density_table_step;
        }

        'predict_steps: for i in 1..(steps + 1) {
            let ut = current_ut + time_step * i as f64;
            let prev_keyframe = keyframes.last().unwrap();

            let altitude = (prev_keyframe.position.magnitude() - radius).max(0.0);
            let air_density = if density_table.is_empty() {
                0.0
            } else {
                let first_index = (altitude / density_table_step).floor() as usize;
                let t = (altitude % density_table_step) / density_table_step;
                let first_value = *density_table
                    .get(first_index)
                    .unwrap_or_else(|| density_table.last().unwrap());
                let second_value = *density_table
                    .get(first_index + 1)
                    .unwrap_or_else(|| density_table.last().unwrap());

                first_value + t * (second_value - first_value)
            };

            let g_acceleration = surface_acceleration(mu, angular_velocity, prev_keyframe.position);

            let rotated_aero_base_vector = Quaternion::from_arc(
                vec3(0.0, 1.0, 0.0),
                prev_keyframe.velocity.normalize(),
                None,
            ) * aero_base_vector;

            let acceleration = |_: f64, velocity: Vector3<f64>| {
                g_acceleration
                    + rotated_aero_base_vector * air_density * velocity.magnitude2() / mass
            };

            let expected_velocity =
                runge_kutta_step(prev_keyframe.velocity, 0.0, time_step, acceleration);
            let expected_position = prev_keyframe.position
                + time_step * (prev_keyframe.velocity + expected_velocity) / 2.0;

            if expected_position.magnitude() - radius < -3000.0 {
                break 'predict_steps;
            }

            keyframes.push(SurfaceTrajectoryKeyframe {
                ut,
                position: expected_position,
                velocity: expected_velocity,
            });
        }

        let mut impact: Option<SurfaceTrajectoryKeyframe> = None;

        // find impact
        let last = keyframes.last().unwrap();
        let last_height =
            height_above_surface_at_position(&body, last.position, &body_reference_frame).await?;
        if last_height <= 0.0 {
            // we're basically using newton's method here
            let iterations = 5;
            let mut guess_ut = last.ut - last_height / last.velocity.dot(last.position.normalize());
            for i in 0..=iterations {
                let mut before_keyframe: Option<SurfaceTrajectoryKeyframe> = None;
                let mut after_keyframe: Option<SurfaceTrajectoryKeyframe> = None;
                for keyframe in keyframes.iter().copied() {
                    if keyframe.ut <= guess_ut {
                        before_keyframe = Some(keyframe);
                    } else {
                        after_keyframe = Some(keyframe);
                        break;
                    }
                }

                let (position, velocity) = match (before_keyframe, after_keyframe) {
                    (None, None) => unreachable!(),
                    (None, Some(after)) => (
                        after.position + after.velocity * (guess_ut - after.ut),
                        after.velocity,
                    ),
                    (Some(before), None) => (
                        before.position + before.velocity * (guess_ut - before.ut),
                        before.velocity,
                    ),
                    (Some(before), Some(after)) => (
                        before.position
                            + (after.position - before.position) * (guess_ut - before.ut)
                                / (after.ut - before.ut),
                        before.velocity
                            + (after.velocity - before.velocity) * (guess_ut - before.ut)
                                / (after.ut - before.ut),
                    ),
                };

                if i == iterations {
                    impact = Some(SurfaceTrajectoryKeyframe {
                        ut: guess_ut,
                        position,
                        velocity,
                    });
                    break;
                }

                let height =
                    height_above_surface_at_position(&body, position, &body_reference_frame)
                        .await?;
                guess_ut -= height / velocity.dot(position.normalize());
            }
        }

        Ok(Self { keyframes, impact })
    }
}
