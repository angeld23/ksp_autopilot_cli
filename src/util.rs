use core::f64;

use anyhow::Result;
use cgmath::{vec3, Angle, InnerSpace, Quaternion, Rad, Rotation, Rotation3, Vector3, Zero};
use krpc_client::services::space_center::{
    CelestialBody, Node, Orbit, ReferenceFrame, SpaceCenter, Vessel,
};
use log::warn;

use crate::{root_finding::newton_find_all_roots, FlightComputer};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustInfo {
    pub thrust: f64,
    pub mass_flow: f64,
    pub vessel_mass: f64,
}

pub async fn get_thrust_info(vessel: &Vessel, pressure: Option<f64>) -> Result<ThrustInfo> {
    let reference_frame = vessel.get_reference_frame().await?;
    let parts = vessel.get_parts().await?;
    let engines = parts.get_engines().await?;

    let pressure =
        pressure.unwrap_or(vessel.flight(None).await?.get_static_pressure().await? as f64);
    let pressure_atm = pressure / 101325.027;

    let vessel_direction: Vector3<f64> = vessel.direction(&reference_frame).await?.into();

    let mut total_thrust = 0f64;
    let mut total_mass_flow = 0f64;
    for engine in engines {
        if engine.get_active().await? {
            let thrust = engine.available_thrust_at(pressure_atm).await? as f64;
            if thrust.is_zero() {
                continue;
            }

            let thrusters = engine.get_thrusters().await?;

            let mut added_thrust = 0f64;
            let mut thruster_count = 0;
            for thruster in thrusters {
                if let Ok(dir) = thruster.initial_thrust_direction(&reference_frame).await {
                    let thrust_direction: Vector3<f64> = dir.into();
                    added_thrust += thrust_direction.dot(vessel_direction) * thrust;
                    thruster_count += 1;
                }
            }

            if thruster_count >= 1 {
                total_thrust += added_thrust / (thruster_count as f64);
            } else {
                let part_direction: Vector3<f64> = engine
                    .get_part()
                    .await?
                    .direction(&reference_frame)
                    .await?
                    .into();
                total_thrust += part_direction.dot(vessel_direction) * thrust;
            }

            let isp = engine.specific_impulse_at(pressure_atm).await? as f64;
            total_mass_flow += thrust / (isp * 9.81);
        }
    }

    Ok(ThrustInfo {
        thrust: total_thrust,
        mass_flow: total_mass_flow,
        vessel_mass: vessel.get_mass().await? as f64,
    })
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BurnInfo {
    pub burn_time: f64,
    pub distance_covered: f64,
}

pub async fn get_burn_info(
    vessel: &Vessel,
    delta_v: f64,
    pressure: Option<f64>,
) -> Result<BurnInfo> {
    let ThrustInfo {
        thrust,
        mass_flow,
        vessel_mass,
    } = get_thrust_info(vessel, pressure).await?;

    if thrust <= 0.0 {
        return Ok(BurnInfo {
            burn_time: f64::INFINITY,
            distance_covered: 0.0,
        });
    }

    let burn_time = (vessel_mass / mass_flow) * (1.0 - (-mass_flow * delta_v / thrust).exp());

    // jesus fucking christ
    let distance_covered = -(thrust / mass_flow)
        * (((vessel_mass - burn_time * mass_flow).ln() * (burn_time - vessel_mass / mass_flow)
            + vessel_mass * vessel_mass.ln() / mass_flow
            - burn_time)
            - burn_time * vessel_mass.ln());

    Ok(BurnInfo {
        burn_time,
        distance_covered,
    })
}

pub async fn get_next_node(vessel: &Vessel) -> Result<Option<Node>> {
    Ok(vessel
        .get_control()
        .await?
        .get_nodes()
        .await?
        .into_iter()
        .next())
}

#[macro_export]
macro_rules! reference_frame_or_vessel_frame {
    ($vessel:expr, $rf:expr) => {
        if let Some(ref r) = $rf {
            r
        } else {
            &$vessel.get_reference_frame().await?
        }
    };
}

pub async fn get_translation_force(
    vessel: &Vessel,
    direction: Vector3<f64>,
    reference_frame: Option<&ReferenceFrame>,
) -> Result<f64> {
    if direction.magnitude().is_zero() {
        return Ok(0.0);
    }
    let direction = direction.normalize();

    let reference_frame = reference_frame_or_vessel_frame!(vessel, reference_frame);
    let rotation: Quaternion<f64> = vessel.rotation(reference_frame).await?.into();
    let inverse_rotation = rotation.invert();

    let direction_vessel_frame = inverse_rotation * direction;

    let (positive, negative) = vessel.get_available_rcs_force().await?;
    let (positive, negative): (Vector3<f64>, Vector3<f64>) = (positive.into(), negative.into());
    // actual rcs force is, as far as i can tell, exactly one fifth of what get_available_rcs_force claims. why????
    let (positive, negative) = (positive / 5.0, negative / 5.0);

    let forces = vec3(
        if direction_vessel_frame.x.is_sign_positive() {
            positive.x
        } else {
            negative.x
        },
        if direction_vessel_frame.y.is_sign_positive() {
            positive.y
        } else {
            negative.y
        },
        if direction_vessel_frame.z.is_sign_positive() {
            positive.z
        } else {
            negative.z
        },
    );

    Ok(forces.dot(direction_vessel_frame))
}

pub async fn get_ascending_node_direction(
    orbit: &Orbit,
    reference_frame: &ReferenceFrame,
) -> Result<Vector3<f64>> {
    let reference_normal: Vector3<f64> = orbit
        .static_reference_plane_normal(reference_frame)
        .await?
        .into();
    let reference_direction: Vector3<f64> = orbit
        .static_reference_plane_direction(reference_frame)
        .await?
        .into();

    let longitude_of_ascending_node = Rad(orbit.get_longitude_of_ascending_node().await?);
    let ascending_node_direction =
        Quaternion::from_axis_angle(reference_normal, -longitude_of_ascending_node)
            * reference_direction;

    Ok(ascending_node_direction)
}

pub async fn get_periapsis_direction(
    orbit: &Orbit,
    reference_frame: &ReferenceFrame,
) -> Result<Vector3<f64>> {
    let ascending_node_direction = get_ascending_node_direction(orbit, reference_frame).await?;
    let argument_of_periapsis = Rad(orbit.get_argument_of_periapsis().await?);
    let orbit_normal = get_orbit_normal(orbit, reference_frame).await?;

    let rotation = Quaternion::from_axis_angle(orbit_normal, -argument_of_periapsis);

    Ok(rotation * ascending_node_direction)
}

pub async fn get_orbit_normal(
    orbit: &Orbit,
    reference_frame: &ReferenceFrame,
) -> Result<Vector3<f64>> {
    let reference_normal: Vector3<f64> = orbit
        .static_reference_plane_normal(reference_frame)
        .await?
        .into();

    let inclination = Rad(orbit.get_inclination().await?);
    let ascending_node_direction = get_ascending_node_direction(orbit, reference_frame).await?;
    let orbit_normal =
        Quaternion::from_axis_angle(ascending_node_direction, -inclination) * reference_normal;

    Ok(orbit_normal)
}

pub async fn orbital_velocity_at(
    orbit: &Orbit,
    ut: f64,
    reference_frame: &ReferenceFrame,
) -> Result<Vector3<f64>> {
    let dt = 0.0001;
    let position_0: Vector3<f64> = orbit.position_at(ut, reference_frame).await?.into();
    let position_1: Vector3<f64> = orbit.position_at(ut + dt, reference_frame).await?.into();

    Ok((position_1 - position_0) / dt)
}

pub async fn get_prograde_normal_radial_directions_at(
    orbit: &Orbit,
    ut: f64,
    reference_frame: &ReferenceFrame,
) -> Result<(Vector3<f64>, Vector3<f64>, Vector3<f64>)> {
    let velocity_at = orbital_velocity_at(orbit, ut, reference_frame).await?;
    if velocity_at.magnitude2().is_zero() {
        return Ok((
            vec3(1.0, 0.0, 0.0),
            vec3(0.0, 1.0, 0.0),
            vec3(0.0, 0.0, 1.0),
        ));
    }

    let prograde = velocity_at.normalize();
    let normal = get_orbit_normal(orbit, reference_frame).await?;
    let radial = normal.cross(prograde);

    Ok((prograde, normal, radial))
}

pub async fn get_circular_orbit_speed(body: &CelestialBody, radius: f64) -> Result<f64> {
    Ok((body.get_gravitational_parameter().await? / radius).sqrt())
}

pub async fn node_exists(node: &Node, vessel: &Vessel) -> Result<bool> {
    let control = vessel.get_control().await?;

    let ut = node.get_ut().await?;
    for other_node in control.get_nodes().await? {
        let other_ut = other_node.get_ut().await?;
        if other_ut == ut {
            return Ok(true);
        } else if other_ut > ut {
            return Ok(false);
        }
    }

    Ok(false)
}

pub fn get_relative_ascending_node_direction(
    current_orbit_normal: Vector3<f64>,
    target_orbit_normal: Vector3<f64>,
) -> Vector3<f64> {
    current_orbit_normal.cross(target_orbit_normal).normalize()
}

pub fn get_relative_descending_node_direction(
    current_orbit_normal: Vector3<f64>,
    target_orbit_normal: Vector3<f64>,
) -> Vector3<f64> {
    -get_relative_ascending_node_direction(current_orbit_normal, target_orbit_normal)
}

pub async fn get_time_to_mean_anomaly(orbit: &Orbit, mean_anomaly: Rad<f64>) -> Result<f64> {
    let current_mean_anomaly = Rad(orbit.get_mean_anomaly().await?);

    let diff = mean_anomaly - current_mean_anomaly
        + if current_mean_anomaly > mean_anomaly {
            Rad::full_turn()
        } else {
            Rad::zero()
        };
    let period = orbit.get_period().await?;
    let rads_per_second = Rad::full_turn() / period;

    Ok(diff / rads_per_second)
}

pub fn flatten_vector(vector: Vector3<f64>, plane_normal: Vector3<f64>) -> Vector3<f64> {
    let plane_normal = plane_normal.normalize();

    vector - plane_normal * vector.dot(plane_normal)
}

pub fn is_below_plane(vector: Vector3<f64>, plane_normal: Vector3<f64>) -> bool {
    vector.dot(plane_normal).is_sign_negative()
}

pub async fn get_true_anomaly_of_direction(
    orbit: &Orbit,
    direction: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<Rad<f64>> {
    let periapsis_direction = get_periapsis_direction(orbit, reference_frame).await?;
    let orbit_normal = get_orbit_normal(orbit, reference_frame).await?;
    let direction = flatten_vector(direction, orbit_normal).normalize();

    let angle = periapsis_direction.angle(direction);
    let bisector_plane = periapsis_direction.cross(orbit_normal);
    let is_after_apoapsis = is_below_plane(direction, bisector_plane);

    Ok(if is_after_apoapsis { -angle } else { angle })
}

pub async fn get_direction_of_true_anomaly(
    orbit: &Orbit,
    true_anomaly: Rad<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<Vector3<f64>> {
    let periapsis_direction = get_periapsis_direction(orbit, reference_frame).await?;
    let orbit_normal = get_orbit_normal(orbit, reference_frame).await?;
    let rotation = Quaternion::from_axis_angle(orbit_normal, -true_anomaly);

    Ok(rotation * periapsis_direction)
}

pub async fn get_relative_ascending_node_true_anomaly(
    orbit: &Orbit,
    target_orbit_normal: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<Rad<f64>> {
    let orbit_normal = get_orbit_normal(orbit, reference_frame).await?;
    let direction = get_relative_ascending_node_direction(orbit_normal, target_orbit_normal);

    get_true_anomaly_of_direction(orbit, direction, reference_frame).await
}

pub async fn get_relative_descending_node_true_anomaly(
    orbit: &Orbit,
    target_orbit_normal: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<Rad<f64>> {
    let orbit_normal = get_orbit_normal(orbit, reference_frame).await?;
    let direction = get_relative_descending_node_direction(orbit_normal, target_orbit_normal);

    get_true_anomaly_of_direction(orbit, direction, reference_frame).await
}

pub async fn get_relative_ascending_node_ut(
    orbit: &Orbit,
    target_orbit_normal: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<f64> {
    Ok(orbit
        .ut_at_true_anomaly(
            get_relative_ascending_node_true_anomaly(orbit, target_orbit_normal, reference_frame)
                .await?
                .0,
        )
        .await?)
}

pub async fn get_relative_descending_node_ut(
    orbit: &Orbit,
    target_orbit_normal: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<f64> {
    Ok(orbit
        .ut_at_true_anomaly(
            get_relative_descending_node_true_anomaly(orbit, target_orbit_normal, reference_frame)
                .await?
                .0,
        )
        .await?)
}

pub fn get_eccentricity(apsis_0: f64, apsis_1: f64) -> f64 {
    ((apsis_0 - apsis_1) / (apsis_0 + apsis_1)).abs()
}

pub fn get_apsis_speed_ratio(apsis_0: f64, apsis_1: f64) -> f64 {
    let eccentricity = get_eccentricity(apsis_0, apsis_1);

    (1.0 - eccentricity * eccentricity).sqrt() / (1.0 - eccentricity)
}

pub async fn get_apoapsis_speed(body: &CelestialBody, apsis_0: f64, apsis_1: f64) -> Result<f64> {
    let ratio = get_apsis_speed_ratio(apsis_0, apsis_1);
    let sma = (apsis_0 + apsis_1) / 2.0;
    let circular_speed = get_circular_orbit_speed(body, sma).await?;

    Ok(circular_speed / ratio)
}

pub async fn get_periapsis_speed(body: &CelestialBody, apsis_0: f64, apsis_1: f64) -> Result<f64> {
    let ratio = get_apsis_speed_ratio(apsis_0, apsis_1);
    let sma = (apsis_0 + apsis_1) / 2.0;
    let circular_speed = get_circular_orbit_speed(body, sma).await?;

    Ok(circular_speed * ratio)
}

pub async fn get_orbital_period(body: &CelestialBody, apsis_0: f64, apsis_1: f64) -> Result<f64> {
    let sma = (apsis_0 + apsis_1) / 2.0;
    let mu = body.get_gravitational_parameter().await?;

    Ok(f64::consts::TAU * (sma.powi(3) / mu).sqrt())
}

pub async fn ut_at_true_anomaly_offset(
    orbit: &Orbit,
    true_anomaly_offset: Rad<f64>,
) -> Result<f64> {
    let period = orbit.get_period().await?;
    let current_true_anomaly = Rad(orbit.get_true_anomaly().await?);
    let stacks = (true_anomaly_offset / Rad::full_turn()).floor();

    let remainder = (current_true_anomaly + true_anomaly_offset).normalize_signed();
    let ut = orbit.ut_at_true_anomaly(remainder.0).await?;

    Ok(ut + period * stacks)
}

/// this is so hacky
pub async fn get_current_ut_from_orbit(orbit: &Orbit) -> Result<f64> {
    Ok(orbit
        .ut_at_true_anomaly(orbit.get_true_anomaly().await? - 0.00000001)
        .await?
        - orbit.get_period().await?)
}

pub async fn height_above_surface_at_position(
    body: &CelestialBody,
    body_position: Vector3<f64>,
    reference_frame: &ReferenceFrame,
) -> Result<f64> {
    let (latitude, longitude) = (
        body.latitude_at_position(body_position.into(), reference_frame)
            .await?,
        body.longitude_at_position(body_position.into(), reference_frame)
            .await?,
    );

    Ok(body
        .altitude_at_position(body_position.into(), reference_frame)
        .await?
        - body.surface_height(latitude, longitude).await?)
}

pub async fn surface_position(
    body: &CelestialBody,
    body_position: Vector3<f64>,
) -> Result<Vector3<f64>> {
    Ok(body_position
        - body_position.normalize_to(
            height_above_surface_at_position(
                body,
                body_position,
                &body.get_reference_frame().await?,
            )
            .await?,
        ))
}

pub async fn get_target_orbit(space_center: &SpaceCenter) -> Result<Option<Orbit>> {
    Ok(
        if let Some(target) = space_center.get_target_body().await? {
            target.get_orbit().await?
        } else if let Some(target) = space_center.get_target_vessel().await? {
            Some(target.get_orbit().await?)
        } else {
            None
        },
    )
}

pub async fn best_surface_orbit_alignment_angles(
    body_position: Vector3<f64>,
    orbit_normal: Vector3<f64>,
) -> Result<(Rad<f64>, Rad<f64>)> {
    let circle_radius = body_position.xz().magnitude();

    let mut function = |angle: f64| {
        orbit_normal.dot(vec3(
            angle.cos() * circle_radius,
            body_position.y,
            angle.sin() * circle_radius,
        ))
    };
    let mut derivative = |angle: f64| {
        -orbit_normal.x * circle_radius * angle.sin() + orbit_normal.z * circle_radius * angle.cos()
    };
    let mut derivative2 = |angle: f64| {
        -orbit_normal.x * circle_radius * angle.cos() - orbit_normal.z * circle_radius * angle.sin()
    };

    let root_angles = newton_find_all_roots(
        0.0,
        Rad::full_turn().0,
        100,
        100,
        &mut function,
        &mut derivative,
    );

    let (angle_0, angle_1) = if root_angles.is_empty() {
        let extrema_angles = newton_find_all_roots(
            0.0,
            Rad::full_turn().0,
            100,
            100,
            &mut derivative,
            &mut derivative2,
        );
        if extrema_angles.is_empty() {
            warn!(
                "best_surface_orbit_alignment_directions: couldn't even find any extrema angles?"
            );
            (0.0, 0.0)
        } else {
            let mut min_angle = *extrema_angles.first().unwrap();
            let mut min = function(min_angle);
            for angle in extrema_angles {
                let product = function(angle);
                if product < min {
                    min_angle = angle;
                    min = product;
                }
            }

            (min_angle, min_angle)
        }
    } else {
        let first = *root_angles.first().unwrap();
        (first, *root_angles.get(2).unwrap_or(&first))
    };

    Ok((Rad(angle_0), Rad(angle_1)))
}

pub async fn throttle_for_acceleration(vessel: &Vessel, acceleration: f64) -> Result<f64> {
    let ThrustInfo {
        thrust,
        vessel_mass,
        ..
    } = get_thrust_info(vessel, None).await?;

    Ok(acceleration / (thrust / vessel_mass))
}

pub fn centrifugal_acceleration(
    angular_velocity: Vector3<f64>,
    body_position: Vector3<f64>,
) -> Vector3<f64> {
    let projected_position = flatten_vector(body_position, angular_velocity);
    let circle_radius = projected_position.magnitude();
    let latitude_speed = angular_velocity.magnitude() * circle_radius;
    let centrifugal_acceleration = latitude_speed.powi(2) / circle_radius;

    projected_position.normalize_to(centrifugal_acceleration)
}

pub fn gravitational_acceleration(
    gravitational_parameter: f64,
    body_position: Vector3<f64>,
) -> Vector3<f64> {
    -body_position.normalize_to(gravitational_parameter / body_position.magnitude2())
}

pub fn surface_acceleration(
    gravitational_parameter: f64,
    angular_velocity: Vector3<f64>,
    body_position: Vector3<f64>,
) -> Vector3<f64> {
    gravitational_acceleration(gravitational_parameter, body_position)
        + centrifugal_acceleration(angular_velocity, body_position)
}

impl FlightComputer {
    pub async fn next_surface_orbit_alignment_delay(
        &self,
        body: &CelestialBody,
        body_position: Vector3<f64>,
        orbit_normal: Vector3<f64>,
        angle_tolerance: impl Into<Rad<f64>>,
    ) -> Result<f64> {
        let angle_tolerance: Rad<f64> = angle_tolerance.into();

        let body_reference_frame = body.get_reference_frame().await?;
        let non_rotating_reference_frame = body.get_non_rotating_reference_frame().await?;
        let absolute_position: Vector3<f64> = self
            .space_center
            .transform_position(
                body_position.into(),
                &body_reference_frame,
                &non_rotating_reference_frame,
            )
            .await?
            .into();
        let current_angle = Rad::atan2(absolute_position.z, absolute_position.x);
        let (angle_0, angle_1) =
            best_surface_orbit_alignment_angles(body_position, orbit_normal).await?;
        let (angle_diff_0, angle_diff_1) = (
            (angle_0 - current_angle).normalize_signed(),
            (angle_1 - current_angle).normalize_signed(),
        );

        if orbit_normal.angle(vec3(0.0, 1.0, 0.0)) > angle_tolerance
            && Rad(angle_diff_0.0.abs().min(angle_diff_1.0.abs())) > angle_tolerance
        {
            let angle_diff =
                if angle_diff_0.0.is_sign_positive() && angle_diff_1.0.is_sign_negative() {
                    angle_diff_0
                } else if angle_diff_0.0.is_sign_negative() && angle_diff_1.0.is_sign_positive() {
                    angle_diff_1
                } else {
                    Rad(angle_diff_0.0.min(angle_diff_1.0)).normalize()
                };

            let wait_time = angle_diff / Rad(body.get_rotational_speed().await?);

            Ok(wait_time)
        } else {
            Ok(0.0)
        }
    }

    pub async fn body_position_of_absolute_position_at_ut(
        &self,
        body: &CelestialBody,
        absolute_position: Vector3<f64>,
        ut: f64,
    ) -> Result<Vector3<f64>> {
        let non_rotating_reference_frame = body.get_non_rotating_reference_frame().await?;
        let body_reference_frame = body.get_reference_frame().await?;
        let angular_velocity: Vector3<f64> = body
            .angular_velocity(&non_rotating_reference_frame)
            .await?
            .into();

        let rotation = Quaternion::from_axis_angle(
            angular_velocity.normalize(),
            -Rad(angular_velocity.magnitude()) * (ut - self.space_center.get_ut().await?),
        );

        Ok(self
            .space_center
            .transform_position(
                (rotation * absolute_position).into(),
                &non_rotating_reference_frame,
                &body_reference_frame,
            )
            .await?
            .into())
    }

    pub async fn body_velocity_of_absolute_velocity_at_ut(
        &self,
        body: &CelestialBody,
        absolute_position: Vector3<f64>,
        absolute_velocity: Vector3<f64>,
        ut: f64,
    ) -> Result<Vector3<f64>> {
        let non_rotating_reference_frame = body.get_non_rotating_reference_frame().await?;
        let body_reference_frame = body.get_reference_frame().await?;
        let angular_velocity: Vector3<f64> = body
            .angular_velocity(&non_rotating_reference_frame)
            .await?
            .into();

        let rotation = Quaternion::from_axis_angle(
            angular_velocity.normalize(),
            -Rad(angular_velocity.magnitude()) * (ut - self.space_center.get_ut().await?),
        );

        Ok(self
            .space_center
            .transform_velocity(
                (rotation * absolute_position).into(),
                (rotation * absolute_velocity).into(),
                &non_rotating_reference_frame,
                &body_reference_frame,
            )
            .await?
            .into())
    }
}
