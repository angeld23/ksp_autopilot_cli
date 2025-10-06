use core::f64;
use std::{collections::BTreeMap, f64::consts::TAU};

use anyhow::{anyhow, Result};
use async_recursion::async_recursion;
use cgmath::{
    ulps_eq, vec2, vec3, Angle, InnerSpace, Quaternion, Rad, Rotation3, Vector2, Vector3,
};
use krpc_client::services::space_center::{CelestialBody, Node, Orbit, SpaceCenter, Vessel};

#[derive(Debug, Clone, PartialEq)]
pub struct LocalOrbit {
    pub body: Box<LocalBody>,
    pub eccentricity: f64,
    pub semi_major_axis: f64,
    pub inclination: Rad<f64>,
    pub longitude_of_ascending_node: Rad<f64>,
    pub longitude_of_periapsis: Rad<f64>,
    pub epoch: f64,
    pub mean_anomaly_at_epoch: Rad<f64>,
}

impl LocalOrbit {
    #[async_recursion]
    pub async fn from_orbit(orbit: &Orbit) -> Result<Self> {
        let lan = orbit.get_longitude_of_ascending_node().await?;
        Ok(Self {
            body: Box::new(LocalBody::from_celestial_body(&orbit.get_body().await?).await?),
            eccentricity: orbit.get_eccentricity().await?,
            semi_major_axis: orbit.get_semi_major_axis().await?,
            inclination: Rad(orbit.get_inclination().await?),
            longitude_of_ascending_node: Rad(lan),
            longitude_of_periapsis: Rad(lan + orbit.get_argument_of_periapsis().await?).normalize(),
            epoch: orbit.get_epoch().await?,
            mean_anomaly_at_epoch: Rad(orbit.get_mean_anomaly_at_epoch().await?),
        })
    }

    pub fn argument_of_periapsis(&self) -> Rad<f64> {
        (self.longitude_of_periapsis - self.longitude_of_ascending_node).normalize()
    }

    pub fn ascending_node_direction(&self) -> Vector3<f64> {
        Quaternion::from_axis_angle(vec3(0.0, 1.0, 0.0), -self.longitude_of_ascending_node)
            * vec3(1.0, 0.0, 0.0)
    }

    pub fn normal(&self) -> Vector3<f64> {
        let ascending_node_direction = self.ascending_node_direction();
        Quaternion::from_axis_angle(ascending_node_direction, -self.inclination)
            * vec3(0.0, 1.0, 0.0)
    }

    pub fn periapsis_direction(&self) -> Vector3<f64> {
        let ascending_node_direction = self.ascending_node_direction();
        let argument_of_periapsis = self.argument_of_periapsis();
        let orbit_normal = self.normal();

        let rotation = Quaternion::from_axis_angle(orbit_normal, -argument_of_periapsis);

        rotation * ascending_node_direction
    }

    pub fn mean_motion(&self) -> Rad<f64> {
        self.body.mean_motion_of_sma(self.semi_major_axis)
    }

    pub fn mean_anomaly_at_ut(&self, ut: f64) -> Rad<f64> {
        let mut ma = self.mean_anomaly_at_epoch + self.mean_motion() * (ut - self.epoch);
        if self.is_elliptical() {
            ma = ma.normalize()
        }
        ma
    }

    pub fn period(&self) -> f64 {
        self.body.orbital_period_of_sma(self.semi_major_axis)
    }

    pub fn semi_minor_axis(&self) -> f64 {
        self.semi_major_axis.abs() * (1.0 - self.eccentricity.powi(2)).abs().sqrt()
    }

    pub fn is_elliptical(&self) -> bool {
        self.eccentricity < 1.0
    }

    pub fn eccentric_anomaly_of_mean_anomaly(&self, mean_anomaly: Rad<f64>) -> Rad<f64> {
        let mean_anomaly = if self.is_elliptical() {
            mean_anomaly.normalize()
        } else {
            mean_anomaly
        };

        let mut guess = if self.is_elliptical() {
            mean_anomaly
        } else {
            Rad((2.0 * mean_anomaly.0.abs() / self.eccentricity + 1.8).ln()
                * mean_anomaly.0.signum())
        };
        for _ in 0..30 {
            if self.is_elliptical() {
                guess -= (guess - Rad(self.eccentricity * guess.sin()) - mean_anomaly)
                    / (1.0 - self.eccentricity * guess.cos());
            } else {
                guess -= (Rad(guess.0.sinh()) * self.eccentricity - guess - mean_anomaly)
                    / (guess.0.cosh() * self.eccentricity - 1.0);
            }
        }

        if self.is_elliptical() {
            guess.normalize()
        } else {
            guess
        }
    }

    pub fn eccentric_anomaly_at_ut(&self, ut: f64) -> Rad<f64> {
        self.eccentric_anomaly_of_mean_anomaly(self.mean_anomaly_at_ut(ut))
    }

    pub fn local_position_at_eccentric_anomaly(&self, eccentric_anomaly: Rad<f64>) -> Vector2<f64> {
        if self.is_elliptical() {
            vec2(
                self.semi_major_axis * (eccentric_anomaly.cos() - self.eccentricity),
                self.semi_minor_axis() * eccentric_anomaly.sin(),
            )
        } else {
            vec2(
                self.semi_major_axis * (eccentric_anomaly.0.cosh() - self.eccentricity),
                self.semi_minor_axis() * eccentric_anomaly.0.sinh(),
            )
        }
    }

    pub fn local_velocity_at_eccentric_anomaly(&self, eccentric_anomaly: Rad<f64>) -> Vector2<f64> {
        if self.is_elliptical() {
            let eccentric_anomaly_derivative =
                TAU / (self.period() * (1.0 - self.eccentricity * eccentric_anomaly.cos()));

            eccentric_anomaly_derivative
                * vec2(
                    -self.semi_major_axis * eccentric_anomaly.sin(),
                    self.semi_minor_axis() * eccentric_anomaly.cos(),
                )
        } else {
            let eccentric_anomaly_derivative =
                self.mean_motion().0 / (self.eccentricity * eccentric_anomaly.0.cosh() - 1.0);

            eccentric_anomaly_derivative
                * vec2(
                    self.semi_major_axis * eccentric_anomaly.0.sinh(),
                    self.semi_minor_axis() * eccentric_anomaly.0.cosh(),
                )
        }
    }

    pub fn local_position_at_ut(&self, ut: f64) -> Vector2<f64> {
        self.local_position_at_eccentric_anomaly(self.eccentric_anomaly_at_ut(ut))
    }

    pub fn local_velocity_at_ut(&self, ut: f64) -> Vector2<f64> {
        self.local_velocity_at_eccentric_anomaly(self.eccentric_anomaly_at_ut(ut))
    }

    pub fn local_to_absolute_vector(&self, local_vector: Vector2<f64>) -> Vector3<f64> {
        let x = self.periapsis_direction();
        let y = x.cross(self.normal());

        local_vector.x * x + local_vector.y * y
    }

    pub fn absolute_to_local_vector(&self, absolute_vector: Vector3<f64>) -> Vector2<f64> {
        let x = self.periapsis_direction();
        let y = x.cross(self.normal());

        vec2(x.dot(absolute_vector), y.dot(absolute_vector))
    }

    pub fn position_at_ut(&self, ut: f64) -> Vector3<f64> {
        self.local_to_absolute_vector(self.local_position_at_ut(ut))
    }

    pub fn velocity_at_ut(&self, ut: f64) -> Vector3<f64> {
        self.local_to_absolute_vector(self.local_velocity_at_ut(ut))
    }

    pub fn true_anomaly_at_ut(&self, ut: f64) -> Rad<f64> {
        self.true_anomaly_of_local_position(self.local_position_at_ut(ut))
    }

    pub fn periapsis(&self) -> f64 {
        self.semi_major_axis * (1.0 - self.eccentricity)
    }

    pub fn apoapsis(&self) -> f64 {
        self.semi_major_axis * (1.0 + self.eccentricity)
    }

    pub fn periapsis_epoch(&self) -> f64 {
        self.epoch - self.mean_anomaly_at_epoch / self.mean_motion()
    }

    pub fn eccentric_anomaly_time_delay(&self, eccentric_anomaly_offset: Rad<f64>) -> f64 {
        let stacks = (eccentric_anomaly_offset / Rad::full_turn())
            .floor()
            .max(0.0);
        let normalized_offset = eccentric_anomaly_offset.normalize();
        let period = self.period();

        if self.is_elliptical() {
            period
                * (stacks
                    + (normalized_offset - Rad(self.eccentricity * normalized_offset.sin()))
                        / Rad::full_turn())
        } else {
            period
                * ((Rad(self.eccentricity * eccentric_anomaly_offset.0.sinh())
                    - eccentric_anomaly_offset)
                    / Rad::full_turn())
        }
    }

    pub fn mean_anomaly_of_eccentric_anomaly(&self, eccentric_anomaly: Rad<f64>) -> Rad<f64> {
        let eccentric_anomaly = eccentric_anomaly.normalize();

        if self.is_elliptical() {
            eccentric_anomaly - Rad(self.eccentricity * eccentric_anomaly.sin())
        } else {
            let eccentric_anomaly = eccentric_anomaly.normalize_signed();
            Rad(self.eccentricity * eccentric_anomaly.0.sinh() - eccentric_anomaly.0)
        }
    }

    pub fn mean_anomaly_time_delay(&self, mean_anomaly_offset: Rad<f64>) -> f64 {
        mean_anomaly_offset / self.mean_motion()
    }

    pub fn eccentric_anomaly_of_true_anomaly(&self, true_anomaly: Rad<f64>) -> Rad<f64> {
        let true_anomaly = if self.is_elliptical() {
            true_anomaly.normalize()
        } else {
            true_anomaly.normalize_signed()
        };

        if ulps_eq!(true_anomaly.0, 0.0)
            || ulps_eq!(true_anomaly.0, Rad::<f64>::full_turn().0 / 2.0)
        {
            return true_anomaly;
        }

        let ea = if self.is_elliptical() {
            Rad(2.0
                * (((1.0 - self.eccentricity) / (1.0 + self.eccentricity)).sqrt()
                    * (true_anomaly / 2.0).tan())
                .atan())
        } else {
            Rad(2.0
                * (((self.eccentricity - 1.0) / (self.eccentricity + 1.0)).sqrt()
                    * (true_anomaly / 2.0).tan())
                .atanh())
        };

        if self.is_elliptical() {
            ea.normalize()
        } else {
            ea.normalize_signed()
        }
    }

    pub fn mean_anomaly_of_true_anomaly(&self, true_anomaly: Rad<f64>) -> Rad<f64> {
        self.mean_anomaly_of_eccentric_anomaly(self.eccentric_anomaly_of_true_anomaly(true_anomaly))
    }

    pub fn true_anomaly_of_local_position(&self, local_position: Vector2<f64>) -> Rad<f64> {
        Rad(local_position.y.atan2(local_position.x))
    }

    pub fn true_anomaly_of_position(&self, position: Vector3<f64>) -> Rad<f64> {
        self.true_anomaly_of_local_position(self.absolute_to_local_vector(position))
    }

    pub fn true_anomaly_of_eccentric_anomaly(&self, eccentric_anomaly: Rad<f64>) -> Rad<f64> {
        self.true_anomaly_of_local_position(
            self.local_position_at_eccentric_anomaly(eccentric_anomaly),
        )
    }

    pub fn true_anomaly_time_delay(&self, true_anomaly_offset: Rad<f64>) -> f64 {
        let stacks = (true_anomaly_offset / Rad::full_turn()).floor().max(0.0);
        let normalized_offset = true_anomaly_offset.normalize();
        let period = self.period();

        self.eccentric_anomaly_time_delay(self.eccentric_anomaly_of_true_anomaly(normalized_offset))
            + period * stacks
    }

    /// # Returns
    /// (ut, distance)
    pub fn find_closest_approach(
        &self,
        other: &LocalOrbit,
        start_ut: f64,
        look_ahead: f64,
    ) -> (f64, f64) {
        assert_eq!(
            self.body.name, other.body.name,
            "parent bodies do not match"
        );

        let increments = 360;

        let mut start = start_ut;
        let mut end = start_ut + look_ahead;

        let mut closest_ut = start;
        let mut closest_distance = f64::INFINITY;

        for _ in 0..10 {
            let step_size = (end - start) / increments as f64;

            for i in 0..increments {
                let ut = start + i as f64 * step_size;

                let position_0 = self.position_at_ut(ut);
                let position_1 = other.position_at_ut(ut);
                let distance = (position_0 - position_1).magnitude();

                if distance < closest_distance {
                    closest_ut = ut;
                    closest_distance = distance;
                }
            }

            start = closest_ut - step_size;
            end = closest_ut + step_size;
        }

        if closest_ut < start_ut {
            let position_0 = self.position_at_ut(start_ut);
            let position_1 = other.position_at_ut(start_ut);
            let distance = (position_0 - position_1).magnitude();

            return (start_ut, distance);
        }

        (closest_ut, closest_distance)
    }

    pub fn will_escape(&self) -> bool {
        !self.is_elliptical() || self.apoapsis() > self.body.sphere_of_influence
    }

    pub fn find_soi_exit(&self, start_ut: f64) -> Option<f64> {
        let start_distance = self.position_at_ut(start_ut).magnitude();
        if start_distance > self.body.sphere_of_influence {
            return Some(start_ut);
        }
        if !self.will_escape() {
            return None;
        }

        let mut bisect_start = start_ut;
        let mut bisect_end = if self.is_elliptical() {
            start_ut + self.period() / 2.0
                - self.mean_anomaly_time_delay(self.mean_anomaly_at_ut(start_ut))
        } else {
            let mut offset = 100.0;

            for _ in 0..30 {
                let distance = self.position_at_ut(start_ut + offset).magnitude();
                if distance > self.body.sphere_of_influence {
                    break;
                }
                offset *= 2.0;
            }

            if self.position_at_ut(start_ut + offset).magnitude() <= self.body.sphere_of_influence {
                return None;
            }

            start_ut + offset
        };

        for _ in 0..30 {
            let middle = (bisect_start + bisect_end) / 2.0;
            let middle_distance = self.position_at_ut(middle).magnitude();

            if middle_distance > self.body.sphere_of_influence {
                bisect_end = middle;
            } else {
                bisect_start = middle;
            }
        }

        Some(bisect_end)
    }

    pub fn get_orbital_state_change(&self, ut: f64) -> OrbitalStateChange {
        OrbitalStateChange {
            body: Box::into_inner(self.body.clone()),
            ut,
            position: self.position_at_ut(ut),
            velocity: self.velocity_at_ut(ut),
        }
    }

    pub fn singleton_trajectory(&self, ut: f64) -> OrbitalTrajectory {
        OrbitalTrajectory {
            state_changes: vec![self.get_orbital_state_change(ut)],
        }
    }

    pub fn prograde(&self, ut: f64) -> Vector3<f64> {
        self.velocity_at_ut(ut).normalize()
    }

    pub fn radial(&self, ut: f64) -> Vector3<f64> {
        self.normal().cross(self.prograde(ut))
    }

    pub fn prograde_normal_radial_to_absolute(&self, ut: f64, pnr: Vector3<f64>) -> Vector3<f64> {
        self.prograde(ut) * pnr.x + self.normal() * pnr.y + self.radial(ut) * pnr.z
    }

    pub fn absolute_to_prograde_normal_radial(
        &self,
        ut: f64,
        absolute: Vector3<f64>,
    ) -> Vector3<f64> {
        let prograde = self.prograde(ut);
        let normal = self.normal();
        let radial = self.radial(ut);

        vec3(
            absolute.dot(prograde),
            absolute.dot(normal),
            absolute.dot(radial),
        )
    }

    pub fn most_recent_periapsis_ut(&self, current_ut: f64) -> f64 {
        let elapsed = current_ut - self.periapsis_epoch();

        self.periapsis_epoch() + self.period() * (elapsed / self.period()).floor()
    }

    pub fn next_periapsis_ut(&self, current_ut: f64) -> f64 {
        if self.is_elliptical() {
            self.most_recent_periapsis_ut(current_ut) + self.period()
        } else {
            self.periapsis_epoch()
        }
    }

    pub fn next_apoapsis_ut(&self, current_ut: f64) -> f64 {
        if self.is_elliptical() {
            let ut = self.most_recent_periapsis_ut(current_ut) + self.period() / 2.0;
            if ut < current_ut {
                ut + self.period()
            } else {
                ut
            }
        } else {
            f64::INFINITY
        }
    }

    pub fn after_node(&self, local_node: LocalNode) -> Self {
        self.body.create_orbit_from_state_vectors(
            local_node.ut,
            self.position_at_ut(local_node.ut),
            self.velocity_at_ut(local_node.ut)
                + self.prograde_normal_radial_to_absolute(local_node.ut, local_node.burn_vector),
        )
    }
}

#[derive(Clone, PartialEq)]
pub struct LocalBody {
    pub name: String,
    pub gravitational_parameter: f64,
    pub sphere_of_influence: f64,
    pub radius: f64,
    pub orbit: Option<LocalOrbit>,
}

impl std::fmt::Debug for LocalBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "LocalBody({})", self.name)
    }
}

impl LocalBody {
    pub async fn from_celestial_body(body: &CelestialBody) -> Result<Self> {
        Ok(Self {
            name: body.get_name().await?,
            gravitational_parameter: body.get_gravitational_parameter().await?,
            sphere_of_influence: body.get_sphere_of_influence().await?,
            radius: body.get_equatorial_radius().await?,
            orbit: if let Some(orbit) = body.get_orbit().await? {
                Some(LocalOrbit::from_orbit(&orbit).await?)
            } else {
                None
            },
        })
    }

    pub fn create_orbit_from_state_vectors(
        &self,
        ut: f64,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> LocalOrbit {
        let specific_angular_momentum = velocity.cross(position);
        let ascending_node_vector = specific_angular_momentum.cross(vec3(0.0, 1.0, 0.0));
        let eccentricity_vector = specific_angular_momentum.cross(velocity)
            / self.gravitational_parameter
            - position.normalize();
        let semi_latus_rectum =
            specific_angular_momentum.magnitude2() / self.gravitational_parameter;

        let semi_major_axis = semi_latus_rectum / (1.0 - eccentricity_vector.magnitude2());
        let inclination = specific_angular_momentum.angle(vec3(0.0, 1.0, 0.0));

        let longitude_of_ascending_node = (ascending_node_vector.angle(vec3(1.0, 0.0, 0.0))
            * ascending_node_vector.z.signum())
        .normalize();
        let argument_of_periapsis = (ascending_node_vector.angle(eccentricity_vector)
            * eccentricity_vector.y.signum())
        .normalize();
        let true_anomaly_at_epoch =
            eccentricity_vector.angle(position) * position.dot(velocity).signum();
        let longitude_of_periapsis =
            (argument_of_periapsis + longitude_of_ascending_node).normalize();

        let mut orbit = LocalOrbit {
            body: Box::new(self.clone()),
            eccentricity: eccentricity_vector.magnitude(),
            semi_major_axis,
            inclination,
            longitude_of_ascending_node,
            longitude_of_periapsis,
            epoch: ut,
            mean_anomaly_at_epoch: Rad(0.0),
        };

        orbit.mean_anomaly_at_epoch = orbit.mean_anomaly_of_true_anomaly(true_anomaly_at_epoch);

        orbit
    }

    pub fn find_soi_enter(
        &self,
        orbit: &LocalOrbit,
        start_ut: f64,
        look_ahead: f64,
    ) -> Option<f64> {
        let body_orbit = self.orbit.as_ref()?;
        let closest_approach_ut = if !orbit.is_elliptical() {
            let (closest_approach_ut, closest_approach_distance) =
                orbit.find_closest_approach(body_orbit, start_ut, look_ahead);
            if closest_approach_distance > self.sphere_of_influence {
                return None;
            }
            closest_approach_ut
        } else {
            let mut earliest_closest_approach_ut: Option<f64> = None;

            let cycles = ((look_ahead / orbit.period()).max(0.1).ceil() as u32) - 1;
            'per_cycle_search: for i in 0..(cycles + 1) {
                let cycle_look_ahead = if i == cycles && cycles > 1 {
                    look_ahead % orbit.period()
                } else {
                    orbit.period()
                };

                let (closest_approach_ut, closest_approach_distance) = orbit.find_closest_approach(
                    body_orbit,
                    start_ut + orbit.period() * i as f64,
                    cycle_look_ahead,
                );

                if closest_approach_distance <= self.sphere_of_influence {
                    earliest_closest_approach_ut = Some(closest_approach_ut);
                    break 'per_cycle_search;
                }
            }

            earliest_closest_approach_ut?
        };

        let start_position = orbit.position_at_ut(start_ut);
        let body_start_position = body_orbit.position_at_ut(start_ut);
        let start_distance = (start_position - body_start_position).magnitude();
        if start_distance <= self.sphere_of_influence {
            return Some(start_ut);
        }

        let mut bisect_start = start_ut;
        let mut bisect_end = closest_approach_ut;
        for _ in 0..30 {
            let middle = (bisect_start + bisect_end) / 2.0;
            let middle_position = orbit.position_at_ut(middle);
            let middle_body_position = body_orbit.position_at_ut(middle);
            let middle_distance = (middle_position - middle_body_position).magnitude();

            if middle_distance <= self.sphere_of_influence {
                bisect_end = middle;
            } else {
                bisect_start = middle;
            }
        }

        Some(bisect_end)
    }

    pub fn circular_orbit_speed(&self, radius: f64) -> f64 {
        (self.gravitational_parameter / radius).sqrt()
    }

    pub fn orbital_speed_from_sma_and_radius(&self, sma: f64, radius: f64) -> f64 {
        (self.gravitational_parameter * (2.0 / radius - 1.0 / sma)).sqrt()
    }

    pub fn mean_motion_of_sma(&self, sma: f64) -> Rad<f64> {
        Rad((self.gravitational_parameter / sma.powi(3).abs()).sqrt())
    }

    pub fn orbital_period_of_sma(&self, sma: f64) -> f64 {
        Rad::full_turn() / self.mean_motion_of_sma(sma)
    }

    pub fn sma_of_orbital_period(&self, period: f64) -> f64 {
        ((period.powi(2) * self.gravitational_parameter) / TAU.powi(2)).cbrt()
    }
}

#[derive(Debug, Clone)]
pub struct OrbitalStateChange {
    pub body: LocalBody,
    pub ut: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

impl OrbitalStateChange {
    pub async fn from_node(node: &Node) -> Result<Self> {
        let non_local_orbit = node.get_orbit().await?;
        let orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
        let ut = node.get_ut().await?;

        Ok(Self {
            body: Box::into_inner(orbit.body.clone()),
            ut,
            position: orbit.position_at_ut(ut),
            velocity: orbit.velocity_at_ut(ut)
                + orbit
                    .prograde_normal_radial_to_absolute(ut, node.burn_vector(None).await?.into()),
        })
    }

    pub fn orbit(&self) -> LocalOrbit {
        self.body
            .create_orbit_from_state_vectors(self.ut, self.position, self.velocity)
    }
}

#[derive(Debug, Clone)]
pub struct OrbitalTrajectory {
    pub state_changes: Vec<OrbitalStateChange>,
}

impl OrbitalTrajectory {
    pub fn state_change_at_ut(&self, ut: f64) -> &OrbitalStateChange {
        assert!(!self.state_changes.is_empty(), "trajectory is empty");

        for i in 0..self.state_changes.len() {
            let current = self.state_changes.get(i).unwrap();
            if let Some(next) = self.state_changes.get(i + 1) {
                if next.ut > ut {
                    return current;
                }
            }
        }

        self.state_changes.last().unwrap()
    }

    pub fn orbit_at_ut(&self, ut: f64) -> LocalOrbit {
        self.state_change_at_ut(ut).orbit()
    }

    pub fn prune(&mut self, latest_ut: f64) {
        self.state_changes
            .retain(|state_change| state_change.ut <= latest_ut);
    }

    pub fn add_local_node(&mut self, local_node: LocalNode) {
        let orbit = self.orbit_at_ut(local_node.ut);

        self.prune(local_node.ut);
        self.state_changes.push(OrbitalStateChange {
            body: Box::into_inner(orbit.body.clone()),
            ut: local_node.ut,
            position: orbit.position_at_ut(local_node.ut),
            velocity: orbit.velocity_at_ut(local_node.ut)
                + orbit.prograde_normal_radial_to_absolute(local_node.ut, local_node.burn_vector),
        });
    }

    pub async fn add_node(&mut self, node: &Node) -> Result<()> {
        self.add_local_node(LocalNode::from_node(node).await?);

        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LocalNode {
    pub ut: f64,
    pub burn_vector: Vector3<f64>,
}

impl LocalNode {
    pub async fn from_node(node: &Node) -> Result<Self> {
        Ok(Self {
            ut: node.get_ut().await?,
            burn_vector: node.burn_vector(None).await?.into(),
        })
    }

    pub async fn from_all_nodes(vessel: &Vessel) -> Result<Vec<Self>> {
        let nodes = vessel.get_control().await?.get_nodes().await?;
        let mut local_nodes = Vec::<Self>::with_capacity(nodes.len());
        for node in nodes {
            local_nodes.push(Self::from_node(&node).await?);
        }
        Ok(local_nodes)
    }

    pub async fn to_node(self, vessel: &Vessel) -> Result<Node> {
        Ok(vessel
            .get_control()
            .await?
            .add_node(
                self.ut,
                self.burn_vector.x as f32,
                self.burn_vector.y as f32,
                self.burn_vector.z as f32,
            )
            .await?)
    }
}

#[derive(Debug, Clone)]
pub struct LocalUniverse {
    pub bodies: BTreeMap<String, LocalBody>,
}

impl LocalUniverse {
    pub async fn create(space_center: &SpaceCenter) -> Result<Self> {
        let mut bodies = BTreeMap::<String, LocalBody>::new();

        for (name, body) in space_center.get_bodies().await? {
            bodies.insert(name, LocalBody::from_celestial_body(&body).await?);
        }

        Ok(Self { bodies })
    }

    pub async fn get_local_body(&self, body: &CelestialBody) -> Result<&LocalBody> {
        self.bodies
            .get(&body.get_name().await?)
            .ok_or(anyhow!("body doesn't exist"))
    }

    pub fn get_child_bodies(&self, parent_body: &LocalBody) -> Vec<LocalBody> {
        self.bodies
            .iter()
            .filter_map(|(_, other_body)| {
                let other_orbit = other_body.orbit.as_ref()?;

                if other_orbit.body.name == parent_body.name {
                    Some(other_body.clone())
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn fill_orbital_trajectory(&self, trajectory: &mut OrbitalTrajectory, end_ut: f64) {
        assert!(!trajectory.state_changes.is_empty(), "trajectory is empty");

        if trajectory.state_changes.last().unwrap().ut >= end_ut {
            return;
        }

        'trajectory_append: loop {
            let latest = trajectory.state_changes.last().unwrap();
            let orbit = latest.orbit();

            let exit_ut = orbit.find_soi_exit(latest.ut).unwrap_or(f64::INFINITY);

            let mut earliest_soi_enter: Option<(f64, LocalBody)> = None;
            for child_body in self.get_child_bodies(&orbit.body) {
                if let Some(soi_enter_ut) = child_body.find_soi_enter(
                    &orbit,
                    latest.ut,
                    earliest_soi_enter
                        .as_ref()
                        .map(|(ut, _)| *ut)
                        .unwrap_or(end_ut)
                        - latest.ut,
                ) {
                    earliest_soi_enter = Some((soi_enter_ut, child_body))
                }
            }

            if let Some((enter_ut, entered_body)) = earliest_soi_enter {
                if enter_ut < exit_ut {
                    let entered_body_orbit = entered_body.orbit.as_ref().unwrap();

                    let body_position = entered_body_orbit.position_at_ut(enter_ut);
                    let body_velocity = entered_body_orbit.velocity_at_ut(enter_ut);

                    let position = orbit.position_at_ut(enter_ut);
                    let velocity = orbit.velocity_at_ut(enter_ut);

                    trajectory.state_changes.push(OrbitalStateChange {
                        body: entered_body,
                        ut: enter_ut,
                        position: position - body_position,
                        velocity: velocity - body_velocity,
                    });

                    continue 'trajectory_append;
                }
            }

            if exit_ut <= end_ut {
                let body_orbit = orbit.body.orbit.as_ref().unwrap();

                let body_position = body_orbit.position_at_ut(exit_ut);
                let body_velocity = body_orbit.velocity_at_ut(exit_ut);

                let relative_position = orbit.position_at_ut(exit_ut);
                let relative_velocity = orbit.velocity_at_ut(exit_ut);

                trajectory.state_changes.push(OrbitalStateChange {
                    body: Box::into_inner(body_orbit.body.clone()),
                    ut: exit_ut,
                    position: body_position + relative_position,
                    velocity: body_velocity + relative_velocity,
                });

                continue 'trajectory_append;
            }

            break 'trajectory_append;
        }
    }
}
