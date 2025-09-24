use anyhow::Result;
use cgmath::{Angle, Deg, InnerSpace, Rad};
use log::debug;

use crate::{
    flight_computer::FlightComputer,
    maneuver::{
        node_change_opposite_radius, node_change_orbit_normal, node_change_velocity,
        node_circularize_at_apsis, node_hohmann_transfer, node_tune_closest_approach,
    },
    orbital_mechanics::LocalOrbit,
};

#[derive(Debug, Clone, Copy)]
pub struct RendevousDescriptor {
    pub max_orbits: f64,
    pub distance: f64,
    pub tune_closest_approach: bool,
    pub match_velocity: bool,
}

impl Default for RendevousDescriptor {
    fn default() -> Self {
        Self {
            max_orbits: 5.0,
            distance: 100.0,
            tune_closest_approach: true,
            match_velocity: true,
        }
    }
}

impl FlightComputer {
    pub async fn rendevous(
        &self,
        target_orbit: &LocalOrbit,
        descriptor: RendevousDescriptor,
    ) -> Result<()> {
        let non_local_orbit = self.vessel.get_orbit().await?;
        let mut orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

        let RendevousDescriptor {
            max_orbits,
            distance,
            tune_closest_approach,
            match_velocity,
        } = descriptor;
        let max_orbits = max_orbits.max(1.0);

        debug!("beginning rendevous sequence...");

        let max_angle = Rad::from(Deg(0.25));
        if orbit.normal().angle(target_orbit.normal()) > max_angle {
            debug!("inclinations do not match. correcting...");
            self.execute_node(
                &node_change_orbit_normal(
                    &orbit,
                    target_orbit.normal(),
                    true,
                    self.space_center.get_ut().await?,
                )
                .to_node(&self.vessel)
                .await?,
            )
            .await?;
        }

        orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;

        let current_ut = self.space_center.get_ut().await?;
        let next_periapsis_ut = orbit.most_recent_periapsis_ut(current_ut) + orbit.period();
        let initial_node = node_hohmann_transfer(&orbit, target_orbit, distance, current_ut);
        let initial_orbits_after_periapsis = (initial_node.ut - next_periapsis_ut) / orbit.period();
        let node = if initial_orbits_after_periapsis <= max_orbits {
            initial_node
        } else {
            debug!("current rendevous transfer would be in {:.1} orbit(s), which is above the provided maximum of {:.1}. entering a better resonant orbit...", (initial_node.ut - current_ut) / orbit.period(), max_orbits);
            let initial_node_mean_anomaly_diff = (target_orbit.mean_anomaly_at_ut(initial_node.ut)
                - orbit.mean_anomaly_at_ut(initial_node.ut))
            .normalize_signed();
            let periapsis_mean_anomaly_diff = target_orbit.mean_anomaly_at_ut(next_periapsis_ut); // our mean anomaly at our periapsis is obviously zero so no subtraction needed

            let required_diff_change =
                (initial_node_mean_anomaly_diff - periapsis_mean_anomaly_diff).normalize();
            // let current_diff_change_rate = target_orbit.mean_motion() - orbit.mean_motion();
            let required_diff_change_rate = required_diff_change / (max_orbits * orbit.period());
            let required_mean_motion = target_orbit.mean_motion() - required_diff_change_rate;
            let required_period = Rad::full_turn() / required_mean_motion;

            let sma = orbit.body.sma_of_orbital_period(required_period);
            self.execute_node(
                &node_change_opposite_radius(&orbit, sma, next_periapsis_ut)
                    .to_node(&self.vessel)
                    .await?,
            )
            .await?;

            orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
            self.execute_node(
                &node_circularize_at_apsis(&orbit, false, self.space_center.get_ut().await?)
                    .to_node(&self.vessel)
                    .await?,
            )
            .await?;

            orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
            node_hohmann_transfer(
                &orbit,
                target_orbit,
                distance,
                self.space_center.get_ut().await?,
            )
        };

        debug!(
            "transfer node in {:.1} orbit(s).",
            (node.ut - self.space_center.get_ut().await?) / orbit.period()
        );
        self.execute_node(&node.to_node(&self.vessel).await?)
            .await?;

        orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
        if tune_closest_approach {
            debug!("tuning closest approach...");
            self.execute_node(
                &node_tune_closest_approach(
                    &orbit,
                    target_orbit,
                    node.ut + orbit.period() / 3.0,
                    distance,
                    100.0,
                )
                .to_node(&self.vessel)
                .await?,
            )
            .await?;
        }

        orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
        if match_velocity {
            debug!("matching velocities at intercept...");
            let (closest_approach_ut, _) =
                orbit.find_closest_approach(target_orbit, node.ut, orbit.after_node(node).period());
            orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
            self.execute_node(
                &node_change_velocity(
                    &orbit,
                    closest_approach_ut,
                    target_orbit.velocity_at_ut(closest_approach_ut),
                )
                .to_node(&self.vessel)
                .await?,
            )
            .await?;
        }

        let current_ut = self.space_center.get_ut().await?;
        orbit = LocalOrbit::from_orbit(&non_local_orbit).await?;
        debug!(
            "rendevous complete, arrived {:.1}m from target",
            (orbit.position_at_ut(current_ut) - target_orbit.position_at_ut(current_ut))
                .magnitude()
        );

        Ok(())
    }
}
