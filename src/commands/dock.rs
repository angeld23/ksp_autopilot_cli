use anyhow::Result;
use cgmath::{Deg, InnerSpace, Vector3};
use clap::Args;
use krpc_client::services::space_center::DockingPort;
use log::warn;

use crate::{docking::DockingDescriptor, flight_computer::FlightComputer};

#[derive(Debug, Args)]
pub struct DockArgs {
    /// The maximum speed the vessel moves at when maneuvering towards the initial approach point.
    #[clap(long, short, default_value_t = 2.0)]
    alignment_speed: f64,
    /// The distance from the target port to start the approach at.
    #[clap(long, short = 'd', default_value_t = 15.0)]
    pre_approach_distance: f64,
    /// The speed at which the vessel moves toward the docking port.
    #[clap(long, short = 's', default_value_t = 1.0)]
    approach_speed: f64,
    /// The distance from the target port at which to change to `slow_approach_speed`.
    #[clap(long, short = 'D', default_value_t = 3.0)]
    slow_approach_distance: f64,
    /// The speed the vessel changes to when it is within `slow_approach_distance` of the target port.
    #[clap(long, short = 'S', default_value_t = 0.25)]
    slow_approach_speed: f64,
    /// The desired roll (in degrees) relative to the target port.
    #[clap(long, short, default_value_t = 0.0)]
    roll: f64,
}

pub async fn dock_command(computer: &FlightComputer, args: &DockArgs) -> Result<()> {
    if let Some(target_port) = computer.space_center.get_target_docking_port().await? {
        let parts = computer.vessel.get_parts().await?;
        let controlling = parts.get_controlling().await?;

        let vessel_frame = computer.vessel.get_reference_frame().await?;
        let controlling_pos: Vector3<f64> = controlling.position(&vessel_frame).await?.into();

        let port = if let Some(port) = controlling.get_docking_port().await? {
            port
        } else {
            let mut closest_port: Option<DockingPort> = None;
            let mut closest_port_distance = f64::MAX;
            for port in computer
                .vessel
                .get_parts()
                .await?
                .get_docking_ports()
                .await?
            {
                let port_pos: Vector3<f64> = port.position(&vessel_frame).await?.into();
                let distance = (port_pos - controlling_pos).magnitude();
                if distance < closest_port_distance {
                    closest_port = Some(port);
                    closest_port_distance = distance;
                }
            }

            if let Some(closest_port) = closest_port {
                closest_port
            } else {
                warn!("no docking port on your vessel!");
                return Ok(());
            }
        };

        computer
            .dock(
                &port,
                &target_port,
                DockingDescriptor {
                    alignment_speed: args.alignment_speed,
                    pre_approach_distance: args.pre_approach_distance,
                    approach_speed: args.approach_speed,
                    slow_approach_distance: args.slow_approach_distance,
                    slow_approach_speed: args.slow_approach_speed,
                    roll: Deg(args.roll).into(),
                },
            )
            .await?;
    } else {
        warn!("no docking port targeted");
    }

    Ok(())
}
