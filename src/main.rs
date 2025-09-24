#![feature(box_into_inner, sort_floats)]
#![allow(dead_code, clippy::uninlined_format_args)]

use std::{process::exit, sync::Arc, time::Duration};

use anyhow::Result;
use cgmath::{prelude::*, Deg, Vector3};
use clap::{Parser, Subcommand};
use commands::{
    adjust_plane::{adjust_plane_command, AdjustPlaneArgs},
    ascent::{ascent_command, AscentCommandArgs},
    circularize::{circularize_command, CircularizeArgs},
    dock::{dock_command, DockArgs},
    execute_node::execute_node_command,
    land::{land_command, LandArgs},
    rendevous::{rendevous_command, RendevousArgs},
    return_to_parent::{return_to_parent_command, ReturnToParentArgs},
    transfer::{transfer_command, TransferArgs},
    tune_closest_approach::{tune_closest_approach_command, TuneClosestApproachArgs},
};
use flight_computer::FlightComputer;
use krpc_client::{
    services::{drawing::Drawing, mech_jeb::MechJeb, space_center::SpaceCenter},
    Client,
};
use orbital_mechanics::LocalUniverse;
use tokio::{spawn, sync::Mutex, time};
use translation::{TranslationController, TranslationTarget};

use crate::commands::scripts::bsfp::{bsfp_command, BsfpArgs};

mod ascent;
mod commands;
mod docking;
mod execute_node;
mod flight_computer;
mod landing;
mod maneuver;
mod numerical_integration;
mod orbital_mechanics;
mod rendevous;
mod root_finding;
mod trajectory_prediction;
mod translation;
mod util;

/// angeld23's awesome autopilot program
#[derive(Debug, Parser)]
#[clap(name = "angel-os", version = "0.1.0", author = "angeld23")]
pub struct App {
    #[clap(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Execute the next node.
    ExecuteNode,
    /// Launch into orbit from the surface of a body.
    Ascent(AscentCommandArgs),
    /// Creates a node to adjust your orbital plane.
    AdjustPlane(AdjustPlaneArgs),
    /// Creates a node to perform a Hohmann transfer to a target.
    Transfer(TransferArgs),
    /// Creates a node to tune the closest approach to a target.
    TuneClosestApproach(TuneClosestApproachArgs),
    /// Creates a node to returns to the parent body.
    ReturnToParent(ReturnToParentArgs),
    /// Performs a landing onto the surface of the current body.
    Land(LandArgs),
    /// Creates a node to circularize an orbit (or change the opposite side's altitude to any other arbitrary value).
    Circularize(CircularizeArgs),
    /// Performs a rendevous to intercept with another vessel orbiting the same body.
    Rendevous(RendevousArgs),
    /// Automatically docks to another vessel.
    Dock(DockArgs),

    // scripts
    /// Lands a BSFP booster.
    Bsfp(BsfpArgs),
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::builder().format_timestamp(None).init();

    let client = Client::new("rust client", "127.0.0.1", 50000, 50001).await?;
    let space_center = Arc::new(SpaceCenter::new(client.clone()));
    let drawing = Arc::new(Drawing::new(client.clone()));
    let mech = Arc::new(MechJeb::new(client.clone()));
    let vessel = space_center.get_active_vessel().await?;
    let computer = Arc::new(FlightComputer {
        space_center: space_center.clone(),
        mech: mech.clone(),
        drawing: drawing.clone(),
        local_universe: LocalUniverse::create(&space_center).await?,
        vessel,
        translation_controller: Mutex::new(TranslationController {
            target: TranslationTarget::Acceleration(Vector3::zero()),
            reference_frame: None,
            vtol_enabled: false,
            vtol_max_tilt: Deg(30.0),
            rcs_enabled: false,
            enabled: false,
        }),
        auto_stage: true,
        execute_node_use_rcs: true,
        saved_control_config: Mutex::new(None),
    });

    {
        let computer = Arc::clone(&computer);

        let app = App::parse();
        spawn(async move {
            match app.command {
                Command::ExecuteNode => execute_node_command(&computer).await?,
                Command::Ascent(args) => ascent_command(&computer, &args).await?,
                Command::AdjustPlane(args) => adjust_plane_command(&computer, &args).await?,
                Command::Transfer(args) => transfer_command(&computer, &args).await?,
                Command::TuneClosestApproach(args) => {
                    tune_closest_approach_command(&computer, &args).await?
                }
                Command::ReturnToParent(args) => return_to_parent_command(&computer, &args).await?,
                Command::Land(args) => land_command(&computer, &args).await?,
                Command::Circularize(args) => circularize_command(&computer, &args).await?,
                Command::Rendevous(args) => rendevous_command(&computer, &args).await?,
                Command::Dock(args) => dock_command(&computer, &args).await?,
                // scripts
                Command::Bsfp(args) => bsfp_command(&computer, &args).await?,
            }

            time::sleep(Duration::from_secs_f64(0.5)).await;
            exit(0);

            #[allow(unreachable_code)]
            Ok::<(), anyhow::Error>(())
        });
    }

    let mut interval = time::interval(Duration::from_secs_f64(1.0 / 20.0));
    loop {
        interval.tick().await;
        computer.update().await?;
    }
}
