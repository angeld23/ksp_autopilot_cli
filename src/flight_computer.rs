use core::f64;
use std::{sync::Arc, time::Duration};

use crate::orbital_mechanics::LocalUniverse;
use crate::translation::TranslationController;
use anyhow::Result;
use krpc_client::services::{
    drawing::Drawing,
    mech_jeb::MechJeb,
    space_center::{SpaceCenter, Vessel, VesselSituation},
};
use tokio::{sync::Mutex, time};

#[derive(Debug, Default, Clone, Copy)]
pub struct ControlConfig {
    pub rcs: bool,
    pub sas: bool,
}

pub struct FlightComputer {
    pub space_center: Arc<SpaceCenter>,
    pub mech: Arc<MechJeb>,
    pub drawing: Arc<Drawing>,
    pub local_universe: LocalUniverse,
    pub vessel: Vessel,
    pub translation_controller: Mutex<TranslationController>,
    pub auto_stage: bool,
    pub execute_node_use_rcs: bool,
    pub saved_control_config: Mutex<Option<ControlConfig>>,
}

impl FlightComputer {
    pub async fn update(&self) -> Result<()> {
        let control = self.vessel.get_control().await?;

        // let auto_pilot = self.vessel.get_auto_pilot().await?;
        let current_stage = control.get_current_stage().await?;
        let parts = self.vessel.get_parts().await?;
        let engines = parts.get_engines().await?;

        // auto-staging
        if self.auto_stage
            && !matches!(
                self.vessel.get_situation().await?,
                VesselSituation::PreLaunch
            )
            && current_stage > 0
            && !control.get_stage_lock().await?
        {
            let mut next_decoupled_engines_all_depleted = true;
            let mut engine_in_any_future_stage = false;
            for engine in engines.iter() {
                let part = engine.get_part().await?;
                let active_stage = part.get_stage().await?;
                let decouple_stage = part.get_decouple_stage().await?;
                let has_fuel = engine.get_has_fuel().await?;
                let name = part.get_name().await?;

                // this counts engines that never get activated via staging (their stage is -1), but that's fine
                if active_stage < current_stage {
                    engine_in_any_future_stage = true;
                }

                if decouple_stage == current_stage - 1 && has_fuel && name != "sepMotor1" {
                    next_decoupled_engines_all_depleted = false;
                }
            }

            if next_decoupled_engines_all_depleted && engine_in_any_future_stage {
                control.activate_next_stage().await?;
            }
        }

        self.translation_controller
            .lock()
            .await
            .update(&self.vessel)
            .await?;

        Ok(())
    }

    pub async fn save_control_config(&self) -> Result<()> {
        let control = self.vessel.get_control().await?;
        *self.saved_control_config.lock().await = Some(ControlConfig {
            rcs: control.get_rcs().await?,
            sas: control.get_sas().await?,
        });

        Ok(())
    }

    pub async fn load_control_config(&self) -> Result<()> {
        if let Some(config) = self.saved_control_config.lock().await.as_ref() {
            let control = self.vessel.get_control().await?;
            control.set_rcs(config.rcs).await?;
            control.set_sas(config.sas).await?;
        }

        Ok(())
    }

    pub async fn reset(&self) -> Result<()> {
        self.translation_controller
            .lock()
            .await
            .reset(&self.vessel)
            .await?;

        let reference_frame = self.vessel.get_reference_frame().await?;
        let control = self.vessel.get_control().await?;
        let auto_pilot = self.vessel.get_auto_pilot().await?;

        auto_pilot.set_reference_frame(&reference_frame).await?;
        auto_pilot.set_target_roll(f32::NAN).await?;
        auto_pilot.set_attenuation_angle((1.0, 1.0, 1.0)).await?;
        auto_pilot.disengage().await?;
        control.set_throttle(0.0).await?;

        self.load_control_config().await?;

        Ok(())
    }

    pub async fn wait_until(&self, ut: f64) -> Result<()> {
        let mut interval = time::interval(Duration::from_secs_f64(1.0 / 20.0));
        loop {
            interval.tick().await;
            if self.space_center.get_ut().await? >= ut {
                break;
            }
        }

        Ok(())
    }

    pub async fn wait(&self, time: f64) -> Result<()> {
        let start_ut = self.space_center.get_ut().await?;
        self.wait_until(start_ut + time).await
    }
}
