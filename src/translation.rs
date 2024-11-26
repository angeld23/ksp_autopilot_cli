use anyhow::Result;
use cgmath::{vec3, InnerSpace, Quaternion, Vector3, Zero};
use derive_more::*;
use krpc_client::services::space_center::{ReferenceFrame, Vessel};

use crate::reference_frame_or_vessel_frame;

#[derive(Debug, Clone, Copy, PartialEq, IsVariant, Unwrap)]
pub enum TranslationTarget {
    Position(Vector3<f64>),
    Velocity(Vector3<f64>),
    Throttle(Vector3<f64>),
}

pub struct TranslationController {
    pub target: TranslationTarget,
    pub reference_frame: Option<ReferenceFrame>,
    pub target_position_max_speed: f64,
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
        //let difference = self.target - position;

        let throttle = match self.target {
            TranslationTarget::Position(target_position) => todo!(),
            TranslationTarget::Velocity(target_velocity) => target_velocity - velocity,
            TranslationTarget::Throttle(throttle) => throttle,
        };

        control
            .set_forward(throttle.dot(forward_direction) as f32)
            .await?;
        control
            .set_right(throttle.dot(right_direction) as f32)
            .await?;
        control.set_up(throttle.dot(up_direction) as f32).await?;

        Ok(())
    }

    pub async fn reset(&mut self, vessel: &Vessel) -> Result<()> {
        if self.enabled {
            self.enabled = false;

            let control = vessel.get_control().await?;
            control.set_forward(0.0).await?;
            control.set_right(0.0).await?;
            control.set_up(0.0).await?;

            self.target = TranslationTarget::Throttle(Vector3::zero());
            self.reference_frame = None;
        }

        Ok(())
    }
}
