use anyhow::Result;

use crate::flight_computer::FlightComputer;

pub async fn execute_node_command(computer: &FlightComputer) -> Result<()> {
    computer.execute_next_node().await?;
    Ok(())
}
