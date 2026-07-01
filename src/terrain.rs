//! Room terrain — walls block movement; swamp raises move fatigue. Combat-agnostic (ADR 0033,
//! extracted from `screeps-combat-engine`). Roads (which lower fatigue) arrive in ADR 0033
//! SHARED-FIX-1.

use crate::constants::{FATIGUE_RATE_PLAIN, FATIGUE_RATE_SWAMP};
use std::collections::HashSet;

/// Room terrain — defaults to all-plain.
#[derive(Clone, Debug, Default)]
pub struct SimTerrain {
    pub walls: HashSet<(u8, u8)>,
    pub swamps: HashSet<(u8, u8)>,
}

impl SimTerrain {
    pub fn is_wall(&self, x: u8, y: u8) -> bool {
        self.walls.contains(&(x, y))
    }
    /// Fatigue added per non-MOVE/non-CARRY part for a step onto this tile.
    pub fn fatigue_rate(&self, x: u8, y: u8) -> u32 {
        if self.swamps.contains(&(x, y)) {
            FATIGUE_RATE_SWAMP
        } else {
            FATIGUE_RATE_PLAIN
        }
    }
}
