//! Room terrain — walls block movement; swamp raises move fatigue; roads lower it. Combat-agnostic
//! (ADR 0033, extracted from `screeps-combat-engine`; roads added in ADR 0033 M3).

use crate::constants::{FATIGUE_RATE_PLAIN, FATIGUE_RATE_ROAD, FATIGUE_RATE_SWAMP};
use std::collections::HashSet;

/// Room terrain — defaults to all-plain, no roads.
#[derive(Clone, Debug, Default)]
pub struct SimTerrain {
    pub walls: HashSet<(u8, u8)>,
    pub swamps: HashSet<(u8, u8)>,
    /// Road tiles (a `road` structure occupies the tile). A road overrides the underlying terrain's
    /// fatigue rate to 1 (engine `movement.js:211` — the road check runs *after* swamp and wins).
    pub roads: HashSet<(u8, u8)>,
}

impl SimTerrain {
    /// Whether movement is blocked at `(x, y)`. A natural wall blocks. NOTE: the engine
    /// (`checkObstacleAtXY`, `movement.js:37`) treats a wall as passable if a road sits on it
    /// (`wall && !hasRoad`), but Screeps forbids building a road construction site on a natural wall,
    /// so that combination is unreachable in real rooms and in generated scenarios — we keep walls
    /// unconditionally blocking (roads price *walkable* terrain only) and do not model the divergence.
    pub fn is_wall(&self, x: u8, y: u8) -> bool {
        self.walls.contains(&(x, y))
    }

    /// Fatigue added per non-MOVE/non-CARRY part (and per loaded-CARRY unit) for a step onto this
    /// tile: road 1, else swamp 10, else plain 2. Road-wins-over-swamp matches the engine's
    /// `execute` order (`movement.js:204-213`: default 2 → swamp 10 → road 1).
    pub fn fatigue_rate(&self, x: u8, y: u8) -> u32 {
        if self.roads.contains(&(x, y)) {
            FATIGUE_RATE_ROAD
        } else if self.swamps.contains(&(x, y)) {
            FATIGUE_RATE_SWAMP
        } else {
            FATIGUE_RATE_PLAIN
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn road_overrides_swamp_and_plain() {
        let mut t = SimTerrain::default();
        assert_eq!(t.fatigue_rate(10, 10), FATIGUE_RATE_PLAIN, "bare tile is plain");
        t.swamps.insert((10, 10));
        assert_eq!(t.fatigue_rate(10, 10), FATIGUE_RATE_SWAMP, "swamp raises the rate");
        t.roads.insert((10, 10)); // a road on the swamp
        assert_eq!(t.fatigue_rate(10, 10), FATIGUE_RATE_ROAD, "a road on a swamp lowers it to 1");
    }
}
