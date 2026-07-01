//! Movement intents — the base action vocabulary every sim layer includes. A layer's own intent
//! type *embeds* `MoveIntents` (e.g. `CombatIntents { moves: MoveIntents, .. }`), so the movement
//! mechanism only ever sees the movement part, and a movement-only decision routine literally has no
//! field in which to place a combat action (ADR 0033).

use crate::world::CreepId;
use screeps::Direction;
use std::collections::HashMap;

/// Per-tick movement intents: each creep's desired step + optional pull pairing.
#[derive(Clone, Debug, Default)]
pub struct MoveIntents {
    /// Per-creep desired direction.
    pub moves: HashMap<CreepId, Direction>,
    /// Pull pairings: puller → the creep it drags (engine `pull`); the dragged creep follows the
    /// puller into its vacated tile even with no MOVE part / nonzero fatigue.
    pub pulls: HashMap<CreepId, CreepId>,
    /// Optional per-creep "why" tag for introspection / replay.
    pub reasons: HashMap<CreepId, String>,
}

impl MoveIntents {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn set_move(&mut self, creep: CreepId, dir: Direction) -> &mut Self {
        self.moves.insert(creep, dir);
        self
    }
    pub fn set_pull(&mut self, puller: CreepId, target: CreepId) -> &mut Self {
        self.pulls.insert(puller, target);
        self
    }
    pub fn set_reason(&mut self, creep: CreepId, reason: impl Into<String>) -> &mut Self {
        self.reasons.insert(creep, reason.into());
        self
    }
}
