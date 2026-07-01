//! The per-tick movement world — terrain + creeps + edge-exit exemptions. Combat-agnostic
//! (ADR 0033, extracted from `screeps-combat-engine`'s `CombatWorld`). The combat sim composes
//! this as `CombatWorld { sim: MovementState, towers, structures, controllers, safe_mode_owner }`.

use crate::body::SimBody;
use crate::terrain::SimTerrain;
use screeps::{Position, RoomName};
use std::collections::{HashMap, HashSet};

/// A faction identity (self-play: side 0 vs side 1; NPCs get their own ids).
pub type PlayerId = u8;
/// Stable per-engagement creep id (minted by the scenario; NOT a game `ObjectId`).
pub type CreepId = u32;
/// Stable per-engagement structure id.
pub type StructureId = u32;

/// A creep in the sim.
#[derive(Clone, Debug)]
pub struct SimCreep {
    pub id: CreepId,
    pub owner: PlayerId,
    pub pos: Position,
    pub body: SimBody,
    /// Fatigue carried into this tick; the creep may move only when it is 0.
    pub fatigue: u32,
    /// Total resources aboard (all types summed, engine `_.sum(creep.store)`). Raises move-fatigue
    /// via the loaded-CARRY weight ([`SimBody::carry_weight`]). Combat creeps carry nothing → `0`.
    pub carry_used: u32,
}

impl SimCreep {
    pub fn is_alive(&self) -> bool {
        self.body.is_alive()
    }

    /// The full move-fatigue *weight*: structural non-MOVE/non-CARRY parts plus the loaded-CARRY
    /// units for the resources aboard (engine `movement.js:119-123,237-238`). This is the multiplier
    /// on terrain fatigue rate per step, and (min 1) the contention rate4 denominator.
    pub fn fatigue_weight(&self) -> u32 {
        self.body.fatigue_weight() + self.body.carry_weight(self.carry_used)
    }
}

/// One room's movement state for a tick: the default/common terrain plus per-room overrides
/// (multi-room scenarios), the creeps, and the NPC edge-exit exemption set.
#[derive(Clone, Debug, Default)]
pub struct MovementState {
    pub tick: u32,
    /// The default/common room terrain — also the terrain of a single-room scenario. Per-room
    /// overrides live in [`rooms`](MovementState::rooms); read via [`terrain_for`](MovementState::terrain_for).
    pub terrain: SimTerrain,
    /// Per-room terrain overrides for multi-room scenarios. A room absent here uses [`terrain`](MovementState::terrain).
    pub rooms: HashMap<RoomName, SimTerrain>,
    pub creeps: Vec<SimCreep>,
    /// Owners that do NOT auto-exit at a room edge — the engine's NPC exemption (`creeps/tick.js:52`
    /// skips Source Keeper / Invader). Default empty (self-play: every creep auto-exits).
    pub npc_owners: HashSet<PlayerId>,
}

impl MovementState {
    pub fn living_creeps(&self) -> impl Iterator<Item = &SimCreep> {
        self.creeps.iter().filter(|c| c.is_alive())
    }

    /// Terrain for `room` — the per-room override if one exists, else the default [`terrain`](MovementState::terrain).
    /// All movement/fatigue/wall checks go through this so the sim is multi-room-correct.
    pub fn terrain_for(&self, room: RoomName) -> &SimTerrain {
        self.rooms.get(&room).unwrap_or(&self.terrain)
    }

    /// Mutable per-room terrain override for `room` (creating an empty one if absent) — used by the
    /// multi-room ScenarioBuilder to give distinct rooms distinct terrain.
    pub fn terrain_mut(&mut self, room: RoomName) -> &mut SimTerrain {
        self.rooms.entry(room).or_default()
    }
}
