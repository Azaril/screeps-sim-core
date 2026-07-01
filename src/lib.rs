//! # screeps-sim-core
//!
//! The shared, combat-agnostic Screeps movement/world **simulation kernel**: terrain, the creep
//! body model, the same-tile contention resolver, fatigue, edge-exit, the per-tick movement world,
//! and the `Simulation` layering contract. It is the JS-free *mechanism* layer that every sim layer
//! builds on — the combat sim (`screeps-combat-engine`), the rover benchmark (`screeps-rover-eval`),
//! and any future layer (economy, lifecycle). A layer owns its own world (containing a
//! [`MovementState`]) and intents (embedding [`MoveIntents`]), and calls [`resolve_movement`] at the
//! movement point of its own tick pipeline. No combat concept lives here.
//!
//! Extracted from `screeps-combat-engine` (ADR 0033). Ground truth is the cloned engine at
//! `C:\code\screeps-engine`; every formula cites the engine source it ports.

pub mod body;
pub mod constants;
pub mod intents;
pub mod movement;
pub mod sim;
pub mod terrain;
pub mod tick;
pub mod world;

pub use body::{BodyPartDef, BoostTier, SimBody};
pub use intents::MoveIntents;
pub use movement::{is_edge, resolve_moves, resolve_moves_with_pulls, step};
pub use sim::{MovementSim, Simulation};
pub use terrain::SimTerrain;
pub use tick::{resolve_movement, MovementReport};
pub use world::{CreepId, MovementState, PlayerId, SimCreep, StructureId};
