//! The sim-layer contract. A [`Simulation`] binds a world, an intent (action) vocabulary, and how a
//! tick resolves. Movement is the base layer; combat / economy layers add their own world + intents
//! and orchestrate around the shared movement tick ([`resolve_movement`]). The `Intents` associated
//! type is what makes a movement-only sim unable to express combat actions — and vice-versa — at the
//! type level (ADR 0033).

use crate::intents::MoveIntents;
use crate::tick::{resolve_movement, MovementReport};
use crate::world::MovementState;

/// A simulation layer: a (world, intents, report) triple with a tick resolver. Add a layer by
/// implementing this over a world that contains a [`MovementState`], an intents type that embeds
/// [`MoveIntents`], and a `step` that calls [`resolve_movement`] at the movement point of its
/// pipeline. No change to this crate is needed to add a layer.
pub trait Simulation {
    /// The world this layer resolves over. Contains a [`MovementState`]; higher layers add their own
    /// state (towers, sources, …) alongside it.
    type World;
    /// The per-tick action vocabulary this layer resolves. A movement-only layer uses [`MoveIntents`];
    /// a combat layer uses a type that *embeds* `MoveIntents` plus its combat verbs.
    type Intents;
    /// The per-tick outcome.
    type Report;

    fn step(world: &mut Self::World, intents: &Self::Intents) -> Self::Report;
}

/// The base layer: pure movement, no combat. Its `Intents` is exactly [`MoveIntents`], so a decision
/// routine driving `MovementSim` can only ever emit movement actions.
pub struct MovementSim;

impl Simulation for MovementSim {
    type World = MovementState;
    type Intents = MoveIntents;
    type Report = MovementReport;

    fn step(world: &mut MovementState, intents: &MoveIntents) -> MovementReport {
        resolve_movement(world, intents)
    }
}
