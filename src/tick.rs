//! The movement tick: resolve contested moves, apply positions + fatigue, relocate edge-crossers,
//! advance the clock. This is the movement half of a Screeps tick — shared by every sim layer (a
//! combat / economy layer calls this at the movement point of its own pipeline, after it has
//! accumulated its own pre-move effects and before it nets them). It is byte-identical to the
//! movement phases of the combat engine's former `resolve_tick` (ADR 0033); the combat netting
//! (damage → deaths → structures) that used to interleave here now lives in the combat layer, which
//! is sound because per-creep *position* (movement) and *hits* (combat) are independent.

use crate::intents::MoveIntents;
use crate::movement::{is_edge, resolve_moves_with_pulls};
use crate::world::{CreepId, MovementState};
use screeps::Position;
use std::collections::HashMap;

/// Outcome of a movement tick.
#[derive(Clone, Debug, Default)]
pub struct MovementReport {
    /// The tick this report is for (pre-increment).
    pub tick: u32,
    /// Creeps that moved this tick → their new (post-contention, pre-edge-exit) position.
    pub moved: HashMap<CreepId, Position>,
}

/// Resolve one movement tick over `world`: contention → apply move + fatigue → edge-exit → tick++.
pub fn resolve_movement(world: &mut MovementState, intents: &MoveIntents) -> MovementReport {
    let report_tick = world.tick;

    // Contention (engine `movement.check`), using tick-start positions.
    let new_positions = resolve_moves_with_pulls(world, &intents.moves, &intents.pulls);

    // Apply movement + fatigue (engine `movement.execute`): move, add move-fatigue (0 on a room-edge
    // tile), then regen (−2 × MOVE parts). Disjoint field borrows (terrain + rooms) so the &mut creep
    // loop can read room-aware terrain.
    let default_terrain = &world.terrain;
    let rooms = &world.rooms;
    for c in world.creeps.iter_mut() {
        if let Some(&np) = new_positions.get(&c.id) {
            c.pos = np;
            let (x, y) = (np.x().u8(), np.y().u8());
            let move_fatigue = if is_edge(x, y) {
                0
            } else {
                // Room-aware: fatigue from the DESTINATION room's terrain, weighted by the creep's
                // full move-weight (structural parts + loaded CARRY).
                let terrain = rooms.get(&np.room_name()).unwrap_or(default_terrain);
                c.fatigue_weight() * terrain.fatigue_rate(x, y)
            };
            c.fatigue += move_fatigue;
        }
        c.fatigue = c.fatigue.saturating_sub(c.body.fatigue_clear());
    }

    // Edge-exit relocation (engine `creeps/tick.js:52-78` + `global.js:42`): a non-NPC creep standing
    // on an exit tile after movement crosses to the adjacent room's mirror tile (one step across that
    // edge, which `checked_add` computes). Occupancy-blind and same-tick, matching the real engine.
    let npc_owners = &world.npc_owners;
    for c in world.creeps.iter_mut() {
        if npc_owners.contains(&c.owner) {
            continue;
        }
        let (x, y) = (c.pos.x().u8(), c.pos.y().u8());
        let offset = if x == 0 {
            (-1, 0)
        } else if y == 0 {
            (0, -1)
        } else if x == 49 {
            (1, 0)
        } else if y == 49 {
            (0, 1)
        } else {
            continue; // not on an exit tile
        };
        if let Ok(np) = c.pos.checked_add(offset) {
            c.pos = np;
        }
    }

    world.tick += 1;
    MovementReport {
        tick: report_tick,
        moved: new_positions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyPartDef, SimBody};
    use crate::world::SimCreep;
    use screeps::{Direction, Part, Position, RoomCoordinate, RoomName};

    fn pos(x: u8, y: u8) -> Position {
        let room: RoomName = "W1N1".parse().unwrap();
        Position::new(RoomCoordinate::new(x).unwrap(), RoomCoordinate::new(y).unwrap(), room)
    }
    fn creep(body: &[Part], carry_used: u32) -> SimCreep {
        SimCreep {
            id: 1,
            owner: 0,
            pos: pos(25, 25),
            body: SimBody::new(body.iter().map(|&p| BodyPartDef::new(p)).collect()),
            fatigue: 0,
            carry_used,
        }
    }
    fn step_right(mut world: MovementState) -> u32 {
        let mut mv = MoveIntents::new();
        mv.set_move(1, Direction::Right);
        resolve_movement(&mut world, &mv);
        world.creeps[0].fatigue
    }

    #[test]
    fn road_step_accrues_less_fatigue_than_plain() {
        // Weight-2 body (2×ATTACK + 1×MOVE): plain rate 2 → +4, regen 2 → net 2.
        let plain = step_right(MovementState {
            creeps: vec![creep(&[Part::Attack, Part::Attack, Part::Move], 0)],
            ..Default::default()
        });
        assert_eq!(plain, 2, "plain step: 2×2 − 2 = 2");

        // Same body over a road at the destination (26,25): rate 1 → +2, regen 2 → net 0.
        let mut roaded = MovementState {
            creeps: vec![creep(&[Part::Attack, Part::Attack, Part::Move], 0)],
            ..Default::default()
        };
        roaded.terrain.roads.insert((26, 25));
        assert_eq!(step_right(roaded), 0, "road step: 2×1 − 2 = 0 (roads reduce fatigue)");
    }

    #[test]
    fn loaded_carry_accrues_fatigue_an_empty_hauler_does_not() {
        // Hauler [2×CARRY, 1×MOVE]: structural weight 0. Empty → 0 fatigue even after a plain step.
        let empty = step_right(MovementState {
            creeps: vec![creep(&[Part::Carry, Part::Carry, Part::Move], 0)],
            ..Default::default()
        });
        assert_eq!(empty, 0, "an empty hauler accrues no move-fatigue");

        // Loaded with 100 (2 CARRY units): weight 2, plain rate 2 → +4, regen 2 → net 2.
        let loaded = step_right(MovementState {
            creeps: vec![creep(&[Part::Carry, Part::Carry, Part::Move], 100)],
            ..Default::default()
        });
        assert_eq!(loaded, 2, "a fully loaded hauler accrues fatigue (2×2 − 2 = 2)");
    }
}
