//! Same-tile movement-conflict resolution — the engine `movement.js check` port. This is where
//! kiting and squad-cohesion fidelity come from: a "move if the tile is free" toy model would hide
//! exactly the failure class the sim exists to surface (a creep that "sat idle" is usually a
//! movement-conflict loss, not a decision bug).
//!
//! Engine fidelity (`src/processor/intents/movement.js`, ground truth `C:\code\screeps-engine`):
//! - **Eligibility** (`canMove`, lines 11-14): a creep moves only if it has a working MOVE part AND
//!   its fatigue was 0 at tick start.
//! - **Same-tile contention** (`check`, lines 104-150): when >1 creep targets a tile, the winner is
//!   chosen by, in order, `rate1` (mutual-swap → 100, else how many movers want the creep's *current*
//!   tile), `rate2` (being pulled), `rate3` (pulling), `rate4 = move_rate / weight`; losers stay.
//! - **Pull** (`canMove`'s `_pulled` branch + rate2/rate3): a creep dragged by an adjacent, moving
//!   puller follows into the puller's vacated tile and is eligible even with **no MOVE part / nonzero
//!   fatigue** — how no-MOVE / under-MOVE compositions stay mobile. See [`resolve_moves_with_pulls`].
//! - **Obstacle + chain-block** (`checkObstacleAtXY` line 16-39 + `removeFromMatrix` line 154-165):
//!   a mover is stripped if its destination is a wall or holds a creep that is NOT itself moving
//!   (engine `!objects[i._id]`, line 22); stripping a mover recursively strips any mover that wanted
//!   the stripped creep's now-unvacated current tile — so a blocked front stops the whole column
//!   (the cohesion mechanic).
//!
//! **Room-edge crossing is not a move here** — a step off the room edge returns `None` (engine
//! `move.js:32` rejects it / `movement.js:88` clamps). The boundary cross is a separate **edge-exit
//! relocation** in [`crate::resolve_tick`]'s Phase D (engine `creeps/tick.js:52` + `global.js:42`):
//! a non-NPC creep standing on an exit tile is moved to the adjacent room's mirror tile.
//! **Not modelled yet:** roads (fatigue stays plain/swamp). Tracked in `AGENTS.md`.

use crate::world::*;
use screeps::{Direction, Position, RoomCoordinate};
use std::collections::{HashMap, HashSet};

/// (dx, dy) for a direction. Screeps y increases downward, so `Top` is `-y`.
fn dir_delta(dir: Direction) -> (i32, i32) {
    match dir {
        Direction::Top => (0, -1),
        Direction::TopRight => (1, -1),
        Direction::Right => (1, 0),
        Direction::BottomRight => (1, 1),
        Direction::Bottom => (0, 1),
        Direction::BottomLeft => (-1, 1),
        Direction::Left => (-1, 0),
        Direction::TopLeft => (-1, -1),
    }
}

/// One step from `pos` in `dir` **within the room**, or `None` if it would leave the room. Crossing a
/// room boundary is **not** a move intent: the engine rejects an off-edge move (`move.js:32`) and
/// clamps it (`movement.js:88`). A creep reaches the exit tile via in-room moves, then the separate
/// **edge-exit relocation** ([`crate::resolve_tick`]'s Phase D, engine `creeps/tick.js:52`) carries it
/// to the adjacent room. Modelling the cross as a step would be the RTS model, not the engine's.
pub fn step(pos: Position, dir: Direction) -> Option<Position> {
    let (dx, dy) = dir_delta(dir);
    let x = pos.x().u8() as i32 + dx;
    let y = pos.y().u8() as i32 + dy;
    if !(0..=49).contains(&x) || !(0..=49).contains(&y) {
        return None;
    }
    Some(Position::new(
        RoomCoordinate::new(x as u8).ok()?,
        RoomCoordinate::new(y as u8).ok()?,
        pos.room_name(),
    ))
}

/// Room-edge tile (fatigue resets to 0 on entering one, `movement.js:242`).
pub fn is_edge(x: u8, y: u8) -> bool {
    x == 0 || x == 49 || y == 0 || y == 49
}

struct Mover {
    id: CreepId,
    /// Room-qualified start + destination (S2): contention maps key by full `Position`, so two creeps
    /// at the same `(x,y)` in DIFFERENT rooms never falsely contend, chain-block, or swap.
    current_pos: Position,
    dest_pos: Position,
    move_rate: u32,
    weight: u32,   // min 1
    pulled: bool, // being dragged (engine `_pulled`): eligible regardless of own MOVE/fatigue (rate2)
    pulling: bool, // dragging another (engine `_pull`, rate3)
}

fn rate1(movers: &[Mover], want_count: &HashMap<Position, usize>, i: usize) -> u32 {
    let m = &movers[i];
    // Mutual swap: some mover is currently on m's destination and wants m's current tile.
    let swap = movers
        .iter()
        .any(|n| n.current_pos == m.dest_pos && n.dest_pos == m.current_pos);
    if swap {
        100
    } else {
        want_count.get(&m.current_pos).copied().unwrap_or(0) as u32
    }
}

fn rate4(movers: &[Mover], i: usize) -> f64 {
    movers[i].move_rate as f64 / movers[i].weight as f64
}

/// Resolve move intents with no pulls (the common case). See [`resolve_moves_with_pulls`].
pub fn resolve_moves(
    world: &MovementState,
    moves: &HashMap<CreepId, Direction>,
) -> HashMap<CreepId, Position> {
    resolve_moves_with_pulls(world, moves, &HashMap::new())
}

/// Resolve move + pull intents for a tick (engine `movement.js check`). `pulls` maps a puller to the
/// creep it drags: a pulled creep follows the puller into its vacated tile and is eligible even with
/// **no MOVE part / nonzero fatigue** (engine `canMove`'s `_pulled` branch) — this is how no-MOVE /
/// under-MOVE combat compositions stay mobile. Returns movers → new position.
pub fn resolve_moves_with_pulls(
    world: &MovementState,
    moves: &HashMap<CreepId, Direction>,
    pulls: &HashMap<CreepId, CreepId>,
) -> HashMap<CreepId, Position> {
    let creep_by_id: HashMap<CreepId, &SimCreep> = world
        .creeps
        .iter()
        .filter(|c| c.is_alive())
        .map(|c| (c.id, c))
        .collect();

    // Valid pulls: puller + target alive, adjacent, puller has a move intent. `pulled_by` maps the
    // dragged creep → its puller and overrides the dragged creep's own move intent.
    let mut pulled_by: HashMap<CreepId, CreepId> = HashMap::new();
    for (&puller, &target) in pulls {
        if let (Some(p), Some(t)) = (creep_by_id.get(&puller), creep_by_id.get(&target)) {
            if moves.contains_key(&puller) && p.pos.get_range_to(t.pos) <= 1 {
                pulled_by.insert(target, puller);
            }
        }
    }
    let pullers: HashSet<CreepId> = pulled_by.values().copied().collect();

    let mut movers: Vec<Mover> = Vec::new();
    // Self-propelled movers: alive, eligible (fatigue 0 + MOVE part), not currently being pulled.
    for c in &world.creeps {
        if !c.is_alive() || pulled_by.contains_key(&c.id) {
            continue;
        }
        let dir = match moves.get(&c.id) {
            Some(&d) => d,
            None => continue,
        };
        if c.fatigue > 0 || !c.body.can_move() {
            continue;
        }
        let dest_pos = match step(c.pos, dir) {
            Some(d) => d,
            None => continue,
        };
        movers.push(Mover {
            id: c.id,
            current_pos: c.pos,
            dest_pos,
            move_rate: c.body.move_rate(),
            weight: c.body.fatigue_weight().max(1),
            pulled: false,
            pulling: pullers.contains(&c.id),
        });
    }
    // Pulled creeps follow their puller into its current tile (only if the puller is itself moving).
    let self_mover_ids: HashSet<CreepId> = movers.iter().map(|m| m.id).collect();
    for (&target, &puller) in &pulled_by {
        if !self_mover_ids.contains(&puller) {
            continue; // puller isn't moving → nothing to follow into
        }
        let t = creep_by_id[&target];
        let p = creep_by_id[&puller];
        movers.push(Mover {
            id: target,
            current_pos: t.pos,
            dest_pos: p.pos, // into the puller's vacated tile
            move_rate: t.body.move_rate(),
            weight: t.body.fatigue_weight().max(1),
            pulled: true,
            pulling: false,
        });
    }
    if movers.is_empty() {
        return HashMap::new();
    }

    let mut want_count: HashMap<Position, usize> = HashMap::new();
    for m in &movers {
        *want_count.entry(m.dest_pos).or_insert(0) += 1;
    }
    // dest tile -> contending mover indices.
    let mut matrix: HashMap<Position, Vec<usize>> = HashMap::new();
    for (i, m) in movers.iter().enumerate() {
        matrix.entry(m.dest_pos).or_default().push(i);
    }
    // dest tile -> movers wanting it (for chain-block: a stayed creep blocks followers).
    let want_idx = matrix.clone();

    let mut moving = vec![true; movers.len()];

    // ── Contention: one winner per contested tile (rate1 then rate4); losers stay ────────────────
    for contenders in matrix.values() {
        if contenders.len() <= 1 {
            continue;
        }
        let mut best = contenders[0];
        for &i in contenders.iter().skip(1) {
            // Engine order: rate1 (swap/affected), rate2 (being pulled), rate3 (pulling), rate4.
            let key = |k: usize| {
                (
                    rate1(&movers, &want_count, k),
                    movers[k].pulled as u32,
                    movers[k].pulling as u32,
                )
            };
            let (a, b) = (key(i), key(best));
            let win = a > b || (a == b && rate4(&movers, i) > rate4(&movers, best));
            if win {
                best = i;
            }
        }
        for &i in contenders.iter() {
            if i != best {
                moving[i] = false;
            }
        }
    }

    let mover_idx_of: HashMap<CreepId, usize> =
        movers.iter().enumerate().map(|(i, m)| (m.id, i)).collect();
    // Per-tile "will an occupant REMAIN here this tick?" — an order-INDEPENDENT occupancy test. In a VALID
    // world every tile holds ≤1 creep, so this is exactly equivalent to the old per-creep `creep_at` lookup;
    // it is DEFENCE-IN-DEPTH for a degenerate input where a tile holds a stack (which should never occur —
    // placement + movement uphold one-creep-per-tile). A `HashMap<Position, CreepId>` collect would keep
    // only ONE id per stacked tile in `world.creeps` Vec-seed order, making the obstacle check below
    // iteration-order-dependent (a bit-determinism break). OR-fold instead — the tile blocks iff SOME
    // occupant stays (a non-mover, or a mover that isn't moving); `|=` is commutative ⇒ order-independent.
    // Keyed by full `Position` (room+x+y) so same-(x,y) tiles in different rooms never interact.
    let mut tile_has_stayer: HashMap<Position, bool> = HashMap::new();
    for c in world.creeps.iter().filter(|c| c.is_alive()) {
        let stays = match mover_idx_of.get(&c.id) {
            Some(&j) => !moving[j], // a mover that is moving vacates; otherwise it stays
            None => true,           // not a mover at all → stays
        };
        *tile_has_stayer.entry(c.pos).or_insert(false) |= stays;
    }

    // ── Obstacle + chain-block (removeFromMatrix) ────────────────────────────────────────────────
    let mut stack: Vec<usize> = Vec::new();
    for (i, m) in movers.iter().enumerate() {
        if !moving[i] {
            stack.push(i); // a contention loser stays → may block followers
            continue;
        }
        // Room-aware (S1): a cross-room dest checks the destination room's terrain, not the start room's.
        let wall = world
            .terrain_for(m.dest_pos.room_name())
            .is_wall(m.dest_pos.x().u8(), m.dest_pos.y().u8());
        // Destination blocked iff some occupant of dest will REMAIN there this tick (non-mover, or a mover
        // that isn't moving). OR-folded into `tile_has_stayer` above, so the decision is independent of
        // `world.creeps` iteration order even when dest holds a stack.
        let occupied = tile_has_stayer.get(&m.dest_pos).copied().unwrap_or(false);
        if wall || occupied {
            moving[i] = false;
            stack.push(i);
        }
    }
    // Propagate: a creep that stays blocks every mover that wanted its current tile.
    while let Some(i) = stack.pop() {
        if let Some(followers) = want_idx.get(&movers[i].current_pos) {
            for &j in followers {
                if moving[j] {
                    moving[j] = false;
                    stack.push(j);
                }
            }
        }
    }

    movers
        .iter()
        .enumerate()
        .filter(|(i, _)| moving[*i])
        .map(|(_, m)| (m.id, m.dest_pos))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::SimBody;
    use screeps::{Part, RoomName};

    fn pos(x: u8, y: u8) -> Position {
        let room: RoomName = "W1N1".parse().unwrap();
        Position::new(
            RoomCoordinate::new(x).unwrap(),
            RoomCoordinate::new(y).unwrap(),
            room,
        )
    }
    fn creep(id: CreepId, x: u8, y: u8, parts: &[(Part, u32)], fatigue: u32) -> SimCreep {
        let body: Vec<_> = parts
            .iter()
            .flat_map(|&(p, n)| std::iter::repeat_n(crate::body::BodyPartDef::new(p), n as usize))
            .collect();
        SimCreep {
            id,
            owner: 0,
            pos: pos(x, y),
            body: SimBody::new(body),
            fatigue,
        }
    }
    fn moves(pairs: &[(CreepId, Direction)]) -> HashMap<CreepId, Direction> {
        pairs.iter().copied().collect()
    }

    #[test]
    fn simple_move_to_empty_tile() {
        let world = MovementState {
            creeps: vec![creep(1, 25, 25, &[(Part::Move, 1)], 0)],
            ..Default::default()
        };
        let r = resolve_moves(&world, &moves(&[(1, Direction::Right)]));
        assert_eq!(r.get(&1), Some(&pos(26, 25)));
    }

    #[test]
    fn wall_blocks_the_move() {
        let mut world = MovementState {
            creeps: vec![creep(1, 25, 25, &[(Part::Move, 1)], 0)],
            ..Default::default()
        };
        world.terrain.walls.insert((26, 25));
        assert!(resolve_moves(&world, &moves(&[(1, Direction::Right)])).is_empty());
    }

    #[test]
    fn fatigued_or_moveless_cannot_move() {
        let world = MovementState {
            creeps: vec![
                creep(1, 10, 10, &[(Part::Attack, 1), (Part::Move, 1)], 5), // fatigued
                creep(2, 20, 20, &[(Part::Attack, 1)], 0),                  // no MOVE part
            ],
            ..Default::default()
        };
        let r = resolve_moves(
            &world,
            &moves(&[(1, Direction::Right), (2, Direction::Right)]),
        );
        assert!(r.is_empty());
    }

    #[test]
    fn two_creeps_contest_one_tile_only_one_moves() {
        // Both want (25,25); higher move/weight ratio wins. C1 is all-MOVE (rate4 high), C2 carries
        // dead weight (lower rate4) → C1 wins, C2 stays.
        let world = MovementState {
            creeps: vec![
                creep(1, 24, 25, &[(Part::Move, 2)], 0),
                creep(2, 26, 25, &[(Part::Attack, 4), (Part::Move, 1)], 0),
            ],
            ..Default::default()
        };
        let r = resolve_moves(
            &world,
            &moves(&[(1, Direction::Right), (2, Direction::Left)]),
        );
        assert_eq!(r.len(), 1);
        assert_eq!(r.get(&1), Some(&pos(25, 25)));
        assert!(!r.contains_key(&2));
    }

    #[test]
    fn adjacent_creeps_swap() {
        // C1 at (25,25)→(26,25); C2 at (26,25)→(25,25). Mutual swap: both move.
        let world = MovementState {
            creeps: vec![
                creep(1, 25, 25, &[(Part::Move, 1)], 0),
                creep(2, 26, 25, &[(Part::Move, 1)], 0),
            ],
            ..Default::default()
        };
        let r = resolve_moves(
            &world,
            &moves(&[(1, Direction::Right), (2, Direction::Left)]),
        );
        assert_eq!(r.get(&1), Some(&pos(26, 25)));
        assert_eq!(r.get(&2), Some(&pos(25, 25)));
    }

    #[test]
    fn blocked_front_stops_the_column() {
        // A column: C1(24,25)→(25,25); C2(23,25)→(24,25). A wall at (25,25) blocks C1; C2 wanted
        // C1's tile, so the chain-block stops C2 too.
        let mut world = MovementState {
            creeps: vec![
                creep(1, 24, 25, &[(Part::Move, 1)], 0),
                creep(2, 23, 25, &[(Part::Move, 1)], 0),
            ],
            ..Default::default()
        };
        world.terrain.walls.insert((25, 25));
        let r = resolve_moves(
            &world,
            &moves(&[(1, Direction::Right), (2, Direction::Right)]),
        );
        assert!(
            r.is_empty(),
            "blocked front must stop the follower (cohesion)"
        );
    }

    #[test]
    fn column_advances_when_front_is_clear() {
        // Same column, no wall: C1 moves into the empty tile, C2 follows into C1's vacated tile.
        let world = MovementState {
            creeps: vec![
                creep(1, 24, 25, &[(Part::Move, 1)], 0),
                creep(2, 23, 25, &[(Part::Move, 1)], 0),
            ],
            ..Default::default()
        };
        let r = resolve_moves(
            &world,
            &moves(&[(1, Direction::Right), (2, Direction::Right)]),
        );
        assert_eq!(r.get(&1), Some(&pos(25, 25)));
        assert_eq!(r.get(&2), Some(&pos(24, 25)));
    }

    fn pulls(pairs: &[(CreepId, CreepId)]) -> HashMap<CreepId, CreepId> {
        pairs.iter().copied().collect()
    }

    #[test]
    fn pull_drags_a_zero_move_creep() {
        // Puller (MOVE) at (25,25)→(26,25) drags a no-MOVE creep at (24,25) into its vacated tile.
        let world = MovementState {
            creeps: vec![
                creep(1, 25, 25, &[(Part::Move, 1)], 0),
                creep(2, 24, 25, &[(Part::Attack, 5)], 0), // no MOVE part
            ],
            ..Default::default()
        };
        let r =
            resolve_moves_with_pulls(&world, &moves(&[(1, Direction::Right)]), &pulls(&[(1, 2)]));
        assert_eq!(r.get(&1), Some(&pos(26, 25)), "puller advances");
        assert_eq!(
            r.get(&2),
            Some(&pos(25, 25)),
            "pulled creep follows into vacated tile"
        );
    }

    #[test]
    fn zero_move_creep_cannot_move_unpulled() {
        // Same no-MOVE creep, given a direct move intent but NOT pulled → it cannot move.
        let world = MovementState {
            creeps: vec![creep(2, 24, 25, &[(Part::Attack, 5)], 0)],
            ..Default::default()
        };
        let r = resolve_moves(&world, &moves(&[(2, Direction::Right)]));
        assert!(r.is_empty(), "a no-MOVE creep is immobile without a pull");
    }

    #[test]
    fn pulled_creep_moves_despite_fatigue() {
        // The dragged creep has MOVE parts but nonzero fatigue (would normally be stuck). The
        // engine `_pulled` branch bypasses both fatigue and the MOVE requirement → it still follows.
        let world = MovementState {
            creeps: vec![
                creep(1, 25, 25, &[(Part::Move, 1)], 0),
                creep(2, 24, 25, &[(Part::Move, 1), (Part::Attack, 4)], 8), // fatigued
            ],
            ..Default::default()
        };
        let r =
            resolve_moves_with_pulls(&world, &moves(&[(1, Direction::Right)]), &pulls(&[(1, 2)]));
        assert_eq!(r.get(&1), Some(&pos(26, 25)));
        assert_eq!(
            r.get(&2),
            Some(&pos(25, 25)),
            "fatigue does not stop a pulled creep"
        );
    }

    #[test]
    fn pull_does_nothing_when_puller_is_blocked() {
        // Puller blocked by a wall → it stays, so the pulled creep has nothing to follow into.
        let mut world = MovementState {
            creeps: vec![
                creep(1, 25, 25, &[(Part::Move, 1)], 0),
                creep(2, 24, 25, &[(Part::Attack, 5)], 0),
            ],
            ..Default::default()
        };
        world.terrain.walls.insert((26, 25));
        let r =
            resolve_moves_with_pulls(&world, &moves(&[(1, Direction::Right)]), &pulls(&[(1, 2)]));
        assert!(r.is_empty(), "a blocked puller drags no one");
    }

    // ── N-room (ADR 0023 S1): edge-crossing movement ──────────────────────────────────────────────
    fn room(name: &str) -> RoomName {
        name.parse().unwrap()
    }
    fn pos_in(name: &str, x: u8, y: u8) -> Position {
        Position::new(
            RoomCoordinate::new(x).unwrap(),
            RoomCoordinate::new(y).unwrap(),
            room(name),
        )
    }

    #[test]
    fn step_does_not_cross_room_boundaries() {
        // A within-room step works; a step off the room edge returns None (crossing is NOT a move —
        // engine `move.js:32` rejects it; the edge-exit relocation in resolve_tick does the cross).
        assert_eq!(
            step(pos_in("W1N1", 48, 25), Direction::Right),
            Some(pos_in("W1N1", 49, 25)),
            "an in-room step advances"
        );
        assert_eq!(
            step(pos_in("W1N1", 49, 25), Direction::Right),
            None,
            "a step off the east edge is rejected (no within-room cross)"
        );
        assert_eq!(step(pos_in("W1N1", 0, 25), Direction::Left), None, "off the west edge: rejected");
        assert_eq!(step(pos_in("W1N1", 25, 0), Direction::Top), None, "off the north edge: rejected");
        assert_eq!(step(pos_in("W1N1", 25, 49), Direction::Bottom), None, "off the south edge: rejected");
    }

    #[test]
    fn resolve_moves_does_not_carry_a_creep_off_an_edge() {
        // A creep on the east edge moving Right does NOT move within the room (the cross is the
        // edge-exit relocation in resolve_tick, not a movement-phase step).
        let world = MovementState {
            creeps: vec![creep(1, 49, 25, &[(Part::Move, 1)], 0)],
            ..Default::default()
        };
        let r = resolve_moves(&world, &moves(&[(1, Direction::Right)]));
        assert!(r.is_empty(), "an off-edge move yields no within-room movement");
    }

    #[test]
    fn wall_check_reads_the_creeps_own_room_terrain() {
        // Movement in a NON-default room reads that room's terrain override (S1 room-aware terrain): a
        // creep in W3N1 moving within W3N1 is blocked by a wall placed in W3N1's override, and the
        // default terrain (a different room) is not consulted.
        let mk = |wall: bool| {
            let mut w = MovementState {
                creeps: vec![SimCreep {
                    id: 1,
                    owner: 0,
                    pos: pos_in("W3N1", 24, 25),
                    body: SimBody::new(vec![crate::body::BodyPartDef::new(Part::Move)]),
                    fatigue: 0,
                }],
                ..Default::default()
            };
            w.terrain_mut(room("W1N1")).walls.insert((25, 25)); // a DIFFERENT room's wall — must be ignored
            if wall {
                w.terrain_mut(room("W3N1")).walls.insert((25, 25));
            }
            w
        };
        assert_eq!(
            resolve_moves(&mk(false), &moves(&[(1, Direction::Right)])).get(&1),
            Some(&pos_in("W3N1", 25, 25)),
            "a wall in a different room (W1N1) is ignored → the creep advances in W3N1"
        );
        assert!(
            resolve_moves(&mk(true), &moves(&[(1, Direction::Right)])).is_empty(),
            "a wall in the creep's own room (W3N1) blocks it"
        );
    }

    #[test]
    fn creeps_in_different_rooms_do_not_contend_for_the_same_xy() {
        // Two creeps, one in each of two rooms, each stepping Right toward (25,25) in ITS OWN room.
        // (x,y)-keyed contention (the pre-S2 bug) would make them fight over a single tile so only one
        // moves; Position-keyed contention lets both advance.
        let mk = |id: CreepId, name: &str| SimCreep {
            id,
            owner: 0,
            pos: pos_in(name, 24, 25),
            body: SimBody::new(vec![crate::body::BodyPartDef::new(Part::Move)]),
            fatigue: 0,
        };
        let world = MovementState {
            creeps: vec![mk(1, "W1N1"), mk(2, "W3N1")],
            ..Default::default()
        };
        let r = resolve_moves(&world, &moves(&[(1, Direction::Right), (2, Direction::Right)]));
        assert_eq!(r.get(&1), Some(&pos_in("W1N1", 25, 25)), "creep in room 1 advances");
        assert_eq!(
            r.get(&2),
            Some(&pos_in("W3N1", 25, 25)),
            "creep in room 2 advances too — no cross-room (x,y) contention"
        );
    }

    #[test]
    fn stacked_tile_obstacle_is_order_independent() {
        // A transported same-tile stack is FAITHFUL: real Screeps cross-room entry is occupancy-blind
        // (engine creeps/tick.js:52 + global.js:34/42 — the relocation guards room-accessibility only,
        // never tile occupancy), so two creeps legitimately share a Position until they walk apart. The
        // obstacle decision over such a stack must NOT depend on `world.creeps` Vec order. Here A@(0,25) is
        // a mover vacating (Top), B@(0,25) is a NON-MOVER staying, C@(1,25) moves Left toward (0,25). B
        // stays on (0,25), so C must ALWAYS be blocked — independent of which of A/B a `HashMap<Position,_>`
        // happened to keep.
        let mk = |order: &[u8; 3]| -> MovementState {
            let by_id = |id: u8| match id {
                1 => creep(1, 0, 25, &[(Part::Move, 1)], 0),   // A: mover, will move Top (vacates 0,25)
                2 => creep(2, 0, 25, &[(Part::Attack, 1)], 0), // B: no MOVE part → non-mover, stays on 0,25
                _ => creep(3, 1, 25, &[(Part::Move, 1)], 0),   // C: mover, will move Left toward (0,25)
            };
            MovementState { creeps: order.iter().map(|&id| by_id(id)).collect(), ..Default::default() }
        };
        let mvs = moves(&[(1, Direction::Top), (3, Direction::Left)]);
        let perms: [[u8; 3]; 6] = [[1, 2, 3], [1, 3, 2], [2, 1, 3], [2, 3, 1], [3, 1, 2], [3, 2, 1]];
        let results: Vec<Option<Position>> = perms.iter().map(|p| resolve_moves(&mk(p), &mvs).get(&3).copied()).collect();
        assert!(results.iter().all(|r| *r == results[0]), "C's resolved move must be order-independent over a stacked tile, got {results:?}");
        assert_eq!(results[0], None, "C must be blocked by the staying B on (0,25) — engine-faithful (B does not vacate)");
    }
}
