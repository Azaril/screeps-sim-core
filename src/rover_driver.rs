//! The offline **rover driver** (ADR 0033 M1) — the headless analogue of the live
//! `MovementSystemExternalProvider`. It runs rover's real `MovementSystem` (resolver + `LocalPathfinder`)
//! over a [`MovementState`] and returns one `Direction` per creep, to hand to
//! [`resolve_movement`](crate::resolve_movement) (the "server"). Gated behind the `rover` feature so the
//! base kernel stays rover-free.
//!
//! **Layering:** this is the movement *mechanism* only. It is generic over the routing cost — the caller
//! injects a [`CostMatrixDataSource`] ("pricing policy", per ADR 0033 / "no one-off pathfinding"): a
//! combat layer supplies tower/threat/structure obstacles (`screeps-combat-agent`), a movement/economy
//! benchmark supplies plain terrain (`screeps-rover-eval`). The driver reads only `MovementState`
//! (creeps + terrain), never any combat state — that is why it lives in the kernel.

use crate::world::{CreepId, MovementState};
use screeps::{Direction, Position};
use screeps_rover::traits::CreepHandle;
use screeps_rover::{
    AnchorConstraint, CostMatrixCache, CostMatrixDataSource, CostMatrixSystem, CreepMovementData,
    FleeTarget, LocalPathfinder, MovementData, MovementError, MovementPriority, MovementSystem,
    MovementSystemExternal, StuckThresholds,
};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Default shove-chain depth for the sim mover (matches the live tuning).
pub const DEFAULT_SHOVE_DEPTH: u32 = 3;

/// The rover tunables one driver run is configured with (ADR 0033 §D5.4 tuning). Everything the
/// `MovementSystem` exposes as a deterministic knob, in one injectable value — the unit a parameter
/// tournament sweeps. `Default` mirrors the live defaults exactly, so the plain
/// [`resolve_moves_via_system`] is byte-identical to the pre-config behavior.
#[derive(Clone, Debug)]
pub struct MoverConfig {
    /// Resolver shove-chain depth (live default 3).
    pub max_shove_depth: u32,
    /// Ticks a cached path is followed before an expiry repath — path COMMITMENT (live default 20,
    /// tournament-tuned 5→20; see rover's `DEFAULT_REUSE_PATH_LENGTH` rationale).
    pub reuse_path_length: u32,
    /// Per-tick pathfinding ops budget (1 op ≈ 0.001 CPU live; default 20_000).
    pub pathfinding_ops_budget: u32,
    /// Chebyshev radius for tier-1 friendly-avoid escalation (0 = all friendlies).
    pub friendly_creep_distance: u32,
    /// The stuck-escalation ladder — how fast blocked creeps escalate through
    /// friendly-avoid → all-friendly → more-ops → shove → report-failure.
    pub stuck_thresholds: StuckThresholds,
    /// Register unrequested same-side creeps as resolver-known stationary occupants
    /// (`set_idle_creep_positions`). OPT-IN, default `false`: proactive avoidance without the
    /// denial-as-stuck signal starves sealed corridors (see the registration block in
    /// [`resolve_moves_via_system_with`]); flips to default-on when that follow-up slice lands.
    pub register_idle_creeps: bool,
}

impl Default for MoverConfig {
    fn default() -> Self {
        MoverConfig {
            max_shove_depth: DEFAULT_SHOVE_DEPTH,
            reuse_path_length: 20,
            pathfinding_ops_budget: 20_000,
            friendly_creep_distance: screeps_rover::DEFAULT_FRIENDLY_CREEP_DISTANCE,
            stuck_thresholds: StuckThresholds::default(),
            register_idle_creeps: false,
        }
    }
}

/// Per-creep movement state (cached path + stuck tracking), persisted across ticks by the caller.
pub type SimMoveCache = HashMap<CreepId, CreepMovementData>;

/// A movement goal for [`resolve_moves_via_system`].
pub enum SimMoveGoal {
    /// Reach `target` within `range`.
    To { target: Position, range: u32 },
    /// Flee to outside `range` of every threat.
    Flee { threats: Vec<Position>, range: u32 },
}

/// A per-creep movement request for [`resolve_moves_via_system`]. `priority` decides who wins a
/// contested tile (the resolver orders by priority before any tie-break) — e.g. a squad's combat
/// creep takes `High` so it claims the forward kite/shooting spot over a support creep.
pub struct SimMoveRequest {
    pub creep: CreepId,
    pub goal: SimMoveGoal,
    pub priority: MovementPriority,
    /// Allow the resolver to SHOVE/swap others to reach the tile (the rover default). Toggle off to A/B
    /// shoving's effect on positioning (the investigated control).
    pub shove: bool,
    /// Optional anchor `(center, range)`: confine the resolver's shoves/swaps for this creep to within
    /// `range` of `center` so a cohesive squad can't be scattered off its scored tiles (the rover
    /// `AnchorConstraint`). `None` = unconstrained.
    pub anchor: Option<(Position, u32)>,
}

impl SimMoveRequest {
    /// A `move_to` request (default priority, shove on): reach `target` within `range`.
    pub fn move_to(creep: CreepId, target: Position, range: u32) -> Self {
        SimMoveRequest {
            creep,
            goal: SimMoveGoal::To { target, range },
            priority: MovementPriority::Normal,
            shove: true,
            anchor: None,
        }
    }

    /// Set the contention priority (e.g. `High` for a combat creep that must win the shooting tile).
    pub fn with_priority(mut self, priority: MovementPriority) -> Self {
        self.priority = priority;
        self
    }

    /// Enable/disable shoving for this request (the investigated control).
    pub fn with_shove(mut self, shove: bool) -> Self {
        self.shove = shove;
        self
    }

    /// Confine this creep's shoves/swaps to within `range` of `center` (anti-scatter anchor).
    pub fn with_anchor(mut self, center: Position, range: u32) -> Self {
        self.anchor = Some((center, range));
        self
    }
}

/// Shared sink the creep handles write their resolved direction into (`move_direction` is `&self`,
/// mirroring the live `creep.move()`, so it needs interior mutability).
type MoveSink = Rc<RefCell<HashMap<CreepId, Direction>>>;

/// A [`CreepHandle`] over a `SimCreep` snapshot; `move_direction` records into the shared sink (the
/// sim's analogue of issuing `creep.move(dir)` to the server).
struct SimCreepHandle {
    id: CreepId,
    pos: Position,
    fatigue: u32,
    sink: MoveSink,
}

impl CreepHandle for SimCreepHandle {
    fn pos(&self) -> Position {
        self.pos
    }
    fn fatigue(&self) -> u32 {
        self.fatigue
    }
    fn spawning(&self) -> bool {
        false
    }
    fn move_direction(&self, dir: Direction) -> Result<(), String> {
        self.sink.borrow_mut().insert(self.id, dir);
        Ok(())
    }
    fn pull(&self, _other: &Self) -> Result<(), String> {
        Ok(()) // pull chains: a sim follow-up (the engine supports Intents.pulls); no-op for now.
    }
    fn move_pulled_by(&self, _other: &Self) -> Result<(), String> {
        Ok(())
    }
}

/// [`MovementState`]-backed [`MovementSystemExternal`] — the headless analogue of the live
/// `MovementSystemExternalProvider`. Owns the move sink, borrows the world + the caller's cache. Reads
/// only `movement.creeps` (positions + fatigue) — no combat state, which is why it is kernel code.
struct SimMovementExternal<'w, 'c> {
    movement: &'w MovementState,
    sink: MoveSink,
    cache: &'c mut SimMoveCache,
}

impl MovementSystemExternal<CreepId> for SimMovementExternal<'_, '_> {
    type Creep = SimCreepHandle;

    fn get_creep(&self, entity: CreepId) -> Result<SimCreepHandle, MovementError> {
        let c = self
            .movement
            .creeps
            .iter()
            .find(|c| c.id == entity && c.is_alive())
            .ok_or_else(|| "creep not found".to_owned())?;
        Ok(SimCreepHandle {
            id: entity,
            pos: c.pos,
            fatigue: c.fatigue,
            sink: self.sink.clone(),
        })
    }

    fn get_creep_movement_data(
        &mut self,
        entity: CreepId,
    ) -> Result<&mut CreepMovementData, MovementError> {
        Ok(self.cache.entry(entity).or_default())
    }

    fn get_entity_position(&self, entity: CreepId) -> Option<Position> {
        self.movement
            .creeps
            .iter()
            .find(|c| c.id == entity && c.is_alive())
            .map(|c| c.pos)
    }
}

/// Run rover's `MovementSystem` (resolver included) over `movement` for `requests`, returning the
/// resolved per-creep directions to hand to [`resolve_movement`](crate::resolve_movement). `cache` is
/// the caller's persisted per-creep movement state (path reuse + stuck-escalation accumulate across
/// ticks). `cost_source` is the caller's routing policy — the ONLY layering seam: combat injects
/// tower/threat/structure obstacles, a movement benchmark injects plain terrain. This is the
/// traffic-managed, unified analogue of routing each creep individually.
pub fn resolve_moves_via_system<S: CostMatrixDataSource + 'static>(
    movement: &MovementState,
    requests: &[SimMoveRequest],
    cache: &mut SimMoveCache,
    cost_source: S,
) -> HashMap<CreepId, Direction> {
    resolve_moves_via_system_with(movement, requests, cache, cost_source, &MoverConfig::default())
}

/// [`resolve_moves_via_system`] with explicit rover tunables — the entry point a parameter
/// tournament sweeps (ADR 0033 §D5.4): same mover, one [`MoverConfig`] per evaluated point.
pub fn resolve_moves_via_system_with<S: CostMatrixDataSource + 'static>(
    movement: &MovementState,
    requests: &[SimMoveRequest],
    cache: &mut SimMoveCache,
    cost_source: S,
    config: &MoverConfig,
) -> HashMap<CreepId, Direction> {
    let sink: MoveSink = Rc::new(RefCell::new(HashMap::new()));
    let mut external = SimMovementExternal {
        movement,
        sink: sink.clone(),
        cache,
    };

    let mut cm_cache = CostMatrixCache::default();
    let mut cms = CostMatrixSystem::new(&mut cm_cache, Box::new(cost_source));
    let mut pf = LocalPathfinder;
    let mut system = MovementSystem::new(&mut cms, &mut pf, None);
    system.set_max_shove_depth(config.max_shove_depth);
    system.set_reuse_path_length(config.reuse_path_length);
    system.set_pathfinding_ops_budget(config.pathfinding_ops_budget);
    system.set_friendly_creep_distance(config.friendly_creep_distance);
    system.set_stuck_thresholds(config.stuck_thresholds.clone());
    // Offline there is no CPU meter, so the budgets are explicitly unlimited. HISTORICAL RECORD:
    // when these lines landed, rover treated an ABSENT budget as EXHAUSTED (`is_none_or`,
    // pre-2026-07-01 `is_cpu_budget_exhausted`), which silently disabled ALL stuck-escalation and
    // expiry repathing here — a stuck creep re-issued its blocked move forever (the permanent
    // livelock the rover-eval failed-move sentinel caught, ADR 0033 §M4 F1). Rover now treats
    // None as UNLIMITED (fixed at the source, aligned with `is_over_tick_limit`), so these two
    // lines are belt-and-braces: kept so the offline contract is stated, not inherited. Work
    // stays bounded deterministically by the pathfinding ops budget.
    system.set_cpu_budget(|| 0.0, f64::MAX);
    system.set_repath_budget(|| 0.0, f64::MAX);

    // PARKED-CREEP REGISTRATION (ADR 0033 §M4 F2 — the `failed_into_parked` eliminator): every
    // living creep with NO request this tick is a known stationary occupant for exactly one tick
    // — the driver sees both the world and the request set, so it registers them and the resolver
    // routes around parked creeps DELIBERATELY instead of pathing into them optimistically and
    // burning `ticks_immobile ≥ 2` engine-rejected intents per blocking event. Scoped to the
    // REQUESTERS' owners: an unrequested creep of another owner is not "idle", it is an opponent
    // moved by its own driver call — pricing hostiles stays the injected cost source's job (the
    // combat layer's threat matrices), exactly as live. Built Handle-sorted, lowest id kept on a
    // (degenerate) stacked tile — the resolver's `current_pos_to_entity` defence-in-depth pattern,
    // so the map is a pure function of the world, never of HashMap iteration order.
    // ⚠ OPT-IN (`config.register_idle_creeps`, default OFF): registration's proactive resolver
    // avoidance DESTROYS THE STUCK SIGNAL — a denied mover sidesteps every tick (constant motion,
    // `ticks_immobile` never accrues), so the friendly-avoid/ops escalation tiers never fire and a
    // parked creep sealing a 1-wide corridor starves its mate in a zero-failed-intent DANCE
    // livelock (the rover-eval corpus ratchets caught it: pinch 7/8 trips, E11N1 11/12). The
    // completing design — idle-DENIALS must count as immobility so the dance feeds the escalation
    // ladder — is a recorded follow-up slice with its own tournament validation; until it lands,
    // the default stays the proven push-fail-escalate behavior (bounded failed intents per event).
    // LIVE-FIDELITY NOTE: the live bot does NOT register idle creeps today (ibex
    // pathing/movementsystem.rs passes none), so offline-with-hook is BETTER-than-live
    // coordination; mirroring this registration in the bot crate is part of the same follow-up.
    if config.register_idle_creeps {
        let requested: std::collections::HashSet<CreepId> =
            requests.iter().map(|r| r.creep).collect();
        let requester_owners: std::collections::HashSet<_> = movement
            .creeps
            .iter()
            .filter(|c| requested.contains(&c.id))
            .map(|c| c.owner)
            .collect();
        let mut parked: Vec<(CreepId, Position)> = movement
            .creeps
            .iter()
            .filter(|c| {
                c.is_alive() && !requested.contains(&c.id) && requester_owners.contains(&c.owner)
            })
            .map(|c| (c.id, c.pos))
            .collect();
        parked.sort_unstable_by_key(|(id, _)| *id);
        let mut idle: HashMap<Position, CreepId> = HashMap::new();
        for (id, pos) in parked {
            idle.entry(pos).or_insert(id);
        }
        system.set_idle_creep_positions(idle);
    }

    // The MovementSystem routes to the (possibly cross-room) target directly — the rover search is
    // multi-room, so no MoveToRoom pre-projection is needed.
    let mut data = MovementData::new();
    for req in requests {
        match &req.goal {
            SimMoveGoal::To { target, range } => {
                let mut mr = data.move_to(req.creep, *target);
                mr.range(*range)
                    .allow_shove(req.shove)
                    .allow_swap(req.shove)
                    .priority(req.priority);
                if let Some((position, range)) = req.anchor {
                    mr.anchor(AnchorConstraint { position, range });
                }
            }
            SimMoveGoal::Flee { threats, range } => {
                let targets: Vec<FleeTarget> = threats
                    .iter()
                    .map(|p| FleeTarget {
                        pos: *p,
                        range: *range,
                    })
                    .collect();
                let mut mr = data.flee(req.creep, targets);
                mr.allow_shove(req.shove)
                    .allow_swap(req.shove)
                    .priority(req.priority);
                if let Some((position, range)) = req.anchor {
                    mr.anchor(AnchorConstraint { position, range });
                }
            }
        }
    }
    let _ = system.process(&mut external, data);

    drop(external);
    Rc::try_unwrap(sink)
        .map(|c| c.into_inner())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::SimBody;
    use crate::intents::MoveIntents;
    use crate::tick::resolve_movement;
    use crate::world::SimCreep;
    use screeps::{LocalCostMatrix, Part, RoomCoordinate, RoomName};
    use screeps_rover::{
        ConstructionSiteCostMatrixCache, CreepCostMatrixCache, LinearCostMatrix,
        StuctureCostMatrixCache,
    };

    fn pos(x: u8, y: u8) -> Position {
        let room: RoomName = "W1N1".parse().unwrap();
        Position::new(RoomCoordinate::new(x).unwrap(), RoomCoordinate::new(y).unwrap(), room)
    }

    /// The minimal pricing policy: open plains everywhere (empty matrices). Proves the kernel driver
    /// works with a NON-combat cost source — the reuse the benchmark (M4) depends on.
    struct PlainCostSource;
    impl CostMatrixDataSource for PlainCostSource {
        fn get_structure_costs(&self, _r: RoomName) -> Option<StuctureCostMatrixCache> {
            Some(StuctureCostMatrixCache {
                roads: LinearCostMatrix::new(),
                other: LinearCostMatrix::new(),
            })
        }
        fn get_construction_site_costs(&self, _r: RoomName) -> Option<ConstructionSiteCostMatrixCache> {
            None
        }
        fn get_creep_costs(&self, _r: RoomName) -> Option<CreepCostMatrixCache> {
            Some(CreepCostMatrixCache {
                friendly_creeps: LinearCostMatrix::new(),
                hostile_creeps: LinearCostMatrix::new(),
                source_keeper_agro: LinearCostMatrix::new(),
            })
        }
    }
    // Touch LocalCostMatrix so the import is used across screeps versions where the alias differs.
    const _: fn() -> LocalCostMatrix = LocalCostMatrix::new;

    #[test]
    fn drives_a_lone_creep_to_its_goal_over_open_plains() {
        let mut world = MovementState {
            creeps: vec![SimCreep {
                id: 1,
                owner: 0,
                pos: pos(10, 25),
                body: SimBody::unboosted(&[Part::Move]),
                fatigue: 0,
                carry_used: 0,
            }],
            ..Default::default()
        };
        let mut cache = SimMoveCache::new();
        let mut reached = false;
        for _ in 0..40 {
            let reqs = [SimMoveRequest::move_to(1, pos(20, 25), 0)];
            let dirs = resolve_moves_via_system(&world, &reqs, &mut cache, PlainCostSource);
            let mut intents = MoveIntents::new();
            for (&id, &d) in &dirs {
                intents.set_move(id, d);
            }
            resolve_movement(&mut world, &intents);
            if world.creeps[0].pos == pos(20, 25) {
                reached = true;
                break;
            }
        }
        assert!(reached, "driver should route the creep east to its goal, ended at {:?}", world.creeps[0].pos);
    }

    /// PARKED-CREEP REGISTRATION end-to-end (ADR 0033 §M4 F2): creep 2 sits mid-route with NO
    /// request — the goal-reached "parked" shape that used to be invisible to rover (its
    /// optimistic first-path runs straight through the tile; the engine then rejects the issued
    /// move for `ticks_immobile ≥ 2` per blocking event — `failed_into_parked`). With the driver
    /// auto-registering unrequested same-side creeps via `set_idle_creep_positions`, the resolver
    /// sidesteps around it deliberately. Gate: applying the issued dirs via `resolve_movement`
    /// executes EVERY intent (moved == issued — zero failed moves), the mover arrives, and the
    /// parked creep is never displaced.
    #[test]
    fn routes_around_a_parked_unrequested_creep_without_failed_moves() {
        let mut world = MovementState {
            creeps: vec![
                SimCreep {
                    id: 1,
                    owner: 0,
                    pos: pos(10, 25),
                    body: SimBody::unboosted(&[Part::Move]),
                    fatigue: 0,
                    carry_used: 0,
                },
                SimCreep {
                    id: 2,
                    owner: 0,
                    pos: pos(15, 25), // parked ON the straight-line route, never requested
                    body: SimBody::unboosted(&[Part::Move]),
                    fatigue: 0,
                    carry_used: 0,
                },
            ],
            ..Default::default()
        };
        let goal = pos(20, 25);
        let mut cache = SimMoveCache::new();
        let mut reached = false;
        let (mut issued, mut executed) = (0usize, 0usize);
        for _ in 0..40 {
            if world.creeps[0].pos == goal {
                reached = true;
                break;
            }
            let reqs = [SimMoveRequest::move_to(1, goal, 0)];
            // OPT IN to parked-creep registration (default off pending the denial-as-stuck
            // follow-up) — this test validates the opt-in mechanics end-to-end.
            let cfg = MoverConfig { register_idle_creeps: true, ..Default::default() };
            let dirs = resolve_moves_via_system_with(&world, &reqs, &mut cache, PlainCostSource, &cfg);
            let mut intents = MoveIntents::new();
            for (&id, &d) in &dirs {
                intents.set_move(id, d);
            }
            let report = resolve_movement(&mut world, &intents);
            issued += dirs.len();
            executed += report.moved.len();
        }
        assert!(reached, "mover must arrive despite the parked blocker, ended at {:?}", world.creeps[0].pos);
        assert_eq!(
            executed, issued,
            "every issued intent must execute — a shortfall is a failed move into the parked tile"
        );
        assert!(issued > 0, "the mover must actually have been driven");
        assert_eq!(world.creeps[1].pos, pos(15, 25), "the parked creep is routed around, not displaced");
    }
}
