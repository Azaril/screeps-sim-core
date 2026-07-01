//! The creep body model — per-part 100-hit pools, back-to-front degradation, and the boost-aware
//! effectiveness helper (`calcBodyEffectiveness`) the movement kernel needs (MOVE/fatigue counts).
//! Faithful to the engine; see the cited source per function. Body-COMBAT arithmetic (attack/heal/
//! dismantle power, TOUGH/boost damage reduction) is NOT here — the mover never needs it; it lives
//! in the combat layer as the `SimBodyCombat` extension trait (`screeps-combat-engine`, ADR 0033).

use crate::constants::*;
use screeps::Part;

/// Boost tier for a body part. The engine keys boosts by mineral (`BOOSTS[type][mineral]`); the
/// three tiers per part type map exactly onto these multipliers, so a tier abstraction is faithful
/// and avoids threading mineral `ResourceType`s through the sim. The live `CombatView` adapter
/// (H2) maps `ResourceType -> BoostTier` when ingesting a real creep.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BoostTier {
    #[default]
    None,
    T1,
    T2,
    T3,
}

impl BoostTier {
    /// Multiplier for an offensive/heal/dismantle action (attack/rangedAttack/heal/dismantle):
    /// ×1/2/3/4 (`BOOSTS[ATTACK][UH|UH2O|XUH2O].attack` etc.).
    pub fn action_mult(self) -> f64 {
        match self {
            BoostTier::None => 1.0,
            BoostTier::T1 => 2.0,
            BoostTier::T2 => 3.0,
            BoostTier::T3 => 4.0,
        }
    }

    /// Incoming-damage multiplier for a TOUGH part (`BOOSTS[TOUGH][GO|GHO2|XGHO2].damage`):
    /// 1.0 / 0.7 / 0.5 / 0.3. Lower = more mitigation.
    pub fn tough_damage_ratio(self) -> f64 {
        match self {
            BoostTier::None => 1.0,
            BoostTier::T1 => 0.7,
            BoostTier::T2 => 0.5,
            BoostTier::T3 => 0.3,
        }
    }

    /// Fatigue-clear multiplier for a MOVE part (`BOOSTS[MOVE][ZO|ZHO2|XZHO2].fatigue`): ×1/2/3/4.
    pub fn move_mult(self) -> f64 {
        self.action_mult()
    }

    /// Carry-capacity multiplier for a CARRY part (`BOOSTS[CARRY][KH|KH2O|XKH2O].capacity`): ×1/2/3/4
    /// — the same tier ladder as [`action_mult`](Self::action_mult), returned as an integer since a
    /// boosted CARRY holds `CARRY_CAPACITY × mult` resources exactly.
    pub fn carry_capacity_mult(self) -> u32 {
        self.action_mult() as u32
    }
}

/// One body part: its type and boost tier. (Per-part current hits are *derived* from the body
/// total via [`SimBody::part_hits`], exactly as the engine recomputes them in `_recalc-body`.)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyPartDef {
    pub part: Part,
    pub boost: BoostTier,
}

impl BodyPartDef {
    pub fn new(part: Part) -> Self {
        Self {
            part,
            boost: BoostTier::None,
        }
    }
    pub fn boosted(part: Part, boost: BoostTier) -> Self {
        Self { part, boost }
    }
}

/// A creep body: an ordered part list (front = index 0) plus the single `hits` total. Part hits
/// are derived from `hits`, not stored, matching the engine's `object.hits` + `_recalc-body`.
#[derive(Clone, Debug)]
pub struct SimBody {
    /// Ordered front (index 0) → back. Front parts degrade first (engine `_recalc-body`), so put
    /// TOUGH/expendable parts front and MOVE/HEAL back.
    pub parts: Vec<BodyPartDef>,
    /// Current total hit points (0..=`hits_max`).
    pub hits: u32,
}

impl SimBody {
    /// A full-health body.
    pub fn new(parts: Vec<BodyPartDef>) -> Self {
        let hits = parts.len() as u32 * BODYPART_HITS;
        Self { parts, hits }
    }

    /// Convenience: a full-health unboosted body from a part slice.
    pub fn unboosted(parts: &[Part]) -> Self {
        Self::new(parts.iter().map(|&p| BodyPartDef::new(p)).collect())
    }

    pub fn hits_max(&self) -> u32 {
        self.parts.len() as u32 * BODYPART_HITS
    }

    pub fn is_alive(&self) -> bool {
        self.hits > 0
    }

    /// Current hits of part `i`, derived back-to-front (engine `_recalc-body.js`: the loop fills
    /// from the last part toward index 0, 100 each). So trailing parts stay full and `body[0]`
    /// (front) is the first to drop to 0 — the basis for "TOUGH front, MOVE back".
    pub fn part_hits(&self, i: usize) -> u32 {
        let len = self.parts.len();
        if i >= len {
            return 0;
        }
        let behind = (len - 1 - i) as i64; // parts after `i` fill before it
        (self.hits as i64 - BODYPART_HITS as i64 * behind).clamp(0, BODYPART_HITS as i64) as u32
    }

    /// `calcBodyEffectiveness(body, part_type, _, base)` (`utils.js:623`): sum over **alive**
    /// (`hits > 0`) parts of `part_type` of `base × boost_mult`.
    pub fn effective_power(&self, part_type: Part, base: u32) -> u32 {
        let mut power = 0.0;
        for (i, p) in self.parts.iter().enumerate() {
            if p.part == part_type && self.part_hits(i) > 0 {
                power += base as f64 * p.boost.action_mult();
            }
        }
        power as u32
    }

    /// Count of ALIVE (`hits > 0`) parts of `part_type` — the raw, un-boost-weighted count (for replay
    /// composition display: "this creep is 8×TOUGH + 25×WORK"). Degrades as front parts are destroyed.
    pub fn alive_part_count(&self, part_type: Part) -> u32 {
        self.parts.iter().enumerate().filter(|(i, p)| p.part == part_type && self.part_hits(*i) > 0).count() as u32
    }

    /// `calcBodyEffectiveness(body, MOVE, 'fatigue', 1)` — the boost-weighted count of alive MOVE
    /// parts (the movement tiebreak's numerator, `movement.js:118`).
    pub fn move_rate(&self) -> u32 {
        self.effective_power(Part::Move, 1)
    }

    /// Count of alive non-MOVE/non-CARRY parts — the *structural* fatigue weight (`movement.js:120,237`).
    /// The full move-fatigue weight also adds the loaded-CARRY units ([`carry_weight`](Self::carry_weight));
    /// use [`SimCreep::fatigue_weight`](crate::SimCreep::fatigue_weight), which sums both.
    pub fn fatigue_weight(&self) -> u32 {
        let mut n = 0;
        for (i, p) in self.parts.iter().enumerate() {
            if p.part != Part::Move && p.part != Part::Carry && self.part_hits(i) > 0 {
                n += 1;
            }
        }
        n
    }

    /// Loaded-CARRY fatigue units for `carry_used` resources aboard (`calcResourcesWeight`,
    /// `movement.js:41`): walking the body back-to-front, each ALIVE CARRY part absorbs
    /// `CARRY_CAPACITY × capacity_boost` of the load and adds 1 to the weight, until the load is
    /// exhausted. An empty creep (`carry_used == 0`) adds nothing; empty CARRY parts are weightless.
    pub fn carry_weight(&self, carry_used: u32) -> u32 {
        let mut remaining = carry_used;
        let mut weight = 0;
        for i in (0..self.parts.len()).rev() {
            if remaining == 0 {
                break;
            }
            let p = self.parts[i];
            if p.part != Part::Carry || self.part_hits(i) == 0 {
                continue;
            }
            remaining = remaining.saturating_sub(CARRY_CAPACITY * p.boost.carry_capacity_mult());
            weight += 1;
        }
        weight
    }

    /// True if the creep has at least one working MOVE part (engine `canMove`).
    pub fn can_move(&self) -> bool {
        self.parts
            .iter()
            .enumerate()
            .any(|(i, p)| p.part == Part::Move && self.part_hits(i) > 0)
    }

    /// Fatigue cleared per tick: `2 × Σ alive MOVE parts` (boost-weighted), per `creeps/tick.js:107`.
    pub fn fatigue_clear(&self) -> u32 {
        let mut mult = 0.0;
        for (i, p) in self.parts.iter().enumerate() {
            if p.part == Part::Move && self.part_hits(i) > 0 {
                mult += p.boost.move_mult();
            }
        }
        (FATIGUE_CLEAR_PER_MOVE as f64 * mult) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(parts: &[(Part, BoostTier)]) -> SimBody {
        SimBody::new(
            parts
                .iter()
                .map(|&(p, b)| BodyPartDef::boosted(p, b))
                .collect(),
        )
    }

    #[test]
    fn part_hits_fill_back_to_front() {
        // [Tough, Attack, Move] — full = 300, all parts 100.
        let mut b = SimBody::unboosted(&[Part::Tough, Part::Attack, Part::Move]);
        assert_eq!(
            (b.part_hits(0), b.part_hits(1), b.part_hits(2)),
            (100, 100, 100)
        );
        // At 150 hits the trailing parts fill first: Move=100, Attack=50, Tough=0.
        b.hits = 150;
        assert_eq!(
            (b.part_hits(0), b.part_hits(1), b.part_hits(2)),
            (0, 50, 100)
        );
    }

    #[test]
    fn power_degrades_as_front_parts_die() {
        let mut b = SimBody::unboosted(&[Part::Tough, Part::Attack, Part::Move]);
        b.hits = 100; // only the Move part (index 2) is alive; the front parts are dead
        assert_eq!(b.fatigue_clear(), 2); // the surviving MOVE still clears fatigue
    }

    #[test]
    fn fatigue_clear_counts_move_parts() {
        assert_eq!(body(&[(Part::Move, BoostTier::None); 3]).fatigue_clear(), 6); // 2 × 3
        assert_eq!(body(&[(Part::Move, BoostTier::T1); 3]).fatigue_clear(), 12);
        // 2 × 3 × 2
    }

    #[test]
    fn carry_weight_counts_loaded_parts_only() {
        // 3×CARRY + 1×MOVE: capacity 150. Weight = ceil(load / 50), capped at 3 alive CARRY parts.
        let b = SimBody::unboosted(&[Part::Carry, Part::Carry, Part::Carry, Part::Move]);
        assert_eq!(b.carry_weight(0), 0, "an empty creep adds no carry weight");
        assert_eq!(b.carry_weight(1), 1, "any load needs its first CARRY part");
        assert_eq!(b.carry_weight(50), 1, "one full part");
        assert_eq!(b.carry_weight(51), 2, "spills into a second part");
        assert_eq!(b.carry_weight(150), 3, "all three parts loaded");
        assert_eq!(b.carry_weight(999), 3, "capped at the alive CARRY count");
    }

    #[test]
    fn carry_weight_is_boost_aware() {
        // A T2 CARRY part (×3 capacity) holds 150, so 150 of load is one part's worth.
        let b = body(&[(Part::Carry, BoostTier::T2), (Part::Move, BoostTier::None)]);
        assert_eq!(b.carry_weight(150), 1, "a ×3-boosted CARRY absorbs 150 in one part");
        assert_eq!(b.carry_weight(151), 1, "still one part — but the load exceeds capacity (capped)");
    }
}
