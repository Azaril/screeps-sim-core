//! Body + movement constants transcribed from the Screeps engine (`common/constants.js`,
//! `processor/intents/movement.js`, `creeps/tick.js`). Verified against `C:\code\screeps-engine`.
//! Combat-only constants (action ranges, rangedMassAttack falloff, tower powers,
//! `attackController` per-part) stay in `screeps-combat-engine`.

/// Hit points per body part (`BODYPART_HITS`).
pub const BODYPART_HITS: u32 = 100;

/// Creep lifetimes (`CREEP_LIFE_TIME` / `CREEP_CLAIM_LIFE_TIME`).
pub const CREEP_LIFE_TIME: u32 = 1500;
pub const CREEP_CLAIM_LIFE_TIME: u32 = 600;

// ── Per-part action power (unboosted) — a property of a body part ────────────
pub const ATTACK_POWER: u32 = 30; // ATTACK, melee, range 1
pub const RANGED_ATTACK_POWER: u32 = 10; // RANGED_ATTACK, range 3
pub const HEAL_POWER: u32 = 12; // HEAL adjacent, range 1
pub const RANGED_HEAL_POWER: u32 = 4; // HEAL at range, range 3
pub const DISMANTLE_POWER: u32 = 50; // WORK dismantle, range 1

// ── Movement / fatigue (movement.js) ────────────────────────────────────────
/// Fatigue added per non-MOVE/non-CARRY part per step, by terrain.
pub const FATIGUE_RATE_ROAD: u32 = 1;
pub const FATIGUE_RATE_PLAIN: u32 = 2;
pub const FATIGUE_RATE_SWAMP: u32 = 10;
/// Fatigue cleared per (unboosted) MOVE part per tick (`-2 * moves`, `creeps/tick.js:107`).
pub const FATIGUE_CLEAR_PER_MOVE: u32 = 2;
