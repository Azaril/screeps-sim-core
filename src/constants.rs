//! Movement + body-sizing constants transcribed from the Screeps engine (`common/constants.js`,
//! `processor/intents/movement.js`, `creeps/tick.js`). Verified against `C:\code\screeps-engine`.
//! This kernel is pure MOVEMENT mechanics: body-COMBAT constants (per-part action power, creep
//! lifetimes) live in the combat layer `screeps-combat-engine` alongside the `SimBodyCombat`
//! extension trait (ADR 0033), NOT here — the mover never needs them.

/// Hit points per body part (`BODYPART_HITS`).
pub const BODYPART_HITS: u32 = 100;

// ── Movement / fatigue (movement.js) ────────────────────────────────────────
/// Fatigue added per non-MOVE/non-CARRY part per step, by terrain.
pub const FATIGUE_RATE_ROAD: u32 = 1;
pub const FATIGUE_RATE_PLAIN: u32 = 2;
pub const FATIGUE_RATE_SWAMP: u32 = 10;
/// Fatigue cleared per (unboosted) MOVE part per tick (`-2 * moves`, `creeps/tick.js:107`).
pub const FATIGUE_CLEAR_PER_MOVE: u32 = 2;

/// Resource units one unboosted CARRY part holds (`CARRY_CAPACITY`). The engine's move-fatigue
/// `weight` gains one unit per ALIVE CARRY part needed to hold the load (`movement.js:41`
/// `calcResourcesWeight`), each part absorbing `CARRY_CAPACITY × capacity_boost` (×1/2/3/4).
pub const CARRY_CAPACITY: u32 = 50;
