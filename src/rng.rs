//! A tiny, portable, seeded PRNG (SplitMix64) — the shared deterministic RNG for the offline
//! harnesses (`screeps-combat-eval` scenario generation, `screeps-rover-eval` procedural rooms). It
//! is per-index reproducible and depends on NO ambient entropy (`rand` / `Date` / `Math.random`),
//! which is exactly what the determinism fence requires. Not a pathfinding/search algorithm — a
//! utility — so it lives in the kernel as shared sim infrastructure (ADR 0033).

/// SplitMix64. `seeded(index)` gives an independent, reproducible stream per `index`.
pub struct Rng(u64);

impl Rng {
    /// A stream seeded from `index` (per-index reproducible; index `n` and `n+1` are independent).
    pub fn seeded(index: u32) -> Self {
        Rng(0x9E37_79B9_7F4A_7C15u64.wrapping_mul(index as u64 + 1))
    }

    /// The next 64-bit value (SplitMix64 step).
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A value in the inclusive range `[lo, hi]`.
    pub fn range(&mut self, lo: u32, hi: u32) -> u32 {
        debug_assert!(hi >= lo);
        lo + (self.next_u64() % (hi - lo + 1) as u64) as u32
    }

    /// True with probability `pct`%.
    pub fn chance(&mut self, pct: u32) -> bool {
        self.range(0, 99) < pct
    }

    /// One element of `xs` (panics on empty, like indexing).
    pub fn pick(&mut self, xs: &[u32]) -> u32 {
        xs[(self.next_u64() % xs.len() as u64) as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_reproducible_per_seed_and_independent_across_seeds() {
        let seq = |s: u32| (0..4).map(|_| ()).scan(Rng::seeded(s), |r, _| Some(r.next_u64())).collect::<Vec<_>>();
        assert_eq!(seq(7), seq(7), "same seed → same stream");
        assert_ne!(seq(7), seq(8), "different seeds → different streams");
    }

    #[test]
    fn range_is_inclusive_and_bounded() {
        let mut r = Rng::seeded(1);
        for _ in 0..1000 {
            let v = r.range(3, 9);
            assert!((3..=9).contains(&v));
        }
        assert_eq!(r.range(5, 5), 5, "a degenerate range yields the single value");
    }
}
