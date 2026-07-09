//! Procedural room-terrain generation (ADR 0044) — a SHARED, engine-like generator used uniformly
//! by the economy, combat, and pathfinding simulators to replace trivial open/synthetic rooms with
//! realistic CLUSTERED walls + swamp patches + guaranteed connected exits. Trivial terrain
//! (open rooms, single-file synthetic corridors) doesn't stress pathfinding/positioning and can't
//! prove behaviour generalises; this makes `routed ≫ Chebyshev` the normal case (the regime the
//! ADR 0044 true-distance haul pricing must handle) and gives combat real cover/chokepoints.
//!
//! Algorithm: cellular-automata (cave) walls give organic clustering; a fully-walled border with a
//! carved channel from each requested EXIT to the room centre guarantees that every exit and the
//! interior are one connected region (so adjacent generated rooms, whose mid-edge exits align at
//! tile 25, chain into a traversable corridor). Deterministic in `seed`.

use crate::rng::Rng;
use crate::terrain::SimTerrain;

const N: usize = 50;
/// Exits sit at the CENTRE of each edge (tile 25) so adjacent rooms' opposing exits align — a
/// room's right exit at (49,25) meets the neighbour's left exit at (0,25).
const MID: usize = 25;

/// Which room edges carry an open EXIT (a gap in the border wall + a carved channel to the interior
/// centre). Rooms in a multi-room corridor set the connecting edges (e.g. [`Exits::horizontal`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Exits {
    pub top: bool,    // y = 0
    pub bottom: bool, // y = 49
    pub left: bool,   // x = 0
    pub right: bool,  // x = 49
}

impl Exits {
    pub fn all() -> Self {
        Exits { top: true, bottom: true, left: true, right: true }
    }
    /// A straight-through corridor room (left↔right).
    pub fn horizontal() -> Self {
        Exits { left: true, right: true, ..Default::default() }
    }
}

/// Terrain-generation parameters. Defaults give a ~cave-like room with moderate swamp.
#[derive(Clone, Copy, Debug)]
pub struct TerrainGenParams {
    /// Initial random wall fill (per-mille) before smoothing — ~450 (45%) yields cave-like density.
    pub wall_fill_permille: u32,
    /// Cellular-automata smoothing passes (a tile becomes wall iff ≥5 of its 8 neighbours are wall).
    pub smooth_iters: u32,
    /// Swamp fill (per-mille of the non-wall tiles).
    pub swamp_fill_permille: u32,
    /// Which edges have a connected exit.
    pub exits: Exits,
}

impl Default for TerrainGenParams {
    fn default() -> Self {
        TerrainGenParams { wall_fill_permille: 450, smooth_iters: 4, swamp_fill_permille: 120, exits: Exits::all() }
    }
}

/// 8-connected flood-fill of the open (non-wall) tiles reachable from `start`. Matches creep
/// movement (diagonals allowed), so a tile in the returned region is genuinely creep-reachable.
fn flood_open(wall: &[[bool; N]; N], start: (usize, usize)) -> [[bool; N]; N] {
    let mut region = [[false; N]; N];
    let mut stack = vec![start];
    while let Some((x, y)) = stack.pop() {
        if wall[y][x] || region[y][x] {
            continue;
        }
        region[y][x] = true;
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if (0..N as i32).contains(&nx) && (0..N as i32).contains(&ny) {
                    stack.push((nx as usize, ny as usize));
                }
            }
        }
    }
    region
}

/// Generate a deterministic, engine-like room terrain (see module docs). Every requested exit is
/// open and connected to the room centre.
pub fn generate_terrain(seed: u32, params: &TerrainGenParams) -> SimTerrain {
    let mut rng = Rng::seeded(seed);

    // 1. Random fill; the border is always wall (exits are carved back out in step 3).
    let mut wall = [[false; N]; N];
    for (y, row) in wall.iter_mut().enumerate() {
        for (x, cell) in row.iter_mut().enumerate() {
            *cell = if x == 0 || y == 0 || x == N - 1 || y == N - 1 {
                true
            } else {
                rng.range(0, 1000) < params.wall_fill_permille
            };
        }
    }

    // 2. Cellular-automata smoothing (interior only; the border stays wall and counts as a wall
    //    neighbour, which naturally thickens edges — cave-like).
    for _ in 0..params.smooth_iters {
        let mut next = wall;
        for y in 1..N - 1 {
            for x in 1..N - 1 {
                let mut neighbours = 0u32;
                for dy in -1i32..=1 {
                    for dx in -1i32..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        if wall[(y as i32 + dy) as usize][(x as i32 + dx) as usize] {
                            neighbours += 1;
                        }
                    }
                }
                next[y][x] = neighbours >= 5;
            }
        }
        wall = next;
    }

    // 3. Connect the exits to the NATURAL cave, not via a straight highway: seed the centre open,
    //    flood-fill it to find the main region, then connect each requested exit with a SHORT
    //    inward carve that stops at the first region tile. The bulk of any cross-room path then
    //    winds through the cave (routed ≫ straight-line — the whole point of realistic terrain),
    //    while connectivity is still guaranteed. The exit band is `MID±1` (3-wide), aligned with
    //    the neighbour room's opposing exit.
    for dy in -1i32..=1 {
        for dx in -1i32..=1 {
            wall[(MID as i32 + dy) as usize][(MID as i32 + dx) as usize] = false;
        }
    }
    let region = flood_open(&wall, (MID, MID));
    let mut connect = |wall: &mut [[bool; N]; N], edge: (i32, i32), inward: (i32, i32)| {
        let (mut x, mut y) = edge;
        loop {
            wall[y as usize][x as usize] = false;
            let (nx, ny) = (x + inward.0, y + inward.1);
            if nx < 0 || ny < 0 || nx >= N as i32 || ny >= N as i32 {
                break;
            }
            if region[ny as usize][nx as usize] {
                break; // reached the natural cave region — let it carry the rest of the path
            }
            x = nx;
            y = ny;
        }
    };
    for off in -1i32..=1 {
        let b = MID as i32 + off;
        if params.exits.left {
            connect(&mut wall, (0, b), (1, 0));
        }
        if params.exits.right {
            connect(&mut wall, (N as i32 - 1, b), (-1, 0));
        }
        if params.exits.top {
            connect(&mut wall, (b, 0), (0, 1));
        }
        if params.exits.bottom {
            connect(&mut wall, (b, N as i32 - 1), (0, -1));
        }
    }

    // 4. Materialise walls; swamp the non-wall tiles at the requested density.
    let mut terrain = SimTerrain::default();
    for y in 0..N {
        for x in 0..N {
            if wall[y][x] {
                terrain.walls.insert((x as u8, y as u8));
            } else if rng.range(0, 1000) < params.swamp_fill_permille {
                terrain.swamps.insert((x as u8, y as u8));
            }
        }
    }
    terrain
}

/// The mid-edge exit tile for an edge — the canonical connection point (tile 25). Adjacent rooms'
/// opposing exits meet here.
pub const EXIT_MID: u8 = MID as u8;

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    /// Flood-fill the walkable region from the centre; return the reached tiles.
    fn reachable_from_centre(t: &SimTerrain) -> HashSet<(u8, u8)> {
        let mut seen = HashSet::new();
        let mut stack = vec![(MID as u8, MID as u8)];
        while let Some((x, y)) = stack.pop() {
            if t.walls.contains(&(x, y)) || !seen.insert((x, y)) {
                continue;
            }
            for (dx, dy) in [(1i32, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if (0..N as i32).contains(&nx) && (0..N as i32).contains(&ny) {
                    stack.push((nx as u8, ny as u8));
                }
            }
        }
        seen
    }

    /// Deterministic in the seed; has real clustered walls (not open, not fully walled); every
    /// requested exit is open AND connected to the centre.
    #[test]
    fn generates_connected_exits_and_real_walls() {
        let p = TerrainGenParams { exits: Exits::horizontal(), ..Default::default() };
        let t = generate_terrain(7, &p);
        let t2 = generate_terrain(7, &p);
        assert_eq!(t.walls, t2.walls, "deterministic in seed");
        assert_eq!(t.swamps, t2.swamps);
        // Real terrain: a substantial-but-not-total wall count (interior has cave structure).
        assert!(t.walls.len() > 200 && t.walls.len() < 2200, "clustered walls, not open/solid: {}", t.walls.len());
        // The horizontal exits are open and both reachable from the centre.
        assert!(!t.walls.contains(&(0, EXIT_MID)), "left exit open");
        assert!(!t.walls.contains(&(49, EXIT_MID)), "right exit open");
        let reach = reachable_from_centre(&t);
        assert!(reach.contains(&(0, EXIT_MID)), "left exit connects to centre");
        assert!(reach.contains(&(49, EXIT_MID)), "right exit connects to centre");
        // A non-requested edge stays sealed (no top exit).
        assert!(t.walls.contains(&(EXIT_MID, 0)), "unrequested top edge stays walled");
    }

    /// Different seeds give different rooms (variety).
    #[test]
    fn seeds_vary() {
        let p = TerrainGenParams::default();
        assert_ne!(generate_terrain(1, &p).walls, generate_terrain(2, &p).walls);
    }
}
