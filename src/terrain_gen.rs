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
use screeps::{Position, RoomCoordinate, RoomName};

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

/// The set of open (non-wall) tiles 8-connected to `start` in a `SimTerrain` — the creep-reachable
/// region containing `start`. General helper for placing things (spawns, sources, squads, path
/// endpoints) on CONNECTED terrain: an arbitrary tile in a generated room may be a wall or in an
/// isolated pocket, so callers pick from this set. Swamps count as open (passable). Empty if `start`
/// is itself a wall.
pub fn connected_open(terrain: &SimTerrain, start: (u8, u8)) -> std::collections::HashSet<(u8, u8)> {
    let mut region = std::collections::HashSet::new();
    if terrain.walls.contains(&start) {
        return region;
    }
    let mut stack = vec![start];
    while let Some((x, y)) = stack.pop() {
        if terrain.walls.contains(&(x, y)) || !region.insert((x, y)) {
            continue;
        }
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if (0..N as i32).contains(&nx) && (0..N as i32).contains(&ny) {
                    stack.push((nx as u8, ny as u8));
                }
            }
        }
    }
    region
}

/// A room edge — which side an exit range sits on.
#[derive(Clone, Copy, Debug)]
pub enum EdgeDir {
    Left,
    Right,
    Top,
    Bottom,
}

/// The generation CORE: a cellular-automata cave plus the given per-edge open EXIT RANGES (inclusive
/// tile ranges), each connected to the interior by a short inward carve to the natural cave region.
/// Callers supply the ranges — a fixed band ([`generate_terrain`]) or SHARED seam-derived ranges
/// ([`generate_terrain_for_room`], the engine-like carving-free alignment).
fn generate_with_edges(cave_seed: u32, params: &TerrainGenParams, edges: &[(EdgeDir, u8, u8)]) -> SimTerrain {
    let mut rng = Rng::seeded(cave_seed);

    // 1. Random fill; the border is always wall (exits are opened in step 3).
    let mut wall = [[false; N]; N];
    for (y, row) in wall.iter_mut().enumerate() {
        for (x, cell) in row.iter_mut().enumerate() {
            *cell = x == 0 || y == 0 || x == N - 1 || y == N - 1 || rng.range(0, 1000) < params.wall_fill_permille;
        }
    }

    // 2. Cellular-automata smoothing (interior only; the border stays wall and counts as a wall
    //    neighbour, which thickens edges — cave-like).
    for _ in 0..params.smooth_iters {
        let mut next = wall;
        for y in 1..N - 1 {
            for x in 1..N - 1 {
                let mut neighbours = 0u32;
                for dy in -1i32..=1 {
                    for dx in -1i32..=1 {
                        if (dx != 0 || dy != 0) && wall[(y as i32 + dy) as usize][(x as i32 + dx) as usize] {
                            neighbours += 1;
                        }
                    }
                }
                next[y][x] = neighbours >= 5;
            }
        }
        wall = next;
    }

    // 3. Seed the centre open, flood-fill it → the main region, and connect each edge exit tile with
    //    a SHORT inward carve that stops at the first region tile. The cave (not a carved highway)
    //    carries the bulk of a cross-room path (routed ≫ straight-line); connectivity guaranteed.
    for dy in -1i32..=1 {
        for dx in -1i32..=1 {
            wall[(MID as i32 + dy) as usize][(MID as i32 + dx) as usize] = false;
        }
    }
    let region = flood_open(&wall, (MID, MID));
    let connect = |wall: &mut [[bool; N]; N], edge: (i32, i32), inward: (i32, i32)| {
        let (mut x, mut y) = edge;
        loop {
            wall[y as usize][x as usize] = false;
            let (nx, ny) = (x + inward.0, y + inward.1);
            if nx < 0 || ny < 0 || nx >= N as i32 || ny >= N as i32 || region[ny as usize][nx as usize] {
                break;
            }
            x = nx;
            y = ny;
        }
    };
    for &(dir, lo, hi) in edges {
        for b in lo..=hi {
            let b = b as i32;
            let (edge, inward) = match dir {
                EdgeDir::Left => ((0, b), (1, 0)),
                EdgeDir::Right => ((N as i32 - 1, b), (-1, 0)),
                EdgeDir::Top => ((b, 0), (0, 1)),
                EdgeDir::Bottom => ((b, N as i32 - 1), (0, -1)),
            };
            connect(&mut wall, edge, inward);
        }
    }

    // 4. Materialise walls; swamp the non-wall tiles at the requested density.
    let mut terrain = SimTerrain::default();
    for (y, row) in wall.iter().enumerate() {
        for (x, &is_wall) in row.iter().enumerate() {
            if is_wall {
                terrain.walls.insert((x as u8, y as u8));
            } else if rng.range(0, 1000) < params.swamp_fill_permille {
                terrain.swamps.insert((x as u8, y as u8));
            }
        }
    }
    terrain
}

/// A stable per-room id — its centre tile's global packed coordinate (unique per room).
fn room_id(r: RoomName) -> u32 {
    Position::new(RoomCoordinate::new(MID as u8).unwrap(), RoomCoordinate::new(MID as u8).unwrap(), r).packed_repr()
}

/// The neighbour room across an edge (via `checked_add` in global space), if on the map.
fn neighbour(r: RoomName, tile: (u8, u8), off: (i32, i32)) -> Option<RoomName> {
    Position::new(RoomCoordinate::new(tile.0).unwrap(), RoomCoordinate::new(tile.1).unwrap(), r)
        .checked_add(off)
        .ok()
        .map(|p| p.room_name())
}

/// The shared EXIT RANGE for the seam between two rooms — ORDER-INDEPENDENT in their ids, so BOTH
/// rooms compute the identical `(lo, hi)` for their shared edge. This is what makes adjacent rooms'
/// exits align with NO fix-up carving (the engine's aligned-exit invariant, realistic + varied):
/// a room's east exit and its neighbour's west exit derive from the same seam. Clear of the corners
/// (5..44) so diagonal edge-relocation is never involved.
pub fn seam_range(id_a: u32, id_b: u32) -> (u8, u8) {
    let (lo, hi) = (id_a.min(id_b) as u64, id_a.max(id_b) as u64);
    let mut s = lo.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ hi.wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    s ^= s >> 31;
    let centre = 8 + (s % 34) as u8; // 8..41
    let half = 1 + ((s >> 20) % 3) as u8; // 1..3 ⇒ 3..7-wide mouth
    (centre - half, centre + half)
}

/// The shared exit range for the seam between two adjacent rooms, by name — the range BOTH rooms
/// open on their shared edge. A CAPTURED (non-generated) room bordering a generated one carves its
/// edge to exactly this range to align with the generated neighbour's [`generate_terrain_for_room`]
/// exit (no guesswork, no mismatch).
pub fn seam_range_between(a: RoomName, b: RoomName) -> (u8, u8) {
    seam_range(room_id(a), room_id(b))
}

/// (Fixed-band, standalone.) Every requested edge opens the mid-edge band `MID±1`. For a single
/// room or a corridor where the aligned band is a constant; prefer [`generate_terrain_for_room`] for
/// a real multi-room map where seams must match by construction.
pub fn generate_terrain(seed: u32, params: &TerrainGenParams) -> SimTerrain {
    let (lo, hi) = (MID as u8 - 1, MID as u8 + 1);
    let mut edges = Vec::new();
    if params.exits.left {
        edges.push((EdgeDir::Left, lo, hi));
    }
    if params.exits.right {
        edges.push((EdgeDir::Right, lo, hi));
    }
    if params.exits.top {
        edges.push((EdgeDir::Top, lo, hi));
    }
    if params.exits.bottom {
        edges.push((EdgeDir::Bottom, lo, hi));
    }
    generate_with_edges(seed, params, &edges)
}

/// ROOM-AWARE generation — the ENGINE-LIKE path. Each OPEN edge (`connect`) takes its exit range
/// from the SHARED seam with the neighbour, so adjacent rooms match by construction with no carving;
/// walled edges have no exit. The cave is seeded from the room identity. Use this for a real
/// multi-room world (economy remotes, combat rooms, pathfinding corpora) so every seam is a valid,
/// aligned entry/exit — a creep never relocates into a wall.
pub fn generate_terrain_for_room(room: RoomName, world_seed: u32, connect: Exits, params: &TerrainGenParams) -> SimTerrain {
    let self_id = room_id(room);
    let mut edges = Vec::new();
    let add = |edges: &mut Vec<(EdgeDir, u8, u8)>, on: bool, dir: EdgeDir, tile: (u8, u8), off: (i32, i32)| {
        if on {
            if let Some(n) = neighbour(room, tile, off) {
                let (lo, hi) = seam_range(self_id, room_id(n));
                edges.push((dir, lo, hi));
            }
        }
    };
    add(&mut edges, connect.left, EdgeDir::Left, (0, MID as u8), (-1, 0));
    add(&mut edges, connect.right, EdgeDir::Right, (49, MID as u8), (1, 0));
    add(&mut edges, connect.top, EdgeDir::Top, (MID as u8, 0), (0, -1));
    add(&mut edges, connect.bottom, EdgeDir::Bottom, (MID as u8, 49), (0, 1));
    generate_with_edges(world_seed ^ self_id, params, &edges)
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

    /// THE alignment invariant: two horizontally-adjacent rooms, each generated INDEPENDENTLY via
    /// `generate_terrain_for_room`, have IDENTICAL open tiles on their shared seam — room A's east
    /// edge (x=49) matches room B's west edge (x=0) tile-for-tile, with no cross-room carving. This
    /// is the engine's aligned-exit property: a creep crossing the seam never lands on a wall.
    #[test]
    fn adjacent_rooms_share_a_matching_seam() {
        let a: RoomName = "E20N20".parse().unwrap();
        let b = Position::new(RoomCoordinate::new(49).unwrap(), RoomCoordinate::new(25).unwrap(), a)
            .checked_add((1, 0))
            .unwrap()
            .room_name(); // the room one step EAST of A
        let p = TerrainGenParams::default();
        let ta = generate_terrain_for_room(a, 42, Exits::all(), &p);
        let tb = generate_terrain_for_room(b, 42, Exits::all(), &p);
        let mut open_seam = 0;
        for y in 0..50u8 {
            let a_east_open = !ta.walls.contains(&(49, y));
            let b_west_open = !tb.walls.contains(&(0, y));
            assert_eq!(a_east_open, b_west_open, "seam tile y={y}: A.east={a_east_open} B.west={b_west_open}");
            if a_east_open {
                open_seam += 1;
            }
        }
        assert!(open_seam >= 3, "the seam has a real (≥3-wide) shared exit, not a sealed edge");
    }
}
