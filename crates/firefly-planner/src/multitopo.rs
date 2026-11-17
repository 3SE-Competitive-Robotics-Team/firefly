use firefly_map::{GridMap, Plane};
use nalgebra::Vector3;

const SEG_UPBOUND_CAP: usize = 3;
const SEARCH_OUTBOUND_FACTOR: f64 = 5.0;

#[derive(Debug, Clone)]
pub struct TopoCandidate {
    pub points: Vec<Vector3<f64>>,
    pub planes: Vec<Vec<Plane>>,
}

/// `segments` 为 [`crate::obstacles::ObstacleScanner::finely_check_with_segments`]
/// 输出的碰撞段（控制点下标闭区间）。段内存在非单基点控制点时返回空。
#[must_use]
pub fn distinctive_candidates(
    map: &GridMap,
    cps_points: &[Vector3<f64>],
    cps_planes: &[Vec<Plane>],
    segments: &[crate::obstacles::CollisionSpan],
) -> Vec<TopoCandidate> {
    let single = || TopoCandidate {
        points: cps_points.to_vec(),
        planes: cps_planes.to_vec(),
    };
    if segments.is_empty() {
        return vec![single()];
    }

    let res = map.resolution();
    let cp_size = cps_points.len();
    let ctrl_pt_dist = (cps_points[0] - cps_points[cp_size - 1]).norm() / (cp_size - 1) as f64;

    let mut segments: Vec<crate::obstacles::CollisionSpan> =
        segments[..segments.len().min(SEG_UPBOUND_CAP)].to_vec();
    let mut rich: Vec<[Vec<Vec<Plane>>; 2]> = segments
        .iter()
        .map(|&(a, b)| {
            let pl: Vec<Vec<Plane>> = (a..=b).map(|j| cps_planes[j].clone()).collect();
            [pl.clone(), pl]
        })
        .collect();

    /*** Step 1:逐段构造翻转方向与镜像基点(失败段就地删除) ***/
    let mut i = 0;
    while i < segments.len() {
        match fill_flipped_side(
            map,
            &segments[i],
            &mut rich[i],
            cps_points,
            res,
            ctrl_pt_dist,
        ) {
            FlippedSide::Built => i += 1,
            FlippedSide::Dropped => {
                segments.remove(i);
                rich.remove(i);
            }
            FlippedSide::Abort => return Vec::new(),
        }
    }

    if segments.is_empty() {
        return vec![single()];
    }

    /*** Step 2:二进制选择表枚举组合 ***/
    let mut out = Vec::with_capacity(1usize << segments.len());
    for combo in 0..(1usize << segments.len()) {
        let mut planes = vec![Vec::new(); cp_size];
        let mut abandoned = false;
        let mut seg_id = 0usize;
        let mut cp_of_seg = 0usize;
        for cp_id in 0..cp_size {
            let in_seg = seg_id < segments.len()
                && (segments[seg_id].0..=segments[seg_id].1).contains(&cp_id);
            if in_seg {
                let side = (combo >> seg_id) & 1;
                let src = &rich[seg_id][side][cp_of_seg];
                if side == 1 && src.is_empty() {
                    abandoned = true;
                    break;
                }
                planes[cp_id].clone_from(src);
                cp_of_seg += 1;
                if cp_id == segments[seg_id].1 {
                    cp_of_seg = 0;
                    seg_id += 1;
                }
            } else {
                planes[cp_id].clone_from(&cps_planes[cp_id]);
            }
        }
        if !abandoned {
            out.push(TopoCandidate {
                points: cps_points.to_vec(),
                planes,
            });
        }
    }
    out
}

enum FlippedSide {
    Built,
    /// 该段被丢弃(找不到占据采样 / 镜像点过近 / 超出搜索上限)。
    Dropped,
    Abort,
}

/// 镜像点被占时沿翻转方向向外搜索自由点(`l` 从分辨率步进到上限)。
fn search_free_outward(
    map: &GridMap,
    rev_base: Vector3<f64>,
    rev_dir: Vector3<f64>,
    res: f64,
    l_upbound: f64,
) -> Option<Vector3<f64>> {
    let mut l = res;
    while l <= l_upbound {
        let cand = rev_base + l * rev_dir;
        if !map.is_occupied_inflated(cand) {
            return Some(cand);
        }
        l += res;
    }
    None
}

fn fill_flipped_side(
    map: &GridMap,
    span: &crate::obstacles::CollisionSpan,
    sides: &mut [Vec<Vec<Plane>>; 2],
    cps_points: &[Vector3<f64>],
    res: f64,
    ctrl_pt_dist: f64,
) -> FlippedSide {
    let (a, b) = *span;
    let pts = &cps_points[a..=b];
    let cp_size = pts.len();

    if cp_size > 1 {
        let occ_start = 'find_start: {
            for j in 0..cp_size - 1 {
                let step = res / (pts[j] - pts[j + 1]).norm() / 2.0;
                let mut a = 1.0;
                while a > 0.0 {
                    let pt = a * pts[j] + (1.0 - a) * pts[j + 1];
                    if map.is_occupied_inflated(pt) {
                        break 'find_start Some((j, pt));
                    }
                    a -= step;
                }
            }
            None
        };
        let Some((occ_start_id, occ_start_pt)) = occ_start else {
            return FlippedSide::Dropped;
        };
        let occ_end = 'find_end: {
            for j in (1..cp_size).rev() {
                let step = res / (pts[j] - pts[j - 1]).norm();
                let mut a = 1.0;
                while a > 0.0 {
                    let pt = a * pts[j] + (1.0 - a) * pts[j - 1];
                    if map.is_occupied_inflated(pt) {
                        break 'find_end Some((j, pt));
                    }
                    a -= step;
                }
            }
            None
        };
        let Some((occ_end_id, occ_end_pt)) = occ_end else {
            return FlippedSide::Dropped;
        };

        for j in occ_start_id..=occ_end_id {
            if sides[0][j].len() != 1 {
                return FlippedSide::Abort;
            }
            let rev_dir = -sides[0][j][0].normal();
            let rev_base = if j == occ_start_id {
                occ_start_pt
            } else if j == occ_end_id {
                occ_end_pt
            } else {
                pts[j] + rev_dir * (sides[0][j][0].point() - pts[j]).norm()
            };
            if map.is_occupied_inflated(rev_base) {
                match search_free_outward(
                    map,
                    rev_base,
                    rev_dir,
                    res,
                    SEARCH_OUTBOUND_FACTOR * ctrl_pt_dist,
                ) {
                    Some(cand) => set_first_plane(&mut sides[1][j], Plane::new(cand, rev_dir)),
                    None => return FlippedSide::Dropped,
                }
            } else if (rev_base - pts[j]).norm() >= res {
                set_first_plane(&mut sides[1][j], Plane::new(rev_base, rev_dir));
            } else {
                return FlippedSide::Dropped;
            }
        }

        if let Some(start_plane) = sides[1][occ_start_id].first().cloned() {
            for slot in sides[1][..occ_start_id].iter_mut().rev() {
                set_first_plane(slot, start_plane.clone());
            }
        }
        if let Some(end_plane) = sides[1][occ_end_id].first().cloned() {
            for slot in &mut sides[1][occ_end_id + 1..cp_size] {
                set_first_plane(slot, end_plane.clone());
            }
        }
    } else {
        return fill_single_point_side(map, sides, pts, res, ctrl_pt_dist);
    }
    FlippedSide::Built
}

fn fill_single_point_side(
    map: &GridMap,
    sides: &mut [Vec<Vec<Plane>>; 2],
    pts: &[Vector3<f64>],
    res: f64,
    ctrl_pt_dist: f64,
) -> FlippedSide {
    if sides[0][0].is_empty() {
        return FlippedSide::Dropped;
    }
    let orig = &sides[0][0][0];
    let rev_dir = -orig.normal();
    let rev_base = pts[0] + rev_dir * (orig.point() - pts[0]).norm();
    if map.is_occupied_inflated(rev_base) {
        match search_free_outward(
            map,
            rev_base,
            rev_dir,
            res,
            SEARCH_OUTBOUND_FACTOR * ctrl_pt_dist,
        ) {
            Some(cand) => set_first_plane(&mut sides[1][0], Plane::new(cand, rev_dir)),
            None => return FlippedSide::Dropped,
        }
    } else if (rev_base - pts[0]).norm() >= res {
        set_first_plane(&mut sides[1][0], Plane::new(rev_base, rev_dir));
    } else {
        return FlippedSide::Dropped;
    }
    FlippedSide::Built
}

fn set_first_plane(slot: &mut Vec<Plane>, plane: Plane) {
    if slot.is_empty() {
        slot.push(plane);
    } else {
        slot[0] = plane;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};

    const RES: f64 = 0.5;

    fn wall_map(walls: &[usize]) -> GridMap {
        let mut map = GridMapBuilder::new(RES, [40, 6, 6])
            .with_obstacles_inflation(RES * 0.99)
            .build()
            .unwrap();
        for &x in walls {
            for y in 0..6 {
                for z in 0..6 {
                    map.set_state([x, y, z], VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        map
    }

    fn line_points() -> Vec<Vector3<f64>> {
        (0..9)
            .map(|i| Vector3::new(2.0 + f64::from(i) * 2.0, 0.5, 0.5))
            .collect()
    }

    fn west_plane(points: &[Vector3<f64>], idx: usize, dist: f64) -> Plane {
        Plane::new(
            points[idx] - Vector3::new(dist, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
        )
    }

    #[test]
    fn no_segments_returns_single_identical_candidate() {
        let map = wall_map(&[]);
        let points = line_points();
        let planes: Vec<Vec<Plane>> = vec![vec![west_plane(&points, 2, 1.0)], vec![], vec![]];
        let cands = distinctive_candidates(&map, &points, &planes, &[]);
        assert_eq!(cands.len(), 1);
        assert_eq!(cands[0].points, points);
        assert_eq!(cands[0].planes[0].len(), 1);
        assert_eq!(cands[0].planes[1].len(), 0);
        assert_eq!(cands[0].planes[2].len(), 0);
        assert!((cands[0].planes[0][0].normal() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn single_segment_yields_two_candidates_with_flipped_side() {
        let map = wall_map(&[17, 18, 19, 20, 21, 22, 23]);
        let points = line_points();
        let span = (3usize, 6usize);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        for (j, slot) in planes[span.0..=span.1].iter_mut().enumerate() {
            slot.push(west_plane(&points, j + span.0, 1.5));
        }
        let cands = distinctive_candidates(&map, &points, &planes, &[span]);
        assert_eq!(cands.len(), 2, "单段 → 两候选");

        for plane in &cands[0].planes[span.0..=span.1] {
            assert!((plane[0].normal() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12);
        }
        assert!(cands[1].planes[0].is_empty());
        assert!(cands[1].planes[7].is_empty());

        for plane in &cands[1].planes[span.0..=span.1] {
            let pl = &plane[0];
            assert!(
                pl.normal().dot(&Vector3::new(-1.0, 0.0, 0.0)) < 0.0,
                "翻侧方向必为 +x"
            );
            assert!(
                pl.point().x >= 11.99,
                "翻侧基点须在自由侧,实际 x={}",
                pl.point().x
            );
        }
    }

    #[test]
    fn mirrored_point_blocked_within_search_bound_drops_segment() {
        let map = wall_map(&(16..40).collect::<Vec<_>>());
        let points = line_points();
        let span = (3usize, 5usize);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        for (j, slot) in planes[span.0..=span.1].iter_mut().enumerate() {
            slot.push(west_plane(&points, j + span.0, 1.5));
        }
        let cands = distinctive_candidates(&map, &points, &planes, &[span]);
        assert_eq!(cands.len(), 1);
        for plane in &cands[0].planes[span.0..=span.1] {
            assert_eq!(plane.len(), 1);
            assert!(
                (plane[0].normal() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12,
                "回落候选应保持原侧"
            );
        }
    }

    #[test]
    fn two_segments_enumerate_four_combinations_in_official_order() {
        let map = wall_map(&[15, 31]);
        let points = line_points();
        let spans = [(2usize, 3usize), (6usize, 7usize)];
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        for &(a, b) in &spans {
            for (j, slot) in planes[a..=b].iter_mut().enumerate() {
                slot.push(west_plane(&points, j + a, 1.5));
            }
        }
        let cands = distinctive_candidates(&map, &points, &planes, &spans);
        assert_eq!(cands.len(), 4, "两段 → 四组合");
        let flipped = |c: &TopoCandidate, j: usize| {
            c.planes[j][0].normal().dot(&Vector3::new(-1.0, 0.0, 0.0)) < 0.0
        };
        assert!(!flipped(&cands[0], 3) && !flipped(&cands[0], 6), "[00]");
        assert!(flipped(&cands[1], 3) && !flipped(&cands[1], 6), "[10]");
        assert!(!flipped(&cands[2], 3) && flipped(&cands[2], 6), "[01]");
        assert!(flipped(&cands[3], 3) && flipped(&cands[3], 6), "[11]");
    }
}
