//! 多拓扑候选轨迹生成——严格对照官方 EGO-Planner-v2
//! `poly_traj_optimizer.cpp::distinctiveTrajs`(L816–1125)。
//!
//! 输入为初始轨迹的约束点、fine check 生成的 {s,v} 平面池与碰撞段
//! (控制点下标闭区间,官方 `segments`)。每个候选与输入约束点完全相同,
//! 仅平面分配不同:每段可取原侧(0)或翻侧(1),二进制组合枚举
//! ≤ 2^3 = 8 个候选(官方 `MAX_TRAJS=8`、`VARIS=2`)。

use firefly_map::{GridMap, Plane};
use nalgebra::Vector3;

/// 段数上限:官方 `min(段数, floor(log(MAX_TRAJS)/log(VARIS)))` = 3。
const SEG_UPBOUND_CAP: usize = 3;
/// 镜像点向外搜索上限倍数:官方 `"5" is the threshold`。
const SEARCH_OUTBOUND_FACTOR: f64 = 5.0;

/// 单个拓扑候选:约束点 + 按约束点索引组织的平面池
/// (官方 `ConstraintPoints`;控制点位置与输入一致,仅平面分配不同)。
#[derive(Debug, Clone)]
pub struct TopoCandidate {
    pub points: Vec<Vector3<f64>>,
    pub planes: Vec<Vec<Plane>>,
}

/// 官方 `distinctiveTrajs`:由碰撞段生成拓扑互异的候选平面集。
///
/// `segments` 为 [`crate::obstacles::ObstacleScanner::finely_check_with_segments`]
/// 输出的碰撞段(控制点下标闭区间)。返回空表示整体中止(官方返回 blank:
/// 段内出现非单基点的控制点,不应发生)。
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
    // 官方:无碰撞段 → 返回 [cps_] 单候选
    if segments.is_empty() {
        return vec![single()];
    }

    let res = map.resolution();
    let cp_size = cps_points.len();
    // 官方 CTRL_PT_DIST:首尾控制点直线距离 / (cp_size − 1)
    let ctrl_pt_dist = (cps_points[0] - cps_points[cp_size - 1]).norm() / (cp_size - 1) as f64;

    // 段列表截断到上限(官方 seg_upbound)
    let mut segments: Vec<crate::obstacles::CollisionSpan> =
        segments[..segments.len().min(SEG_UPBOUND_CAP)].to_vec();
    // RichInfoSegs:每段 [原侧, 翻侧] 平面池(段内局部索引);两侧控制点相同
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
        // 官方:全部段被删后回落 [cps_] 单候选
        return vec![single()];
    }

    /*** Step 2:二进制选择表枚举组合 ***/
    // 枚举顺序与官方计数器(selection[0] 为最低位,全 0 原侧在前)一致。
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
                // 官方:选中翻侧但翻侧为空 → 弃该组合
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
                // 段外:复制 cps_ 原值
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

/// Step 1 的单段处理结果。
enum FlippedSide {
    /// 翻侧已构造完成。
    Built,
    /// 该段被丢弃(找不到占据采样 / 镜像点过近 / 超出搜索上限)。
    Dropped,
    /// 整体中止(段内出现非单基点控制点,官方 `ROS_ERROR` + 返回 blank)。
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

/// 官方 Step 1:为一段构造翻转侧平面(写入 `sides[1]`,段内局部索引)。
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
        // 1.1 首/末占据采样点(沿控制点折线,步长≈分辨率;
        // 首向循环采样更密——对照官方两处 step_size 的差异)
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

        // 1.2 翻转方向 + 镜像基点([occ_start, occ_end])
        for j in occ_start_id..=occ_end_id {
            // 官方:恰好 1 个基点,否则报错并放弃全部候选
            if sides[0][j].len() != 1 {
                return FlippedSide::Abort;
            }
            let rev_dir = -sides[0][j][0].normal();
            // 段端点用占据采样点特判,其余镜像原基距
            let rev_base = if j == occ_start_id {
                occ_start_pt
            } else if j == occ_end_id {
                occ_end_pt
            } else {
                pts[j] + rev_dir * (sides[0][j][0].point() - pts[j]).norm()
            };
            if map.is_occupied_inflated(rev_base) {
                // 沿翻转方向向外搜索自由点,上限 5×平均控制点间距
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
                // 镜像点距控制点过近(too close)→ 丢段
                return FlippedSide::Dropped;
            }
        }

        // 1.3 段外控制点复制 occ 端点的翻侧值
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

/// 官方 `else` 分支:单控制点段(`cp_size == 1`)的翻侧构造。
fn fill_single_point_side(
    map: &GridMap,
    sides: &mut [Vec<Vec<Plane>>; 2],
    pts: &[Vector3<f64>],
    res: f64,
    ctrl_pt_dist: f64,
) -> FlippedSide {
    if sides[0][0].is_empty() {
        // 官方直接读 [0][0];空平面无法翻转,丢段
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

/// 替换第 0 个平面、保留其余(对照官方 `[0] =` 元素赋值语义)。
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

    /// 合成地图:分辨率 0.5,`walls` 内的体素列全高占据,膨胀 1 格
    /// (`is_occupied_inflated` 只看膨胀层;半径 0 会得到全空膨胀层)。
    fn wall_map(walls: &[usize]) -> GridMap {
        let mut map = GridMapBuilder::new(RES, [40, 6, 6]).build().unwrap();
        for &x in walls {
            for y in 0..6 {
                for z in 0..6 {
                    map.set_state([x, y, z], VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles(RES * 0.99);
        map
    }

    /// 沿 x 轴的合成控制点:x = 2..=18 步长 2(9 点,间距 2 m)。
    fn line_points() -> Vec<Vector3<f64>> {
        (0..9)
            .map(|i| Vector3::new(2.0 + f64::from(i) * 2.0, 0.5, 0.5))
            .collect()
    }

    /// 给下标 `idx` 的控制点一个朝 −x 的原侧平面(基点在点西侧 `dist` 处)。
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
        // 体素 17..23 占据(含 1 格膨胀 → 占据区 x∈[8.0,12.0))
        let map = wall_map(&[17, 18, 19, 20, 21, 22, 23]);
        let points = line_points();
        // 段覆盖下标 3..=6(x = 8..14,穿墙),原侧平面朝 −x、基点在点西 1.5 m
        let span = (3usize, 6usize);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        for (j, slot) in planes[span.0..=span.1].iter_mut().enumerate() {
            slot.push(west_plane(&points, j + span.0, 1.5));
        }
        let cands = distinctive_candidates(&map, &points, &planes, &[span]);
        assert_eq!(cands.len(), 2, "单段 → 两候选");

        // 候选 0(全原侧)= 输入
        for j in span.0..=span.1 {
            assert!((cands[0].planes[j][0].normal() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12);
        }
        // 候选 1:段外复制输入(此处无平面)
        assert!(cands[1].planes[0].is_empty());
        assert!(cands[1].planes[7].is_empty());

        // 候选 1(翻侧):段内方向相反(+x),基点在自由侧(x ≥ 12.0)
        for j in span.0..=span.1 {
            let pl = &cands[1].planes[j][0];
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
        // 体素 16 起全图占据(含 1 格膨胀 → 占据区 x∈[7.5,20)):
        // 镜像点被占且 5×间距(10 m)内向 +x 无自由点(越界亦视为占据)
        let map = wall_map(&(16..40).collect::<Vec<_>>());
        let points = line_points();
        let span = (3usize, 5usize);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        for (j, slot) in planes[span.0..=span.1].iter_mut().enumerate() {
            slot.push(west_plane(&points, j + span.0, 1.5));
        }
        let cands = distinctive_candidates(&map, &points, &planes, &[span]);
        // 段被丢弃后回落单候选(官方 seg_upbound==0 分支),且等于输入
        assert_eq!(cands.len(), 1);
        for j in span.0..=span.1 {
            assert_eq!(cands[0].planes[j].len(), 1);
            assert!(
                (cands[0].planes[j][0].normal() - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12,
                "回落候选应保持原侧"
            );
        }
    }

    #[test]
    fn two_segments_enumerate_four_combinations_in_official_order() {
        // 两堵分离薄墙(体素 15 与 31,含膨胀 → 占据区 [7.0,8.5)/[15.0,16.5)),
        // 段互不相邻 → 4 组合,枚举顺序与官方计数器一致:
        // [00, 10, 01, 11](selection[0] 为最低位)
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
        // 原侧方向 −x,翻侧 +x;combo 位 k 对应第 k 段(bit0 → 段 2..=3)
        let flipped = |c: &TopoCandidate, j: usize| {
            c.planes[j][0].normal().dot(&Vector3::new(-1.0, 0.0, 0.0)) < 0.0
        };
        assert!(!flipped(&cands[0], 3) && !flipped(&cands[0], 6), "[00]");
        assert!(flipped(&cands[1], 3) && !flipped(&cands[1], 6), "[10]");
        assert!(!flipped(&cands[2], 3) && flipped(&cands[2], 6), "[01]");
        assert!(flipped(&cands[3], 3) && flipped(&cands[3], 6), "[11]");
    }
}
