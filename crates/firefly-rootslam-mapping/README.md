# firefly-rootslam-mapping

Persistent mapping, loop detection and verification, pose-graph optimization,
and global bundle adjustment based on the RootSLAM paper. The authors'
implementation is unpublished; this crate implements the described methods
without claiming an exact source or performance reproduction.

## Optimization edge set

The pose graph holds four kinds of edges. Only the measurement-bearing ones
take part in the optimization, matching the paper, which relates keyframes
"through the odometry and through previously closed loops" (section III-C):

| Kind | Measurement | In Equation (1) and (9) |
| --- | --- | --- |
| `Temporal` | odometry relative pose at insertion | yes |
| `Loop` | gP3P estimate of a validated loop closure | yes |
| `Covisibility` | the relative pose the map holds right now | no |
| `SpanningTree` | the relative pose the map holds right now | no |

The covisibility graph and its spanning forest restate the current estimates,
so their residual is zero by construction. They keep the graph connected and
group retrieval candidates into islands; as optimization constraints they would
only add stiffness that pins the map to its present shape.

## References

- [RootSLAM paper](https://arxiv.org/abs/2610.15157)
- [Workspace architecture](../../docs/architecture.md#basalt-与-rootslam-独立应用)
- [License index](../../docs/licenses/README.md)
