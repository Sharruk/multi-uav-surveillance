# Scenario Technical Specifications — STIRS-2025

Per-scenario deep-dive for all five simulation environments. Numbers reflect the actual PyBullet world (Phase 3, `fixed_layout=True`, seed 42).

---

## Scenario: Downtown

### Environment Composition

| Property | Value |
|:---------|:------|
| City grid | 3 × 3 blocks |
| City size | 26 × 26 m |
| Block size | 6.0 m |
| Road width | 2.0 m |
| Road type | 6 vertical + 2 horizontal (full grid) |
| Building density | High |
| Building height | 6–18 m |
| Arena XY bound | 15 m |

### Crowd Dynamics

| Property | Value |
|:---------|:------|
| Total crowd agents | **15** |
| Sidewalk walkers | 10 (distributed across all road-parallel strips) |
| Bus-stop gatherers | 5 (waiting at road-edge positions near city periphery) |
| Agent speed | 0.8–1.5 m/s (uniform random) |
| Pedestrian body | GEOM_SPHERE collision r=0.15 m + GEOM_CYLINDER visual h=1.16 m |
| LiDAR detectable | Yes (collidable sphere, detectable below ≈1.5 m drone altitude) |
| State machine | WALK → waypoint on strip; WAIT → 40–200 steps; bus stop agents re-wait 80–200 steps |
| Clothing variety | 12 colour variants (random per agent) |

### Crowd Zone Geometry

| Zone | Centre | Radius | Density |
|:-----|:-------|-------:|:--------|
| Central area | (0, 0) | 7.0 m | High |
| NE sub-zone | (5, 4) | 3.5 m | Medium |
| NW sub-zone | (−4, 3) | 3.0 m | Medium |
| SE sub-zone | (3, −5) | 2.5 m | Low |

### Dynamic Elements

| Element | Count | Details |
|:--------|------:|:--------|
| Boids birds | 8 | Altitude 3–7 m, GEOM_SPHERE collision r=0.12 m, LiDAR detectable |
| Wind | Continuous | 0.4 m/s base NE, turbulence σ=35%, 1.5× at 10 m altitude |

### Street Infrastructure (visual-only)

| Item | Count |
|:-----|------:|
| Street light poles (7.5 m) | Both sides of all roads |
| Traffic signals (3-light) | One per grid intersection (9 total) |
| Utility poles + overhead lines | Along all roads |
| Parked cars | Lots + roadside spots |
| Benches | Park and plaza blocks |
| Bus stop shelters | 3 road-edge positions |
| Trash bins | Near benches |
| Sidewalk trees | Varied trunk height 1.8–4.2 m, 5 green shades |

### Landmark Extras (Downtown-Specific)

- Raised stone plaza platform (4 × 4 m, h=0.36 m)
- Central fountain base + water column
- Four corner planters with round topiary shrubs

### Performance

| Metric | Value |
|:-------|:------|
| 3-drone FPS (headless) | **126.4** |
| 5-drone FPS (headless) | **108.9** |
| FPS target (≥100) | PASS |

### Research Applications

1. **Occlusion stress test** — Tall buildings (up to 18 m) frequently block drone LiDAR rays; tests how algorithms infer positions from partial observations.
2. **Urban canyon navigation** — 2 m road width and dense block grid create confined flight corridors.
3. **Bus-stop crowd behaviour** — Agents cluster and disperse at fixed road-edge positions, creating localised density spikes.
4. **Multi-target tracking** — Sidewalk walkers follow predictable strip paths; good for tracking algorithm validation with moderate crowd.

### Recommended UAV Configuration

| Setup | Drone Count | Purpose |
|:------|:------------|:--------|
| Patrol / tracking | 1–3 | Surveillance of a single street or plaza |
| Coverage sweep | 5–8 | Full city block coverage in formation |
| Stress test (max tested) | 10 | Formation coordination under high occlusion |

---

## Scenario: Event Crowd Control

### Environment Composition

| Property | Value |
|:---------|:------|
| City grid | 3 × 3 blocks (wider block size) |
| City size | 26 × 26 m |
| Block size | 7.0 m |
| Road width | 2.0 m |
| Building density | Medium |
| Building height | 4–12 m |
| Arena XY bound | 15 m |

### Crowd Dynamics

| Property | Value |
|:---------|:------|
| Total crowd agents | **70** |
| Plaza crowd (GATHER) | 50 in central plaza block, slow wander at 0.55× speed |
| Sidewalk crowd (WALK) | 20 along road-parallel strips |
| Agent speed | 0.8–1.5 m/s (sidewalk); ≈0.44–0.83 m/s (plaza gather) |
| LiDAR detectable | Yes |
| Dominant state | GATHER (50 agents slowly wander within plaza interior) |

### Crowd Zone Geometry

| Zone | Centre | Radius | Density |
|:-----|:-------|-------:|:--------|
| Main event area | (−2, 0) | 5.5 m | Very High |
| East tent zone | (4, 3.5) | 2.5 m | High |
| West tent zone | (−4.5, 3.5) | 2.5 m | High |
| South approach | (0, −6.5) | 2.0 m | Medium |

### Dynamic Elements

| Element | Count | Details |
|:--------|------:|:--------|
| Boids birds | 8 | Altitude 3–7 m |
| Wind | Continuous | Same as downtown defaults |

### Landmark Extras (Event-Specific)

- Grand plaza stone surface (5.5 × 5.5 m)
- Stage platform with backdrop wall (3 × 2.4 m footprint, 1 m high)
- Three spotlight cones above stage
- Three event tent canopies with structural poles
- Three crowd barrier fence sections (front row + two wings)

### Performance

| Metric | Value |
|:-------|:------|
| 3-drone FPS (headless) | **132.7** |
| 5-drone FPS (headless) | **118.1** |
| FPS target (≥100) | PASS |

**FPS note:** 70 collidable crowd agents is the highest body count of all scenarios. Sphere collision shapes reduce raycast cost vs. cylinders. Event is the most LiDAR-intensive scenario.

### Research Applications

1. **Crowd density estimation** — Densest pedestrian concentration; ideal for testing algorithms that estimate local density from LiDAR point clouds.
2. **Target tracking under occlusion** — Agents at 0.15 m radius are below LiDAR angular resolution at high altitude; drones must descend to detect individuals.
3. **Formation control over gathering** — Optimal drone spread to maximise crowd coverage is a non-trivial multi-agent problem.
4. **Boundary / barrier effects** — Crowd barriers create sub-zones within the plaza that constrain agent movement.

### Recommended UAV Configuration

| Setup | Drone Count | Purpose |
|:------|:------------|:--------|
| Dense crowd tracking | 3–5 | Monitor plaza from above |
| Multi-zone coverage | 8 | Cover plaza + three tent zones simultaneously |
| Maximum density stress | 10 | Maximum body count load test |

---

## Scenario: Residential Monitoring

### Environment Composition

| Property | Value |
|:---------|:------|
| City grid | 3 × 3 blocks |
| City size | 24 × 24 m |
| Block size | 6.0 m |
| Road width | 2.0 m |
| Building density | Medium |
| Building height | 3–9 m |
| Arena XY bound | 14 m |

### Crowd Dynamics

| Property | Value |
|:---------|:------|
| Total crowd agents | **8** |
| Park visitors (GATHER) | 5 wandering in park block interior |
| Sidewalk walkers (WALK) | 3 on road-parallel strips |
| Agent speed | 0.8–1.5 m/s |
| LiDAR detectable | Yes |

### Crowd Zone Geometry

| Zone | Centre | Radius | Density |
|:-----|:-------|-------:|:--------|
| Central streets | (0, 0) | 5.0 m | Low |
| NE park | (5.5, 5.5) | 3.0 m | Medium |
| NW park | (−5.5, 5.5) | 2.5 m | Low |

### Dynamic Elements

| Element | Count | Details |
|:--------|------:|:--------|
| Boids birds | 8 | Altitude 3–7 m |
| Wind | Continuous | Configurable |

### Landmark Extras (Residential-Specific)

- Playground slide (red visual box)
- Swing frame with A-frame poles and crossbar
- Park bench near playground

### Performance

| Metric | Value |
|:-------|:------|
| 3-drone FPS (headless) | **193.8** |
| 5-drone FPS (headless) | **134.7** |
| FPS target (≥100) | PASS |

**FPS note:** Lowest crowd count + lowest building height = minimum raycast occlusion. Fastest scenario for training.

### Research Applications

1. **Coverage optimisation** — Wide-open sightlines test maximum area coverage algorithms with minimal interference.
2. **Sparse target tracking** — Low agent count requires active search strategies rather than passive monitoring.
3. **Algorithm baseline** — Cleanest environment for isolating algorithm performance from environmental complexity.
4. **Park-gathering behaviour** — GATHER state agents cluster and wander within a bounded block; tests area-monitoring policies.

### Recommended UAV Configuration

| Setup | Drone Count | Purpose |
|:------|:------------|:--------|
| Baseline single-agent | 1 | Individual performance baseline |
| Coverage swarm | 3–5 | Area coverage with sparse targets |
| Training default | 3 | Standard training configuration |

---

## Scenario: Mixed Urban Area

### Environment Composition

| Property | Value |
|:---------|:------|
| City grid | 3 × 3 blocks |
| City size | 24 × 24 m |
| Block size | 6.0 m |
| Road width | 2.0 m |
| Building density | Medium |
| Building height | 3–14 m |
| Arena XY bound | 14 m |

### Crowd Dynamics

| Property | Value |
|:---------|:------|
| Total crowd agents | **35** |
| Sidewalk walkers (WALK) | 25 across all road strips |
| Park gatherers (GATHER) | 10 in park block interiors |
| Agent speed | 0.8–1.5 m/s |
| LiDAR detectable | Yes |

### Crowd Zone Geometry

| Zone | Centre | Radius | Density |
|:-----|:-------|-------:|:--------|
| Central mixed | (0, 0) | 5.0 m | Medium |
| SE residential | (5.5, −5.5) | 3.0 m | Low |
| NW commercial | (−5, 5) | 3.0 m | Medium |
| Market strip | (0, 6.5) | 2.0 m | High |

### Dynamic Elements

| Element | Count | Details |
|:--------|------:|:--------|
| Boids birds | 8 | Altitude 3–7 m |
| Wind | Continuous | Configurable |

### Landmark Extras (Mixed-Specific)

- Three street market stalls with canopy, pole, and goods display table
- Market area at northern zone (high-density crowd sub-zone)

### Performance

| Metric | Value |
|:-------|:------|
| 3-drone FPS (headless) | **153.8** |
| 5-drone FPS (headless) | **117.2** |
| FPS target (≥100) | PASS |

### Research Applications

1. **General-purpose testing** — Intermediate complexity makes this the best default scenario for initial algorithm development.
2. **Heterogeneous crowd** — Mix of sidewalk walkers and park gatherers tests algorithms that must handle different crowd states simultaneously.
3. **Height variation** — Building range 3–14 m creates varied occlusion — neither as severe as downtown nor as clear as residential.
4. **Market zone density** — High-density sub-zone at market stalls creates localised crowd spike within an otherwise moderate scene.

### Recommended UAV Configuration

| Setup | Drone Count | Purpose |
|:------|:------------|:--------|
| Standard research | 3–5 | Algorithm development default |
| Cross-zone coverage | 5–8 | Cover market + park + central simultaneously |

---

## Scenario: Industrial Zone

### Environment Composition

| Property | Value |
|:---------|:------|
| City grid | 3 × 3 blocks (larger block, wider road) |
| City size | 26 × 26 m |
| Block size | 8.0 m |
| Road width | 3.0 m |
| Building density | Low |
| Building height | 4–10 m |
| Arena XY bound | 15 m |

### Crowd Dynamics

| Property | Value |
|:---------|:------|
| Total crowd agents | **10** |
| Sidewalk workers (WALK) | 8 on road-parallel strips |
| Yard workers (GATHER/WALK) | 2 in open block interiors |
| Agent speed | 0.8–1.5 m/s |
| LiDAR detectable | Yes |

### Crowd Zone Geometry

| Zone | Centre | Radius | Density |
|:-----|:-------|-------:|:--------|
| Central yard | (0, 0) | 4.0 m | Low |
| NE yard | (5, 5) | 2.0 m | Low |

### Dynamic Elements

| Element | Count | Details |
|:--------|------:|:--------|
| Boids birds | 8 | Altitude 3–7 m |
| Wind | Continuous | Configurable |

### Landmark Extras (Industrial-Specific)

- **Shipping containers** — Two yard clusters (SW and SE), 3×3 grid each, stacked 1–2 layers; 5 colour variants (red, blue, green, yellow, grey)
- **Industrial crane** — Vertical mast (r=0.18 m, h=9 m) + horizontal jib arm at (−9, −5)
- **Fuel/chemical tanks** — Two large cylindrical tanks with dome caps at (8.5, 6) and (10, 6)
- **Perimeter fence** — Posts every 2 m around city boundary

### Performance

| Metric | Value |
|:-------|:------|
| 3-drone FPS (headless) | **179.5** |
| 5-drone FPS (headless) | **133.5** |
| FPS target (≥100) | PASS |

**FPS note:** Low building density + low crowd count = fewest ray intersections. Second-fastest scenario after residential.

### Research Applications

1. **Long-range tracking** — Wide roads and low buildings let drones maintain visual contact over greater distances.
2. **Container obstacle patterns** — Stacked containers create irregular, non-building obstacle geometry not seen in urban scenarios.
3. **Sparse worker surveillance** — Low agent count simulates industrial site monitoring where few workers are spread over a wide area.
4. **Perimeter monitoring** — Fence ring creates a natural patrol route for formation control experiments.

### Recommended UAV Configuration

| Setup | Drone Count | Purpose |
|:------|:------------|:--------|
| Perimeter patrol | 2–3 | Follow fence boundary |
| Wide-area coverage | 5–8 | Cover full 26×26 m yard |
| Crane avoidance test | 3–5 | Waypoint through tall crane structure |

---

## Cross-Scenario Comparison

| Scenario | Agents | Birds | Buildings | 3-Drone FPS | 5-Drone FPS | Complexity |
|:---------|-------:|------:|----------:|------------:|------------:|:-----------|
| Downtown | 15 | 8 | High | 126.4 | 108.9 | High |
| Event | 70 | 8 | Medium | 132.7 | 118.1 | Very High |
| Residential | 8 | 8 | Medium | 193.8 | 134.7 | Low |
| Mixed | 35 | 8 | Medium | 153.8 | 117.2 | Medium |
| Industrial | 10 | 8 | Low | 179.5 | 133.5 | Medium |

## Shared Across All Scenarios

### Crowd Agent Physical Model

```
Body:   GEOM_SPHERE collision r=0.15 m (collidable, LiDAR-detectable)
        GEOM_CYLINDER visual r=0.12 m, h=1.16 m (torso)
Head:   GEOM_SPHERE visual r=0.10 m (no collision)
Mass:   0 (kinematic — position set each step via resetBasePositionAndOrientation)
```

### Bird Physical Model

```
Body:   GEOM_SPHERE collision r=0.12 m (collidable)
        GEOM_BOX visual 0.15×0.06×0.04 m, oriented toward velocity
Mass:   0 (kinematic)
Boids:  separation r=2m, alignment r=4m, cohesion r=6m
        max speed 4 m/s, max steer 2 m/s²
```

### Sidewalk Strip Geometry

Strips are generated parallel to each road at offset = `road_half + 0.4 m` (sidewalk half-width 0.4 m). Waypoints placed every 2 m along strip length, filtered for building interiors. Each WALK-state agent is assigned to one strip and travels between randomly sampled waypoints on that strip.

### Wind Force Model

```
At position p, step t:
  base_vec = direction_unit × base_speed
  altitude_boost = altitude_factor × (p.z / 10.0)
  gust = N(0, turbulence × base_speed) per axis
  F_total = drone_mass × (base_vec × (1 + altitude_boost) + gust)
```

Applied via `p.applyExternalForce()` each physics step.
