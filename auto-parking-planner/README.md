# Auto Parking Planner

A professional, self-contained, browser-based surface-parking layout planner.
It generates — and continuously regenerates — an optimized parking layout
whenever the land, building, setbacks, sidewalks, parking standards or access
points change.

> **Disclaimer** — This application is an early-stage planning and feasibility
> tool. Final compliance must be verified against the applicable municipality,
> Saudi Building Code, accessibility requirements, Civil Defense requirements,
> and project-specific authority approvals.

---

## 1. Running the application

No build step, no server, no dependencies, no API keys.

```
auto-parking-planner/
├── index.html      ← open this file in a browser
├── styles.css      ← application chrome styling
├── app.js          ← all logic (geometry, generator, renderer, UI)
└── README.md
```

**Open `index.html` directly in any modern browser** (Chrome, Edge, Firefox,
Safari — desktop or tablet). The app loads a working demo automatically:

* Land 60 × 45 m, road (20 m) along the south boundary
* Building 24 × 18 m, centred slightly toward the north
* Front setback 6 m, other setbacks 3 m, 2 m sidewalk around the building
* Stall 2.50 × 5.50 m, two-way aisle 6 m
* Demand: 1 space / 30 m² on 1 200 m² GFA → **40 required spaces**

With these exact standards the site legally fits about 34 stalls, so the demo
deliberately opens showing a deficit — the compliance engine reporting real
geometry rather than painting stalls inside drive aisles. Widen the land,
relax a setback, or drag the building and watch the balance change live.

Your session is autosaved to browser storage; use **Export → Project JSON**
for a portable save file.

## 2. Interface

| Area | Content |
|------|---------|
| **Left panel** | Collapsible input sections: project info, land boundary, roads, building, setbacks, sidewalks, parking standards, parking demand, access & circulation, optimization, layers, presets, export |
| **Centre** | Interactive SVG site plan with toolbar (select / pan / measure / dimension / zone tools / manual stall / access), zoom, scale presets (1:100 … 1:500), grid & snapping |
| **Right panel** | Live KPI cards, compliance banner, warnings, required-vs-proposed summary table, generated design summary text |

**The key behaviour:** drag, resize or rotate the building on the plan (or
change any regulation input) and the parking rows, counts, KPIs and compliance
status regenerate automatically — a lightweight preview while dragging, the
full optimizer on release.

## 3. Parking-generation algorithm (summary)

1. **Site context** — the land polygon (rectangle or custom polygon drawn by
   clicking; area via the shoelace formula) is analysed into edges with roles
   (front/rear/left/right from the road designated *front*). Each edge gets a
   parking-exclusion inset = max(setback, road setback, street sidewalk,
   landscape buffer, wall clearance).
2. **Obstacles** — each building is expanded by its sidewalk plus
   building-to-parking / building-to-aisle clearances; user no-parking and
   landscape zones are added; each access point reserves a *throat* clear zone
   and casts a **spine aisle** (≥ main-aisle width) from the entrance into the
   site — this is the circulation backbone.
3. **Candidate layouts** — for every enabled row orientation (0°, 90°,
   longest-edge, building-parallel, custom) × parking angle (90/60/45/parallel,
   or "auto" = all) × packing direction, the valid area is filled with
   **modules** (stall row + drive aisle + stall row for double-loading; row +
   aisle for single). Aisle widths come from the editable standards per angle;
   angled rows derive pitch `w/sin θ` and depth `l·sin θ + w·cos θ` from the
   stall geometry.
4. **Stall validation** — every candidate stall is kept only if it is fully
   inside the boundary insets, and does not intersect the building+sidewalk,
   zones, entrance throats or spine aisles. Aisles are split into segments
   around the building; a segment (and its stalls) is kept only if it connects
   to a spine — disconnected stalls are removed and reported.
5. **Accessible & EV** — accessible stalls (with shared access-aisle slots)
   are reserved nearest the building entrance on the front side; EV stalls are
   tagged next by the entered percentage.
6. **Scoring & options** — every candidate is scored with editable weights
   (stall count, demand compliance, circulation quality, dead-ends, accessible
   proximity, landscape preservation, geometry simplicity). Three options are
   kept: **A** maximum parking, **B** best weighted score, **C** maximum
   landscape (re-run with an extra perimeter landscape inset and a
   landscape-heavy score).
7. **Manual edits** — delete stalls/rows, convert to accessible/EV, shift or
   reverse rows, add manual stalls, draw no-parking / landscape / crossing
   zones. Edits are re-applied after every automatic regeneration and are
   cleared by **Regenerate** (edits only) or **Reset** (edits + zones +
   dimensions).

## 4. Compliance engine

Warnings are colour-coded (green compliant / amber warning / red
non-compliant) and cover: road below minimum width, building setback
violations, coverage above maximum, parking deficit, insufficient
accessible/EV stalls, sub-standard aisle widths, entrance too close to a
corner, access-point spacing, disconnected circulation, dead-end aisles beyond
the maximum, stalls intersecting crossings, and landscape below the required
percentage. No unverified legal values are hard-coded — every threshold is an
editable field, and presets (including the example *"Saudi Municipality —
Custom Project Preset"*) are stored in browser localStorage and can be saved,
loaded, duplicated, exported and imported as JSON.

## 5. Export

* **SVG** — standalone plan with border, title block (project, date, scale,
  parking summary), north arrow and scale bar
* **PNG** — 2× raster of the same sheet
* **Print / PDF** — printable sheet via the browser's print dialog
* **CSV** — parking schedule (one row per stall: type, position, orientation, size)
* **Project JSON** — full state, settings, manual edits and alternative
  summaries; re-importable

## 6. Known technical limitations

* Polygon insetting uses successive half-plane clips — exact for rectangles
  and convex sites, **conservative (may over-clip) for concave sites**.
* One row orientation is used per layout; mixed-orientation hybrid layouts
  (e.g. perimeter parallel + interior 90°) are not yet generated.
* Aisle segments are split around obstacle *bounding boxes* in the row
  frame (conservative for strongly rotated buildings) and clipped to the
  developable polygon; paved-area statistics subtract pairwise aisle
  overlaps, so only rare triple overlaps are still counted twice.
* Circulation is modelled as entrance spines + row aisles; a full network
  graph (loop roads, one-way systems, fire-truck turning circles) is not
  simulated. Fire-route checking is limited to a width comparison.
* Stall-level manual edits are keyed to the generated grid; if inputs change
  enough that the grid shifts, some edits may no longer find their stall
  (zones and manual stalls always survive).
* Accessible stall width is provided via a shared access-aisle slot (a full
  stall slot is hatched as an aisle), rather than by re-spacing the row; the
  CSV schedule and selection card therefore report the drawn slot size, not
  the nominal accessible-stall inputs.
* Latitude/longitude are metadata only — by design, geographic coordinates
  are not used for geometry (no GIS projection).

## 7. Suggested next version

1. True polygon offsetting (straight-skeleton or Clipper-style) for concave
   sites, replacing the half-plane inset.
2. Mixed-orientation solutions: independent fills of the sub-areas left/right/
   above/below the building, then merged.
3. A real circulation graph with loop detection, one-way flow solving and
   fire-truck sweep checks.
4. Multi-storey / basement parking levels and ramp placement.
5. DXF export for CAD interoperability.
6. Curb-cut geometry with turning radii at entrances.
7. Landscape-island auto-insertion every N stalls (municipality rule).
8. Web-worker optimization for very large sites.
